/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2020, Alexander Mishin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#define FDT

#include <sys/types.h>
#include <sys/endian.h>
#include <sys/module.h>
#include <sys/systm.h>  /* uprintf */
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/conf.h>   /* cdevsw struct */
#include <sys/param.h>  /* defines used in kernel.h */
#include <sys/kernel.h> /* types used in module initialization */
#include <sys/bus.h>
#include <sys/uio.h>    /* uio struct */

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#define DIV_CEIL(a, b) (((a) / (b)) + (((a) % (b)) > 0 ? 1 : 0))
#define DIV_ROUND_UINT(a, b) ((((a) + (b)/2) / (b)))

#define	BH1750_POLLTIME			5	/* in seconds */

/* Number of bits in high or low parts of MTreg */
#define BH1750_MTREG_BYTE_LEN		5

#define BH1750_POWER_DOWN		0x00
#define BH1750_POWER_ON			0x01
#define BH1750_ONE_TIME_H_RES_MODE	0x20
#define BH1750_MTREG_H_BYTE		0x40
#define BH1750_MTREG_L_BYTE		0x60
#define BH1750_DATA_READY_TIME		0x80	/* ~120 ms */

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

/* chip mtreg parameters */
struct bh1750_mtreg_t {
    uint16_t val_min;
    uint16_t val_max;
    uint16_t val_default;
    unsigned long step_usec;
};

/* step_usec = ceil(chip_data_ready_usec / mtreg_value) */
static const struct bh1750_mtreg_t
bh1750_mtreg_params = { 0x1f, 0xfe, 0x45, DIV_CEIL(120000, 0x45) };

struct bh1750_softc {
    device_t			 dev;
    phandle_t			 node;
    uint8_t			 addr;
    uint8_t			 quality_lack;
    bool			 detaching;
    uint16_t			 mtreg;
    uint16_t			 counts;
    unsigned long		 illuminance;
    unsigned long		 sensitivity;
    unsigned long		 ready_time;
    unsigned long		 k;
    const struct bh1750_mtreg_t	*mtreg_params;
    struct timeout_task		 task;
};

static int bh1750_probe(device_t);
static int bh1750_attach(device_t);
static int bh1750_detach(device_t);

static void bh1750_sysctl_register(struct bh1750_softc*);
static void bh1750_poll(void*, int);
static void bh1750_start(void *sc);

static int bh1750_fdt_get_params(struct bh1750_softc*);
static int bh1750_set_mtreg(struct bh1750_softc*, uint16_t);
static int bh1750_set_quality(struct bh1750_softc*, uint16_t);
static int bh1750_mtreg_sysctl(SYSCTL_HANDLER_ARGS);

static int bh1750_write(struct bh1750_softc*, uint8_t opecode);
static int bh1750_read(struct bh1750_softc*, uint16_t* data);
static int bh1750_read_data(struct bh1750_softc*);

static const struct ofw_compat_data bh1750_compat_data[] = {
    {"bh1750", (uintptr_t)"BH1750 Ambient Light Sensor module"},
    {NULL, false}
};

static device_method_t bh1750_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,	bh1750_probe),
    DEVMETHOD(device_attach,	bh1750_attach),
    DEVMETHOD(device_detach,	bh1750_detach),

    DEVMETHOD_END
};

static devclass_t bh1750_devclass;

DEFINE_CLASS_0(bh1750, bh1750_driver, bh1750_methods, sizeof(struct bh1750_softc));

DRIVER_MODULE(bh1750, iicbus, bh1750_driver, bh1750_devclass, NULL, NULL);
MODULE_VERSION(bh1750, 1);
MODULE_DEPEND(bh1750, iicbus, 1, 1, 1);
IICBUS_FDT_PNP_INFO(bh1750_compat_data);

#endif

/* Device _probe() method */
static int
bh1750_probe(device_t dev)
{

#ifdef FDT
	const struct ofw_compat_data *compat;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	compat = ofw_bus_search_compatible(dev, bh1750_compat_data);

	if (compat->ocd_str == NULL)
		return (ENXIO);

	device_set_desc(dev, (const char *)compat->ocd_data);

	return (BUS_PROBE_DEFAULT);
#else
	device_set_desc(dev, "BH1750 Ambient Light Sensor module");

	return (BUS_PROBE_NOWILDCARD);
#endif

}

/* Device _detach() method */
static int
bh1750_detach(device_t dev)
{
	struct bh1750_softc *sc = device_get_softc(dev);

	sc->detaching = true;
	while (taskqueue_cancel_timeout(taskqueue_thread, &sc->task, NULL) != 0)
		taskqueue_drain_timeout(taskqueue_thread, &sc->task);

	/* Send POWER_DOWN opecode if the device was initialized */
	if (sc->mtreg > 0)
		bh1750_write(sc, BH1750_POWER_DOWN);

	return (0);
}

/* Device _attach() method */
static int
bh1750_attach(device_t dev)
{
	struct bh1750_softc *sc = device_get_softc(dev);

	sc->dev = dev;
	sc->addr = iicbus_get_addr(dev);
	sc->node = ofw_bus_get_node(dev);
	sc->mtreg_params = &bh1750_mtreg_params;

	/*
	 * We have to wait until interrupts are enabled.  Sometimes I2C read
	 * and write only works when the interrupts are available.
	 */
	config_intrhook_oneshot(bh1750_start, sc);

	return (0);
}

static void
bh1750_poll(void *arg, int pending __unused)
{
	struct bh1750_softc *sc = (struct bh1750_softc *)arg;

	bh1750_read_data(sc);
	if (!sc->detaching)
		taskqueue_enqueue_timeout_sbt(taskqueue_thread, &sc->task,
		    BH1750_POLLTIME * SBT_1S, 0, C_PREL(3));
}

static int
bh1750_set_quality(struct bh1750_softc *sc, uint16_t quality_lack)
{
	/* up to 50% by the datasheet */
	if (quality_lack > 50)
		return (-1);

	/* Lack of quality leads to increasing of ready-time */
	sc->quality_lack = quality_lack;
	sc->ready_time = sc->ready_time \
	    * (100 + quality_lack) / 100;

	return (0);
}

static int
bh1750_set_mtreg(struct bh1750_softc *sc, uint16_t mtreg_val)
{
	int ret;
	uint8_t mt_byte;

	// Power down the device before changes
	ret = bh1750_write(sc, BH1750_POWER_DOWN);
	if (ret)
		return (ret);

	// Transfer hight byte of MTreg
	mt_byte = mtreg_val >> BH1750_MTREG_BYTE_LEN;
	ret = bh1750_write(sc, BH1750_MTREG_H_BYTE | mt_byte);
	if (ret)
		return (ret);

	// Transfer low byte of MTreg
	mt_byte = mtreg_val & ((0x01 << BH1750_MTREG_BYTE_LEN) - 1);
	ret = bh1750_write(sc, BH1750_MTREG_L_BYTE | mt_byte);
	if (ret)
		return (ret);

	sc->mtreg = mtreg_val;
	sc->sensitivity = sc->k / mtreg_val;
	sc->ready_time = mtreg_val * sc->mtreg_params->step_usec \
	    * (100 + sc->quality_lack) / 100;

	return (0);
}

/* Write command */
static int
bh1750_write(struct bh1750_softc *sc, uint8_t opecode)
{
    int try = 0;

    struct iic_msg msg[] = {
	{sc->addr, IIC_M_WR, 1, &(opecode)}
    };

    for (;;)
    {
	if (iicbus_transfer(sc->dev, msg, 1) == 0)
	    return (0);
	if (++try > 5) {
	    device_printf(sc->dev, "iicbus write failed\n");
	    return (-1);
	}
	pause("bh1750_write", hz);
    }

    return (0);
}

/* Read data */
static int
bh1750_read(struct bh1750_softc *sc, uint16_t *result)
{
    int try = 0;
    uint16_t be_result;

    struct iic_msg msg[] = {
	{sc->addr, IIC_M_RD, 2, (uint8_t *)&be_result}
    };

    try = 0;
    for (;;)
    {
	if (iicbus_transfer(sc->dev, msg, 1) == 0)
	    break;
	if (++try > 5) {
	    device_printf(sc->dev, "iicbus read failed\n");
	    return (-1);
	}
	pause("bh1750_read", hz);
    }

    *result = be16toh(be_result);

    return (0);
}

/* Read the sensor data*/
static int
bh1750_read_data(struct bh1750_softc *sc)
{
	if (bh1750_write(sc, BH1750_ONE_TIME_H_RES_MODE) != 0)
		return (-1);

	DELAY(DIV_CEIL(sc->ready_time, 1000));

	if (bh1750_read(sc, &sc->counts) != 0)
		return (-1);

	/* milli lux */
	sc->illuminance = sc->k * sc->counts / sc->mtreg;

	return (0);
}

static int
bh1750_mtreg_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct bh1750_softc *sc = arg1;
	uint16_t _mtreg = (uint16_t)sc->mtreg;
	int error = 0;

	error = SYSCTL_OUT(req, &_mtreg, sizeof(_mtreg));
	if (error != 0 || req->newptr == NULL)
		return (error);

	error = SYSCTL_IN(req, &_mtreg, sizeof(_mtreg));
	if (error != 0)
		return (error);

	if (_mtreg < sc->mtreg_params->val_min ||
	    _mtreg > sc->mtreg_params->val_max)
		return (EINVAL);

	error = bh1750_set_mtreg(sc, _mtreg);

	return (error);
}

static int
bh1750_quality_sysctl(SYSCTL_HANDLER_ARGS)
{
    struct bh1750_softc *sc = arg1;
    uint8_t _lack = (uint8_t)sc->quality_lack;
    int error = 0;

    error = SYSCTL_OUT(req, &_lack, sizeof(_lack));
    if (error != 0 || req->newptr == NULL)
	return (error);

    error = SYSCTL_IN(req, &_lack, sizeof(_lack));
    if (error != 0)
	return (error);

    /* Lack of quality from 0 to 50 percent by the datasheet */
    if (_lack > 50)
	error = EINVAL;
    else
	error = bh1750_set_quality(sc, _lack);

    return (error);
}

/* Create sysctl variables and set their handlers */
static void
bh1750_sysctl_register(struct bh1750_softc *sc)
{
    struct sysctl_ctx_list	*ctx;
    struct sysctl_oid		*tree_node;
    struct sysctl_oid_list	*tree;

    ctx = device_get_sysctl_ctx(sc->dev);
    tree_node = device_get_sysctl_tree(sc->dev);
    tree = SYSCTL_CHILDREN(tree_node);

    SYSCTL_ADD_U16(ctx, tree, OID_AUTO, "counts",
	CTLFLAG_RD,
	&sc->counts, 0, "raw measurement data");

    SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "sensitivity",
	CTLFLAG_RD,
	&sc->sensitivity, "measure sensitivity, mlx/counts");

    SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "illuminance",
	CTLFLAG_RD,
	&sc->illuminance, "light intensity, mlx");

    SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "mtreg",
	CTLTYPE_U16 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	&bh1750_mtreg_sysctl, "CU", "MTreg value");

    SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "quality-lack",
	CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	&bh1750_quality_sysctl, "CU", "lack of quality from 0 to 50, %");

    SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "ready-time",
	CTLFLAG_RD,
	&sc->ready_time, "measurement time, usec");

}

/* Get MTreg properties if set from a device node */
static int
bh1750_fdt_get_params(struct bh1750_softc *sc)
{
    pcell_t pmtreg, pquality;
    uint16_t mtreg;
    uint8_t quality;
    int mtreg_found, quality_found;

    /* If no "mtreg" parameter is set the default value will be applied*/
    mtreg_found = OF_getencprop(sc->node, "mtreg", &pmtreg, sizeof(pmtreg));
    if (mtreg_found > 0)
    {
	mtreg = (uint16_t)pmtreg;
	if ((mtreg != pmtreg) || (bh1750_set_mtreg(sc, mtreg) != 0))
	{
	    device_printf(sc->dev,
		"could not acquire correct MTreg value from DTS\n");

	    return (EINVAL);
	}
    }
    else
	bh1750_set_mtreg(sc, sc->mtreg_params->val_default);

    quality_found = OF_getencprop(sc->node, "quality-lack", &pquality, sizeof(pquality));
    if (quality_found > 0)
    {
	quality = (uint16_t)pquality;
//	if ((quality != pquality) || (bh1750_set_quality(sc, quality) != 0))
	if (bh1750_set_quality(sc, quality) != 0)
	{
	    device_printf(sc->dev,
		"could not acquire correct quality-lack value from DTS\n");

	    return (EINVAL);
	}
    }
    else
	bh1750_set_quality(sc, 0);

    /* Print more details */
    if (bootverbose)
    {
	if (mtreg_found > 0)
	    device_printf(sc->dev,
		"Acquired MTreg: 0x%0x from DTS\n", sc->mtreg);
	else
	    device_printf(sc->dev,
		"Acquired MTreg: 0x%0x by default\n", sc->mtreg);

	if (quality_found > 0)
	    device_printf(sc->dev,
		"Acquired quality-lack: 0x%0x from DTS\n", sc->quality_lack);
	else
	    device_printf(sc->dev,
		"Acquired quality-lack: 0x%0x by default\n", sc->quality_lack);
    }

    return (0);
}

static void
bh1750_start(void *arg)
{
	struct bh1750_softc *sc;
	sc = arg;

	/* Check if device is connected */
	if (bh1750_write(sc, BH1750_POWER_ON) != 0) {
		device_printf(sc->dev, "failed to connect to the device\n");
		return;
	}

	/* For H-resolution:
	   milli_lux = counts * 1000 * (10 / 12) * (69 / MTReg)
	 */
	sc->k = 1000 * 10 * sc->mtreg_params->val_default / 12;

	bh1750_fdt_get_params(sc);

	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->task, 0, bh1750_poll, sc);

	/* 
	 * Do an initial read so we have correct values for reporting before
	 * registering the sysctls that can access those values. This also
	 * schedules the periodic polling the driver does every few seconds to
	 * update the sysctl variables.
	 */
	bh1750_poll(sc, 0);

	/* Add sysctl variables */
	bh1750_sysctl_register(sc);
}
