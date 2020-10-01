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

#define	BH1750_POLLTIME			5	/* in seconds */

/* Number of bits in high or low parts of MTreg */
#define BH1750_MTREG_BYTE_LEN		5

#define BH1750_POWER_DOWN		0x00
#define BH1750_POWER_ON			0x01
#define BH1750_RESET			0x07
#define BH1750_MTREG_H_BYTE		0x40
#define BH1750_MTREG_L_BYTE		0x60

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

/* chip mtreg parameters */
struct bh1750_mtreg_t {
    uint16_t val_min;
    uint16_t val_max;
    uint16_t val_default;
    uint16_t step_usec;
};
struct bh1750_mode_t {
    uint8_t opecode;
    uint16_t k;
};

/* step_usec = ceil(chip_data_ready_usec / mtreg_value) */
static const struct bh1750_mtreg_t
bh1750_mtreg_params =
{
    .val_min = 0x1f,
    .val_max = 0xfe,
    .val_default = 0x45,
    .step_usec = 0x6cc
};

/* For H-resolution:
   milli_lux = (counts / 1.2) * (MTReg_default / MTReg) * 1000
   thus,
   k = MTReg_default/1.2 * 1000
 */
static const struct bh1750_mode_t
bh1750_mode_params[] =
{
    { .opecode = 0,    .k = 0 },
    { .opecode = 0x20, .k = 0xe09c },
    { .opecode = 0x21, .k = 0x704e }
};

struct bh1750_softc {
    device_t			 dev;
    phandle_t			 node;
    uint8_t			 addr;
    uint8_t			 quality_lack;
    uint8_t			 polltime;
    uint8_t			 hres_mode;
    uint16_t			 mtreg;
    uint16_t			 counts;
    uint16_t			 k;
    uint16_t			 sensitivity;
    bool			 detaching;
    unsigned long		 illuminance;
    unsigned long		 ready_time;
    const struct bh1750_mtreg_t	*mtreg_params;
    const struct bh1750_mode_t	*mode_params;
    struct timeout_task		 task;
};

static int bh1750_probe(device_t);
static int bh1750_attach(device_t);
static int bh1750_detach(device_t);

static void bh1750_sysctl_register(struct bh1750_softc*);
static void bh1750_poll(void*, int);
static void bh1750_start(void *sc);

static int bh1750_fdt_get_params(struct bh1750_softc*);
static int bh1750_set_mtreg(struct bh1750_softc*, uint16_t, bool);
static int bh1750_set_hres_mode(struct bh1750_softc*, uint8_t, bool);
static int bh1750_set_quality(struct bh1750_softc*, uint8_t);
static int bh1750_set_polltime(struct bh1750_softc*, uint8_t);

static int bh1750_mtreg_sysctl(SYSCTL_HANDLER_ARGS);
static int bh1750_hres_mode_sysctl(SYSCTL_HANDLER_ARGS);
static int bh1750_quality_sysctl(SYSCTL_HANDLER_ARGS);

static int bh1750_write(struct bh1750_softc*, uint8_t opecode);
static int bh1750_write_mtreg(struct bh1750_softc*);

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
	sc->mode_params = bh1750_mode_params;

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
		    sc->polltime * SBT_1S, 0, C_PREL(3));
}

/* Sets the time period of polling */
static int
bh1750_set_polltime(struct bh1750_softc *sc, uint8_t polltime)
{
	/* Time period for updating the results */
	if (polltime == 0)
		return (-1);

	sc->polltime = polltime;

	return (0);
}

/* Sets the time penalty in percents for low-quality chips */
static int
bh1750_set_quality(struct bh1750_softc *sc, uint8_t quality_lack)
{
	/* up to 50% by the datasheet */
	if (quality_lack > 50)
		return (-1);

	/* Lack of quality leads to increasing of ready-time */
	sc->quality_lack = quality_lack;
	sc->ready_time = sc->mtreg * sc->mtreg_params->step_usec \
	    * (100 + sc->quality_lack) / 100;

	return (0);
}

/* Sets the time penalty in percents for low-quality chips */
static int
bh1750_set_hres_mode(struct bh1750_softc *sc, uint8_t hres_mode, bool task_interrupt)
{
	/* H-resolution mode or mode2 only */
	if ((hres_mode > 2) || (hres_mode == 0))
		return (-1);

	if (task_interrupt)
		/* to be sure the task with old MTreg is canceled or finished */
		while (taskqueue_cancel_timeout(taskqueue_thread, &sc->task, NULL) != 0)
			taskqueue_drain_timeout(taskqueue_thread, &sc->task);

	/* Different modes have different sensivities */
	sc->hres_mode = hres_mode;
	sc->k = sc->mode_params[hres_mode].k;
	sc->sensitivity = sc->k / sc->mtreg;

	if (task_interrupt)
		/* Get the measurement value and start the polling again */
		bh1750_poll(sc, 0);

	return (0);
}

/* Set MTreg value and recalculate dependent parameters */
static int
bh1750_set_mtreg(struct bh1750_softc *sc, uint16_t mtreg_val, bool task_interrupt)
{
	int error;

	/* Check the MTreg range */
	if (mtreg_val < sc->mtreg_params->val_min ||
	    mtreg_val > sc->mtreg_params->val_max)
		return (EINVAL);

	if (task_interrupt)
		/* to be sure the task with old MTreg is canceled or finished */
		while (taskqueue_cancel_timeout(taskqueue_thread, &sc->task, NULL) != 0)
			taskqueue_drain_timeout(taskqueue_thread, &sc->task);

	/* set and recalculate */
	sc->mtreg = mtreg_val;
	sc->sensitivity = sc->k / mtreg_val;
	sc->ready_time = mtreg_val * sc->mtreg_params->step_usec \
	    * (100 + sc->quality_lack) / 100;

	/* Write the MTreg to the sensor. If error start the polling anyway */
	error = bh1750_write_mtreg(sc);
	if (error)
	    device_printf(sc->dev,
		"could not write MTreg value\n");

	if (task_interrupt)
		/* Get the measurement value and start the polling again */
		bh1750_poll(sc, 0);

	return (0);
}

/* write MTreg value to the sensor */
static int
bh1750_write_mtreg(struct bh1750_softc *sc)
{
	int error;
	uint8_t mt_byte;

	// Power down the device before changes
	error = bh1750_write(sc, BH1750_POWER_DOWN);
	if (error)
		return (error);

	// Transfer hight byte of MTreg
	mt_byte = sc->mtreg >> BH1750_MTREG_BYTE_LEN;
	error = bh1750_write(sc, BH1750_MTREG_H_BYTE | mt_byte);
	if (error)
		return (error);

	// Transfer low byte of MTreg
	mt_byte = sc->mtreg & ((0x01 << BH1750_MTREG_BYTE_LEN) - 1);
	error = bh1750_write(sc, BH1750_MTREG_L_BYTE | mt_byte);

	return (error);
}

/* Write command */
static int
bh1750_write(struct bh1750_softc *sc, uint8_t opecode)
{
	int error = 0;
	struct iic_msg msg;

	msg.slave = sc->addr;
	msg.flags = IIC_M_WR;
	msg.len   = 1;
	msg.buf   = &opecode;

	error = iicbus_transfer_excl(sc->dev, &msg, 1, IIC_INTRWAIT);

	return (error);
}

/* Read data */
static int
bh1750_read(struct bh1750_softc *sc, uint16_t *result)
{
	int error = 0;
	uint16_t be_result;
	struct iic_msg msg;

	msg.slave = sc->addr;
	msg.flags = IIC_M_RD;
	msg.len   = 2;
	msg.buf   = (uint8_t *)&be_result;

	error = iicbus_transfer_excl(sc->dev, &msg, 1, IIC_INTRWAIT);
	if (!error)
		*result = be16toh(be_result);

	return (error);
}

/* Read the sensor data*/
static int
bh1750_read_data(struct bh1750_softc *sc)
{
	if (bh1750_write(sc, sc->mode_params[sc->hres_mode].opecode) != 0)
		return (-1);

	DELAY(sc->ready_time);

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
	int error;

	error = SYSCTL_OUT(req, &_mtreg, sizeof(_mtreg));
	if (error != 0 || req->newptr == NULL)
		return (error);

	error = SYSCTL_IN(req, &_mtreg, sizeof(_mtreg));
	if (error != 0)
		return (error);

	error = bh1750_set_mtreg(sc, _mtreg, true);

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
	if (bh1750_set_quality(sc, _lack) != 0)
		error = EINVAL;

	return (error);
}

static int
bh1750_hres_mode_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct bh1750_softc *sc = arg1;
	uint8_t _hres_mode = (uint8_t)sc->hres_mode;
	int error = 0;

	error = SYSCTL_OUT(req, &_hres_mode, sizeof(_hres_mode));
	if (error != 0 || req->newptr == NULL)
		return (error);

	error = SYSCTL_IN(req, &_hres_mode, sizeof(_hres_mode));
	if (error != 0)
		return (error);

	/* Lack of quality from 0 to 50 percent by the datasheet */
	if (bh1750_set_hres_mode(sc, _hres_mode, true) != 0)
		error = EINVAL;

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

	SYSCTL_ADD_U8(ctx, tree, OID_AUTO, "polling-time",
	    CTLFLAG_RD,
	    &sc->polltime, 0, "polling period from 1 to 255, s");

	SYSCTL_ADD_U16(ctx, tree, OID_AUTO, "sensitivity",
	    CTLFLAG_RD,
	    &sc->sensitivity, 0, "measure sensitivity, mlx/counts");

	SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "illuminance",
	    CTLFLAG_RD,
	    &sc->illuminance, "light intensity, mlx");

	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "mtreg",
	    CTLTYPE_U16 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    &bh1750_mtreg_sysctl, "CU", "MTreg value");

	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "quality-lack",
	    CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    &bh1750_quality_sysctl, "CU", "lack of quality from 0 to 50, %");

	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "hres-mode",
	    CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    &bh1750_hres_mode_sysctl, "CU", "H-resolution mode, 1 or 2");

	SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "ready-time",
	    CTLFLAG_RD,
	    &sc->ready_time, "measurement time, usec");

}

/* Get MTreg properties if set from a device node */
static int
bh1750_fdt_get_params(struct bh1750_softc *sc)
{
	pcell_t param_cell;
	ssize_t param_found;
	uint16_t mtreg;
	uint8_t param8;

	/* If no "mtreg" parameter is set the default value will be applied */
	param_found = OF_getencprop(sc->node, "mtreg", &param_cell, sizeof(param_cell));
	if (param_found > 0)
	{
		mtreg = (uint16_t)param_cell;
		if ((mtreg != param_cell) || (bh1750_set_mtreg(sc, mtreg, false) != 0))
		{
			device_printf(sc->dev,
			    "could not acquire correct MTreg value from DTS\n");

			return (EINVAL);
		}
		if (bootverbose)
			device_printf(sc->dev,
			    "Acquired MTreg: 0x%0x from DTS\n", sc->mtreg);
	}
	else
	{
		bh1750_set_mtreg(sc, sc->mtreg_params->val_default, false);
		if (bootverbose)
			device_printf(sc->dev,
			    "Acquired MTreg: 0x%0x by default\n", sc->mtreg);
	}

	/* Set H-resolution mode if found */
	param_found = OF_getencprop(sc->node, "hres-mode", &param_cell, sizeof(param_cell));
	if (param_found > 0)
	{
		param8 = (uint8_t)param_cell;
		if ((param8 != param_cell) || (bh1750_set_hres_mode(sc, param8, false) != 0))
		{
			device_printf(sc->dev,
			    "could not acquire correct H-resulution mode value from DTS\n");

			return (EINVAL);
		}
		if (bootverbose)
			device_printf(sc->dev,
			    "Acquired H-resolution mode: %u from DTS\n", sc->hres_mode);
	}
	else
	{
		bh1750_set_hres_mode(sc, 1, false);
		if (bootverbose)
			device_printf(sc->dev,
			    "Acquired H-resolution mode: %u by default\n", sc->hres_mode);
	}

	/* Set quality lack percent if found */
	param_found = OF_getencprop(sc->node, "quality-lack", &param_cell, sizeof(param_cell));
	if (param_found > 0)
	{
		param8 = (uint8_t)param_cell;
		if ((param8 != param_cell) || (bh1750_set_quality(sc, param8) != 0))
		{
			device_printf(sc->dev,
			    "could not acquire correct quality-lack value from DTS\n");

			return (EINVAL);
		}
		if (bootverbose)
			device_printf(sc->dev,
			    "Acquired quality-lack: %u%% from DTS\n", sc->quality_lack);
	}
	else
	{
		bh1750_set_quality(sc, 0);
		if (bootverbose)
			device_printf(sc->dev,
			    "Acquired quality-lack: %u%% by default\n", sc->quality_lack);
	}

	/* Set polling period if found */
	param_found = OF_getencprop(sc->node, "polling-time", &param_cell, sizeof(param_cell));
	if (param_found > 0)
	{
		param8 = (uint8_t)param_cell;
		if ((param8 != param_cell) || (bh1750_set_polltime(sc, param8) != 0))
		{
			device_printf(sc->dev,
			    "could not acquire correct polling-time value from DTS\n");

			return (EINVAL);
		}
		if (bootverbose)
			device_printf(sc->dev,
			    "Acquired polling-time: %us from DTS\n", sc->polltime);
	}
	else
	{
		bh1750_set_polltime(sc, BH1750_POLLTIME);
		if (bootverbose)
			device_printf(sc->dev,
			    "Acquired polling-time: %us by default\n", sc->polltime);
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

	/* Get parameters from fdt */
	bh1750_fdt_get_params(sc);

	/* Init the polling task */
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
