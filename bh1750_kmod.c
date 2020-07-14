/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2020 Alexander Mishin
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

#define BH1750_POWER_DOWN		0x00
#define BH1750_POWER_ON			0x01
#define BH1750_ONE_TIME_H_RES_MODE	0x20
#define BH1750_DATA_READY_TIME		0x80	/* ~120 ms */

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

struct bh1750_softc {
    device_t			 dev;
    uint8_t			 addr;
    phandle_t			 node;
    uint16_t			 measurement;
    struct timeout_task		 task;
    bool			 detaching;
};

static int bh1750_probe(device_t);
static int bh1750_attach(device_t);
static int bh1750_detach(device_t);

static void bh1750_sysctl_register(struct bh1750_softc*);
static void bh1750_poll(void*, int);
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

    return (0);
}

/* Device _attach() method */
static int
bh1750_attach(device_t dev)
{
    struct bh1750_softc *sc;

    sc = device_get_softc(dev);

    sc->dev = dev;
    sc->addr = iicbus_get_addr(dev);
    sc->node = ofw_bus_get_node(dev);

    TIMEOUT_TASK_INIT(taskqueue_thread, &sc->task, 0, bh1750_poll, sc);

    /* 
     * Do an initial read so we have correct values for reporting before
     * registering the sysctls that can access those values.  This also
     * schedules the periodic polling the driver does every few seconds to
     * update the sysctl variables.
     */
    bh1750_poll(sc, 0);

    bh1750_sysctl_register(sc);

    return (0);
}

static void
bh1750_poll(void *arg, int pending __unused)
{
	struct bh1750_softc	*sc;

	sc = (struct bh1750_softc *)arg;

	bh1750_read_data(sc);
	if (!sc->detaching)
		taskqueue_enqueue_timeout_sbt(taskqueue_thread, &sc->task,
		    BH1750_POLLTIME * SBT_1S, 0, C_PREL(3));
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
    bh1750_write(sc, BH1750_ONE_TIME_H_RES_MODE);

    DELAY(BH1750_DATA_READY_TIME);

    bh1750_read(sc, &(sc->measurement));

    return (0);
}

/* Create sysctl variables and set their handlers */
static void
bh1750_sysctl_register(struct bh1750_softc *sc)
{
    struct sysctl_ctx_list	*ctx;
    struct sysctl_oid		*tree;

    ctx = device_get_sysctl_ctx(sc->dev);
    tree = device_get_sysctl_tree(sc->dev);

    SYSCTL_ADD_U16(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	"measurement", CTLFLAG_RD,
	&sc->measurement, 0, "light sensor measurement data");
}
