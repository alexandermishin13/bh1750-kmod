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
#include <sys/module.h>
#include <sys/systm.h>  /* uprintf */
#include <sys/conf.h>   /* cdevsw struct */
#include <sys/param.h>  /* defines used in kernel.h */
#include <sys/kernel.h> /* types used in module initialization */
#include <sys/bus.h>
#include <sys/uio.h>    /* uio struct */

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#define BH1750_CDEV_NAME	"bh1750"

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

struct bh1750_softc {
    device_t			 bh1750_dev;
    uint8_t			 bh1750_addr;
    phandle_t			 bh1750_node;
    struct cdev			*bh1750_cdev;
};

static int bh1750_probe(device_t);
static int bh1750_attach(device_t);
static int bh1750_detach(device_t);

static const struct ofw_compat_data bh1750_compat_data[] = {
    {"bh1750", (uintptr_t)"BH1750 Light Sensor module"},
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

/* Function prototypes */
static d_open_t      bh1750_open;
static d_close_t     bh1750_close;
static d_read_t      bh1750_read;
static d_write_t     bh1750_write;
//static d_ioctl_t     bh1750_ioctl;

/* Character device entry points */
static struct cdevsw bh1750_cdevsw = {
    .d_version = D_VERSION,
    .d_open = bh1750_open,
    .d_close = bh1750_close,
    .d_read = bh1750_read,
    .d_write = bh1750_write,
//    .d_ioctl = bh1750_ioctl,
    .d_name = BH1750_CDEV_NAME,
};

/* Device _probe() method */
static int
tea5767_probe(device_t dev)
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
    device_set_desc(dev, "BH1750 compatible Light Sensor module");

    return (BUS_PROBE_NOWILDCARD);
#endif

}

/* Device _detach() method */
static int
bh1750_detach(device_t dev)
{
    struct bh1750_softc *sc = device_get_softc(dev);

    if (sc->bh1750_cdev != NULL)
	destroy_dev(sc->bh1750_cdev);

//    free(sc->bh1750_control, M_BH1750REG_WRITE);
//    free(sc->bh1750_status, M_BH1750REG_READ);

    return (0);
}

/* Device _attach() method */
static int
bh1750_attach(device_t dev)
{
    struct bh1750_softc	*sc;
    int err;

    sc = device_get_softc(dev);

    sc->bh1750_dev = dev;
    sc->bh1750_addr = iicbus_get_addr(dev);
    sc->bh1750_node = ofw_bus_get_node(dev);

    /* Create the bh1750 cdev */
    err = make_dev_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
	&sc->bh1750_cdev,
	&bh1750_cdevsw,
	0,
	UID_ROOT,
	GID_WHEEL,
	0600,
	BH1750_CDEV_NAME);

    if (err != 0) {
	device_printf(dev, "Unable to create bh1750 cdev\n");
	bh1750_detach(dev);
	return (err);
    }

    sc->bh1750_cdev->si_drv1 = sc;

    return (0);
}

