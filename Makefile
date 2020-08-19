# $FreeBSD$

KMOD=bh1750
SRCS=bh1750.c
SUBDIR=fdt-overlay man

SRCS+=	\
	bus_if.h \
	device_if.h \
	iicbus_if.h \
	ofw_bus_if.h \
	opt_platform.h \
	fdt_pinctrl_if.h \

#CFLAGS+=  -DDEBUG

.include <bsd.kmod.mk>
