KMOD=bh1750
SRCS=bh1750_kmod.c
SUBDIR=fdt-overlay

SRCS+=	\
	bus_if.h \
	device_if.h \
	iicbus_if.h \
	ofw_bus_if.h \
	opt_platform.h \
	fdt_pinctrl_if.h \

CFLAGS+=  -DDEBUG

.include <bsd.kmod.mk>
