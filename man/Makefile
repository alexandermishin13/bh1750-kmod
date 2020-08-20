# $FreeBSD$

PREFIX= /usr/local

MAN= bh1750.4
MANDIR= ${PREFIX}/man/man

MANFULLDIR= ${DESTDIR}${MANDIR}4

beforeinstall:
.if !exists(${MANFULLDIR})
	@mkdir -v -p ${MANFULLDIR}
.endif

uninstall:
.if exists(${MANFULLDIR}/${MAN}.gz)
	@rm -v ${MANFULLDIR}/${MAN}.gz
	@rmdir -v ${MANFULLDIR}
.endif

.include <bsd.prog.mk>
