# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

SUBDIRS = \
	src \
	include \
	unitTesting

hppGikdir = $(libdir)/pkgconfig
hppGik_DATA = \
	hppGik.pc

.PHONY: doc install-doc
doc install-doc:
	cd doc && ${MAKE} $@
