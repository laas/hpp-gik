# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

hppGikdir = $(datadir)/doc/hppGik

EXTRA_DIST = \
	header.html \
	footer.html \
	hpp.css \
	additionalDoc/hppGik.h \
	pictures/HRP2.jpg


install-doc: doc
	mkdir -p $(hppGikdir)
	$(INSTALL_DATA) html/* $(hppGikdir)
	mkdir -p $(hppGikdir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(hppGikdir)/pictures

doc: hpp.dox
	doxygen $<

clean-local:
	-rm -r html
