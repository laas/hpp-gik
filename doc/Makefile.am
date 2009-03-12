# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

hppGikdir = $(datadir)/doc/hppGik/html

EXTRA_DIST = \
	header.html \
	footer.html \
	hpp.css \
	additionalDoc/main-page.h \
	pictures/HRP2.jpg


install: html-doc
	mkdir -p $(hppGikdir)
	$(INSTALL_DATA) html/* $(hppGikdir)
	mkdir -p $(hppGikdir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(hppGikdir)/pictures
	$(INSTALL_DATA) hppGik.doxytag $(hppGikdir)

html-doc: hpp.dox
	doxygen $<

clean-local:
	-rm -r html
