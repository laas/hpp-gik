# 
#  Copyright 
# 
TESTS = \
	testhppGik

noinst_PROGRAMS = testhppGik
testhppGik_SOURCES=\
	main.cpp		\
	hppGikBasicExample.cpp	\
	hppGikBasicExample.h	\
	hppGikTest.cpp		\
	hppGikTest.h		\
	hppGikViewer.cpp 	\
	hppGikViewer.h


testhppGik_CPPFLAGS=\
	-I$(srcdir)/../include		\
	$(ABS_GIKTASK_CFLAGS)		\
	$(DYNAMICSJRLJAPAN_CFLAGS)

testhppGik_LDFLAGS = $(DYNAMICSJRLJAPAN_LIBS) ../src/libhppGik.la $(LIBX11) -lm
