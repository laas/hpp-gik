# 
#  Copyright 
# 
TESTS = \
	testhppGik

noinst_PROGRAMS = testhppGik
testhppGik_SOURCES=\
	main.cpp		\
	hppGikTest.cpp		\
	hppGikTest.h




testhppGik_CPPFLAGS=\
	-I$(top_srcdir)/include		\
	$(ABS_GIKTASK_CFLAGS)		\
	$(HRP2_CFLAGS)

testhppGik_LDFLAGS = $(ABS_GIKTASK_LIBS) $(HRP2_LIBS) $(top_builddir)/src/libhppGik.la -lm
