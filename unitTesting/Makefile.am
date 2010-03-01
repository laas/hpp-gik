# 
#  Copyright 
# 
TESTS = tests

noinst_PROGRAMS = tests

tests_SOURCES= tests.cc

tests_CPPFLAGS=\
	-I$(top_srcdir)/include		\
	$(ABS_GIKTASK_CFLAGS)		\
	$(HRP2_CFLAGS)

tests_LDFLAGS = $(ABS_GIKTASK_LIBS) $(HRP2_LIBS) $(top_builddir)/src/libhppGik.la -lm
