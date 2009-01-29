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
	-I$(includedir)/hppGik		\
	$(ABS_GIKTASK_CFLAGS)		\
	$(DYNAMICSJRLJAPAN_CFLAGS)	\
	$(HRP2_CFLAGS)

testhppGik_LDFLAGS = $(DYNAMICSJRLJAPAN_LIBS) $(HRP2_LIBS) -lhppGik -lm
