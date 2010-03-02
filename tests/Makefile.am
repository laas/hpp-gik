include $(top_srcdir)/build-aux/init.mk
include $(top_srcdir)/build-aux/autotest.mk

# Add available warnings flags.
AM_CXXFLAGS += $(WARNING_CXXFLAGS)

# Package the following files.
EXTRA_DIST += package.m4.in

# Clean atconfig (do not distribute it).
CLEANFILES += atconfig

# Source which are shared by all tests.
COMMON_SOURCES = common.hh

# simple-test
check_PROGRAMS = tests
tests_SOURCES = tests.cc $(COMMON_SOURCES)
tests_LDADD = $(top_builddir)/src/lib@PACKAGE_TARNAME@.la
tests_CPPFLAGS = $(AM_CPPFLAGS) $(ABSTRACTGIKTASK_CFLAGS) $(HRP2DYNAMICS_CFLAGS) 

.PHONY: generate-reference

# Re-generate reference outputs.
generate-reference: $(check_PROGRAMS)
	@echo "Generating references...";	\
	for i in $(check_PROGRAMS); do		\
	 IN="$(srcdir)/$$i.stdin";		\
	 OUT="$(srcdir)/$$i.stdout";		\
	 if test -f "$$IN"; then 		\
	  "./$$i" < $$IN > $$OUT || true;	\
	 else					\
	  "./$$i" > $$OUT || true;		\
	 fi					\
	done


# ------------ #
# Test suite.  #
# ------------ #

TESTSUITE_AT +=					\
	simple-test.at

# Distribute reference files.
EXTRA_DIST +=					\
	simple-test.stdout
