dnl autoconf script for hppGik libs
dnl
dnl   Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
dnl
dnl   Author: Florent Lamiraux LAAS-CNRS
dnl

AC_INIT([hppGik],[1.1.5],[openrobots@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE([foreign])
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX

PKG_CHECK_MODULES(ABS_GIKTASK, abstractGikTask)
ABS_GIKTASK_PREFIX=`$PKG_CONFIG abstractGikTask --variable=prefix`
AC_SUBST(ABS_GIKTASK_PREFIX)
AC_SUBST(ABS_GIKTASK_CFLAGS)

PKG_CHECK_MODULES(DYNAMICSJRLJAPAN, dynamicsJRLJapan >= 1.3.1)
AC_SUBST(DYNAMICSJRLJAPAN_CFLAGS)
AC_SUBST(DYNAMICSJRLJAPAN_LIBS)

AC_OUTPUT(Makefile
	src/Makefile
	include/Makefile
	doc/Makefile
	doc/hpp.dox
	hppGik.pc
	unitTesting/Makefile
)
