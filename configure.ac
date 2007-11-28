dnl autoconf script for hppGik libs
dnl
dnl   Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
dnl
dnl   Author: Florent Lamiraux LAAS-CNRS
dnl

AC_INIT([hppGik],[1.5.99],[openrobots@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE([foreign no-define])
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX

define(abstractgiktask_reqd, [abstractGikTask >= 1.2])
AC_SUBST(ABSTRACTGIKTASK_REQD, "abstractgiktask_reqd")
PKG_CHECK_MODULES(ABS_GIKTASK, abstractgiktask_reqd)
ABS_GIKTASK_PREFIX=`$PKG_CONFIG abstractGikTask --variable=prefix`
AC_SUBST(ABS_GIKTASK_PREFIX)

define(dynamicsjrljapan_reqd, [dynamicsJRLJapan >= 1.6])
AC_SUBST(DYNAMICSJRLJAPAN_REQD, "dynamicsjrljapan_reqd")
PKG_CHECK_MODULES(DYNAMICSJRLJAPAN, dynamicsjrljapan_reqd)

AC_OUTPUT(Makefile
	src/Makefile
	include/Makefile
	doc/Makefile
	doc/hpp.dox
	hppGik.pc
	unitTesting/Makefile
)
