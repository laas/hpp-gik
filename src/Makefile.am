# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

lib_LTLIBRARIES = libhppGik.la
libhppGik_la_SOURCES = \
	constraints/hppGikJointStateConstraint.cpp \
	constraints/hppGikPlaneConstraint.cpp \
	constraints/hppGikPointingConstraint.cpp \
	constraints/hppGikGazeConstraint.cpp \
	constraints/hppGikParallelConstraint.cpp \
	constraints/hppGikRotationConstraint.cpp \
	constraints/hppGikPositionConstraint.cpp \
	constraints/hppGikComConstraint.cpp \
	constraints/hppGikTransformationConstraint.cpp \
	constraints/hppGikMotionConstraint.cpp \
	core/hppGikSolver.cpp \
	core/hppGikMotionPlanColumn.cpp \
	core/hppGikMotionPlanElement.cpp \
	core/hppGikMotionPlan.cpp \
	core/hppGikMotionPlanRow.cpp \
	motionplanners/hppGikLocomotionPlan.cpp \
	motionplanners/hppGikPreviewController.cpp \
	motionplanners/hppGikSingleMotionsPlan.cpp \
	tasks/hppGikGenericTask.cpp \
	tasks/hppGikGenericTaskElements.cpp \
	tasks/hppGikWholeBodyTask.cpp \
	tasks/hppGikRobotTask.cpp \
	tasks/hppGikStepBackTask.cpp \
	tasks/hppGikStepTask.cpp \
	tasks/hppGikHandTask.cpp \
	tasks/hppGikMetaTask.cpp \
	tasks/hppGikReachTask.cpp \
	tasks/hppGikConfigurationTask.cpp \
	tasks/hppGikHalfSittingTask.cpp \
	robot/hppGikFootprintRelated.cpp \
	robot/hppGikMaskFactory.cpp \
	robot/hppRobotMotion.cpp \
	robot/hppGikStandingRobot.cpp \
	hppGikTools.cpp


libhppGik_la_CPPFLAGS=\
	-I$(srcdir)/../include		\
	$(ABS_GIKTASK_CFLAGS)		\
	$(DYNAMICSJRLJAPAN_CFLAGS)	\
	-DGAINDIR='"${datadir}/gik"'
libhppGik_la_LDFLAGS= -release ${PACKAGE_VERSION}

libhppGik_la_LIBADD = $(DYNAMICSJRLJAPAN_LIBS)
