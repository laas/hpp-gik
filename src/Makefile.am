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
	constraints/hppGikConfigurationConstraint.cpp \
	constraints/hppGikComConstraint.cpp \
	constraints/hppGikTransformationConstraint.cpp \
	constraints/hppGikMotionConstraint.cpp \
	core/hppGikBounder.cpp \
	core/hppGikSolver.cpp \
	core/hppGikSolverBasic.cpp \
	core/hppGikMotionPlanColumn.cpp \
	core/hppGikMotionPlanElement.cpp \
	core/hppGikMotionPlan.cpp \
	core/hppGikMotionPlanRow.cpp \
	motionplanners/hppGikLocomotionPlan.cpp \
	motionplanners/hppGikPreviewController.cpp \
	motionplanners/elements/hppGikComMotion.cpp \
	motionplanners/elements/hppGikFootDisplaceElement.cpp \
	motionplanners/elements/hppGikInterpolatedElement.cpp \
	motionplanners/elements/hppGikNoLocomotion.cpp \
	motionplanners/elements/hppGikStepElement.cpp \
	motionplanners/elements/hppGikWalkElement.cpp \
	motionplanners/elements/hppGikZMPshiftElement.cpp \
	tasks/hppGikGenericTask.cpp \
	tasks/hppGikWholeBodyTask.cpp \
	tasks/hppGikRobotTask.cpp \
	tasks/hppGikStepBackTask.cpp \
	tasks/hppGikStepTask.cpp \
	tasks/hppGikHandTask.cpp \
	tasks/hppGikMetaTask.cpp \
	tasks/hppGikReachTask.cpp \
	tasks/hppGikHalfSittingTask.cpp \
	robot/hppGikFootprintRelated.cpp \
	robot/hppGikMaskFactory.cpp \
	robot/hppRobotMotion.cpp \
	robot/hppGikStandingRobot.cpp \
	hppGikTools.cpp


libhppGik_la_CPPFLAGS=\
	-I$(srcdir)/../include		\
	$(ABS_GIKTASK_CFLAGS)		\
	-DGAINDIR='"${datadir}/gik"'
	
libhppGik_la_LDFLAGS= $(ABS_GIKTASK_LIBS)	\
	-release ${PACKAGE_VERSION}
