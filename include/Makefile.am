# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

gainfiledir=	$(datadir)/gik
gainfile_DATA=\
	motionplanners/hppGikPreviewController5ms.ini	\
	motionplanners/hppGikPreviewController10ms.ini	\
	motionplanners/hppGikPreviewController20ms.ini	\
	motionplanners/hppGikPreviewController50ms.ini
EXTRA_DIST=	$(gainfile_DATA)

hppGikdir = $(includedir)/hppGik
hppGik_HEADERS = \
	hppGikTools.h

hppGikConstdir = $(includedir)/hppGik/constraints
hppGikConst_HEADERS = \
	constraints/hppGikJointStateConstraint.h \
	constraints/hppGikComConstraint.h	\
	constraints/hppGikPlaneConstraint.h 	\
	constraints/hppGikVectorizableConstraint.h 	\
	constraints/hppGikPointingConstraint.h \
	constraints/hppGikGazeConstraint.h \
	constraints/hppGikParallelConstraint.h \
	constraints/hppGikRotationConstraint.h \
	constraints/hppGikLPConstraint.h \
	constraints/hppGikConfigurationConstraint.h \
	constraints/hppGikPositionConstraint.h \
	constraints/hppGikTransformationConstraint.h \
	constraints/hppGikPrioritizedStateConstraint.h \
	constraints/hppGikMotionConstraint.h 

hppGikCoredir = $(includedir)/hppGik/core
hppGikCore_HEADERS = \
	core/hppGikSolver.h \
	core/hppGikMotionPlanColumn.h \
	core/hppGikMotionPlanElement.h \
	core/hppGikMotionPlan.h \
	core/hppGikPrioritizedMotion.h \
	core/hppGikMotionPlanRow.h

hppGikTasksdir = $(includedir)/hppGik/tasks
hppGikTasks_HEADERS = \
	tasks/hppGikGenericTask.h \
	tasks/hppGikWholeBodyTask.h \
	tasks/hppGikRobotTask.h \
	tasks/hppGikStepBackTask.h \
	tasks/hppGikStepTask.h \
	tasks/hppGikHandTask.h \
	tasks/hppGikMetaTask.h \
	tasks/hppGikReachTask.h \
	tasks/hppGikHalfSittingTask.h

hppGikPlannersdir = $(includedir)/hppGik/motionplanners
hppGikPlanners_HEADERS = \
	motionplanners/hppGikLocomotionPlan.h \
	motionplanners/hppGikStepPlanner.h \
	motionplanners/hppGikPreviewController.h
	
hppGikElementsdir = $(includedir)/hppGik/motionplanners/elements
hppGikElements_HEADERS = \
	motionplanners/elements/hppGikComMotion.h \
	motionplanners/elements/hppGikFootDisplaceElement.h \
	motionplanners/elements/hppGikInterpolatedElement.h \
	motionplanners/elements/hppGikLocomotionElement.h \
	motionplanners/elements/hppGikNoLocomotion.h \
	motionplanners/elements/hppGikReadyElement.h \
	motionplanners/elements/hppGikStepElement.h \
	motionplanners/elements/hppGikZMPshiftElement.h

hppGikRobotdir = $(includedir)/hppGik/robot
hppGikRobot_HEADERS = \
	robot/hppGikFootprintRelated.h \
	robot/hppGikMaskFactory.h \
	robot/hppRobotMotion.h \
	robot/hppGikStandingRobot.h \
	robot/hppGik2DShape.h

