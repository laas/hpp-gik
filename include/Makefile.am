include $(top_srcdir)/build-aux/init.mk

# -- Distribute and install headers
nobase_include_HEADERS =				\
	hpp/gik/examples/example.h

nobase_include_HEADERS +=				\
	hpp/gik/constraints/hppGikJointStateConstraint.h \
	hpp/gik/constraints/hppGikComConstraint.h	\
	hpp/gik/constraints/hppGikPlaneConstraint.h 	\
	hpp/gik/constraints/hppGikVectorizableConstraint.h 	\
	hpp/gik/constraints/hppGikPointingConstraint.h \
	hpp/gik/constraints/hppGikGazeConstraint.h \
	hpp/gik/constraints/hppGikParallelConstraint.h \
	hpp/gik/constraints/hppGikRotationConstraint.h \
	hpp/gik/constraints/hppGikConfigurationConstraint.h \
	hpp/gik/constraints/hppGikPositionConstraint.h \
	hpp/gik/constraints/hppGikTransformationConstraint.h \
	hpp/gik/constraints/hppGikPrioritizedStateConstraint.h \
	hpp/gik/constraints/hppGikMotionConstraint.h 

nobase_include_HEADERS +=				\
	hpp/gik/core/hppGikBounder.h \
	hpp/gik/core/hppGikSolver.h \
	hpp/gik/core/hppGikSolverBasic.h \
	hpp/gik/core/hppGikMotionPlanColumn.h \
	hpp/gik/core/hppGikMotionPlanElement.h \
	hpp/gik/core/hppGikMotionPlan.h \
	hpp/gik/core/hppGikPrioritizedMotion.h \
	hpp/gik/core/hppGikMotionPlanRow.h

nobase_include_HEADERS +=				\
	hpp/gik/tasks/hppGikGenericTask.h \
	hpp/gik/tasks/hppGikWholeBodyTask.h \
	hpp/gik/tasks/hppGikRobotTask.h \
	hpp/gik/tasks/hppGikStepBackTask.h \
	hpp/gik/tasks/hppGikStepTask.h \
	hpp/gik/tasks/hppGikHandTask.h \
	hpp/gik/tasks/hppGikMetaTask.h \
	hpp/gik/tasks/hppGikReachTask.h \
	hpp/gik/tasks/hppGikHalfSittingTask.h

nobase_include_HEADERS +=				\
	hpp/gik/motionplanners/hppGikLocomotionPlan.h \
	hpp/gik/motionplanners/hppGikPreviewController.h

nobase_include_HEADERS +=				\
	hpp/gik/motionplanners/elements/hppGikComMotion.h \
	hpp/gik/motionplanners/elements/hppGikFootDisplaceElement.h \
	hpp/gik/motionplanners/elements/hppGikInterpolatedElement.h \
	hpp/gik/motionplanners/elements/hppGikLocomotionElement.h \
	hpp/gik/motionplanners/elements/hppGikNoLocomotion.h \
	hpp/gik/motionplanners/elements/hppGikReadyElement.h \
	hpp/gik/motionplanners/elements/hppGikStepElement.h \
	hpp/gik/motionplanners/elements/hppGikWalkElement.h \
	hpp/gik/motionplanners/elements/hppGikZMPshiftElement.h

nobase_include_HEADERS +=				\
	hpp/gik/robot/hppGikFootprintRelated.h \
	hpp/gik/robot/hppGikMaskFactory.h \
	hpp/gik/robot/hppRobotMotion.h \
	hpp/gik/robot/hppGikStandingRobot.h \
	hpp/gik/robot/hppGik2DShape.h

gainfiledir=	$(datadir)/gik
gainfile_DATA=\
	hpp/gik/motionplanners/hppGikPreviewController5ms.ini	\
	hpp/gik/motionplanners/hppGikPreviewController10ms.ini	\
	hpp/gik/motionplanners/hppGikPreviewController20ms.ini	\
	hpp/gik/motionplanners/hppGikPreviewController50ms.ini
EXTRA_DIST+=	$(gainfile_DATA)

#
#  Install deprecated headers
#

