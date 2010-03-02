include $(top_srcdir)/build-aux/init.mk

# -- Distribute and install headers
nobase_include_HEADERS =				\
	hppGikTools.h

nobase_include_HEADERS +=				\
	examples/example.h

nobase_include_HEADERS +=				\
	constraints/hppGikJointStateConstraint.h \
	constraints/hppGikComConstraint.h	\
	constraints/hppGikPlaneConstraint.h 	\
	constraints/hppGikVectorizableConstraint.h 	\
	constraints/hppGikPointingConstraint.h \
	constraints/hppGikGazeConstraint.h \
	constraints/hppGikParallelConstraint.h \
	constraints/hppGikRotationConstraint.h \
	constraints/hppGikConfigurationConstraint.h \
	constraints/hppGikPositionConstraint.h \
	constraints/hppGikTransformationConstraint.h \
	constraints/hppGikPrioritizedStateConstraint.h \
	constraints/hppGikMotionConstraint.h 

nobase_include_HEADERS +=				\
	core/hppGikBounder.h \
	core/hppGikSolver.h \
	core/hppGikSolverBasic.h \
	core/hppGikMotionPlanColumn.h \
	core/hppGikMotionPlanElement.h \
	core/hppGikMotionPlan.h \
	core/hppGikPrioritizedMotion.h \
	core/hppGikMotionPlanRow.h

nobase_include_HEADERS +=				\
	tasks/hppGikGenericTask.h \
	tasks/hppGikWholeBodyTask.h \
	tasks/hppGikRobotTask.h \
	tasks/hppGikStepBackTask.h \
	tasks/hppGikStepTask.h \
	tasks/hppGikHandTask.h \
	tasks/hppGikMetaTask.h \
	tasks/hppGikReachTask.h \
	tasks/hppGikHalfSittingTask.h

nobase_include_HEADERS +=				\
	motionplanners/hppGikLocomotionPlan.h \
	motionplanners/hppGikPreviewController.h

nobase_include_HEADERS +=				\
	motionplanners/elements/hppGikComMotion.h \
	motionplanners/elements/hppGikFootDisplaceElement.h \
	motionplanners/elements/hppGikInterpolatedElement.h \
	motionplanners/elements/hppGikLocomotionElement.h \
	motionplanners/elements/hppGikNoLocomotion.h \
	motionplanners/elements/hppGikReadyElement.h \
	motionplanners/elements/hppGikStepElement.h \
	motionplanners/elements/hppGikWalkElement.h \
	motionplanners/elements/hppGikZMPshiftElement.h

nobase_include_HEADERS +=				\
	robot/hppGikFootprintRelated.h \
	robot/hppGikMaskFactory.h \
	robot/hppRobotMotion.h \
	robot/hppGikStandingRobot.h \
	robot/hppGik2DShape.h

gainfiledir=	$(datadir)/hpp/gik
gainfile_DATA=\
	hpp/gik/motionplanner/preview-controller-5ms.ini	\
	hpp/gik/motionplanner/preview-controller-10ms.ini	\
	hpp/gik/motionplanner/preview-controller-20ms.ini	\
	hpp/gik/motionplanner/preview-controller-50ms.ini
EXTRA_DIST+=	$(gainfile_DATA)

#
#  Install deprecated headers
#

