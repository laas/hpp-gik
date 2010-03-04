include $(top_srcdir)/build-aux/init.mk

# -- Distribute and install headers

#
#  Install deprecated headers
#
hppGikdir = $(includedir)/hppGik
hppGik_HEADERS = \
	hppGikTools.h

hppGikExampledir = $(includedir)/hppGik/examples
hppGikExample_HEADERS = \
	examples/example.h


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
	constraints/hppGikConfigurationConstraint.h \
	constraints/hppGikPositionConstraint.h \
	constraints/hppGikTransformationConstraint.h \
	constraints/hppGikPrioritizedStateConstraint.h \
	constraints/hppGikMotionConstraint.h 

hppGikCoredir = $(includedir)/hppGik/core
hppGikCore_HEADERS = \
	core/hppGikBounder.h \
	core/hppGikSolver.h \
	core/hppGikSolverBasic.h \
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
	motionplanners/elements/hppGikWalkElement.h \
	motionplanners/elements/hppGikZMPshiftElement.h

hppGikRobotdir = $(includedir)/hppGik/robot
hppGikRobot_HEADERS = \
	robot/hppGikFootprintRelated.h \
	robot/hppGikMaskFactory.h \
	robot/hppRobotMotion.h \
	robot/hppGikStandingRobot.h \
	robot/hppGik2DShape.h

#
#  Install new headers
#

nobase_include_HEADERS = \
	hpp/gik/doc.hh \
	hpp/gik/tools.hh \
	hpp/gik/example/example.hh \
	hpp/gik/constraint/joint-state-constraint.hh \
	hpp/gik/constraint/com-constraint.hh \
	hpp/gik/constraint/plane-constraint.hh \
	hpp/gik/constraint/vectorizable-constraint.hh \
	hpp/gik/constraint/pointing-constraint.hh \
	hpp/gik/constraint/gaze-constraint.hh \
	hpp/gik/constraint/parallel-constraint.hh \
	hpp/gik/constraint/rotation-constraint.hh \
	hpp/gik/constraint/configuration-constraint.hh \
	hpp/gik/constraint/position-constraint.hh \
	hpp/gik/constraint/transformation-constraint.hh \
	hpp/gik/constraint/prioritized-state-constraint.hh \
	hpp/gik/constraint/motion-constraint.hh \
	hpp/gik/core/bounder.hh \
	hpp/gik/core/solver.hh \
	hpp/gik/core/solver-basic.hh \
	hpp/gik/core/motion-plan-column.hh \
	hpp/gik/core/motion-plan-element.hh \
	hpp/gik/core/motion-plan.hh \
	hpp/gik/core/prioritized-motion.hh \
	hpp/gik/core/motion-plan-row.hh \
	hpp/gik/motionplanner/locomotion-plan.hh \
	hpp/gik/motionplanner/preview-controller.hh \
	hpp/gik/motionplanner/element/com-motion.hh \
	hpp/gik/motionplanner/element/foot-displace-element.hh \
	hpp/gik/motionplanner/element/interpolated-element.hh \
	hpp/gik/motionplanner/element/locomotion-element.hh \
	hpp/gik/motionplanner/element/no-locomotion.hh \
	hpp/gik/motionplanner/element/ready-element.hh \
	hpp/gik/motionplanner/element/step-element.hh \
	hpp/gik/motionplanner/element/walk-element.hh \
	hpp/gik/motionplanner/element/zmp-shift-element.hh \
	hpp/gik/robot/foot-print-related.hh \
	hpp/gik/robot/mask-factory.hh \
	hpp/gik/robot/robot-motion.hh \
	hpp/gik/robot/standing-robot.hh \
	hpp/gik/robot/shape-2d.hh \
	hpp/gik/task/generic-task.hh \
	hpp/gik/task/whole-body-task.hh \
	hpp/gik/task/robot-task.hh \
	hpp/gik/task/step-back-task.hh \
	hpp/gik/task/step-task.hh \
	hpp/gik/task/hand-task.hh \
	hpp/gik/task/meta-task.hh \
	hpp/gik/task/reach-task.hh \
	hpp/gik/task/half-sitting-task.hh

gainfiledir=	$(datadir)/hpp/gik
gainfile_DATA=\
	hpp/gik/motionplanner/preview-controller-5ms.ini	\
	hpp/gik/motionplanner/preview-controller-10ms.ini	\
	hpp/gik/motionplanner/preview-controller-20ms.ini	\
	hpp/gik/motionplanner/preview-controller-50ms.ini
EXTRA_DIST+=	$(gainfile_DATA)
