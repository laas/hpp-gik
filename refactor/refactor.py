#!/usr/bin/python

oldHeaders = [\
'hppGikTools.h', \
'examples/example.h', \
'constraints/hppGikJointStateConstraint.h', \
'constraints/hppGikComConstraint.h', \
'constraints/hppGikPlaneConstraint.h', \
'constraints/hppGikVectorizableConstraint.h', \
'constraints/hppGikPointingConstraint.h', \
'constraints/hppGikGazeConstraint.h', \
'constraints/hppGikParallelConstraint.h', \
'constraints/hppGikRotationConstraint.h', \
'constraints/hppGikConfigurationConstraint.h', \
'constraints/hppGikPositionConstraint.h', \
'constraints/hppGikTransformationConstraint.h', \
'constraints/hppGikPrioritizedStateConstraint.h', \
'constraints/hppGikMotionConstraint.h', \
'core/hppGikBounder.h', \
'core/hppGikSolver.h', \
'core/hppGikSolverBasic.h', \
'core/hppGikMotionPlanColumn.h', \
'core/hppGikMotionPlanElement.h', \
'core/hppGikMotionPlan.h', \
'core/hppGikPrioritizedMotion.h', \
'core/hppGikMotionPlanRow.h', \
'motionplanners/hppGikLocomotionPlan.h', \
'motionplanners/hppGikPreviewController.h', \
'motionplanners/elements/hppGikComMotion.h', \
'motionplanners/elements/hppGikFootDisplaceElement.h', \
'motionplanners/elements/hppGikInterpolatedElement.h', \
'motionplanners/elements/hppGikLocomotionElement.h', \
'motionplanners/elements/hppGikNoLocomotion.h', \
'motionplanners/elements/hppGikReadyElement.h', \
'motionplanners/elements/hppGikStepElement.h', \
'motionplanners/elements/hppGikWalkElement.h', \
'motionplanners/elements/hppGikZMPshiftElement.h', \
'robot/hppGikFootprintRelated.h', \
'robot/hppGikMaskFactory.h', \
'robot/hppRobotMotion.h', \
'robot/hppGikStandingRobot.h', \
'robot/hppGik2DShape.h', \
'tasks/hppGikGenericTask.h', \
'tasks/hppGikWholeBodyTask.h', \
'tasks/hppGikRobotTask.h', \
'tasks/hppGikStepBackTask.h', \
'tasks/hppGikStepTask.h', \
'tasks/hppGikHandTask.h', \
'tasks/hppGikMetaTask.h', \
'tasks/hppGikReachTask.h', \
'tasks/hppGikHalfSittingTask.h'\
]

newHeaders = [\
'hpp/gik/tools.hh', \
'hpp/gik/example/example.hh', \
'hpp/gik/constraint/joint-state-constraint.hh', \
'hpp/gik/constraint/com-constraint.hh', \
'hpp/gik/constraint/plane-constraint.hh', \
'hpp/gik/constraint/vectorizable-constraint.hh', \
'hpp/gik/constraint/pointing-constraint.hh', \
'hpp/gik/constraint/gaze-constraint.hh', \
'hpp/gik/constraint/parallel-constraint.hh', \
'hpp/gik/constraint/rotation-constraint.hh', \
'hpp/gik/constraint/configuration-constraint.hh', \
'hpp/gik/constraint/position-constraint.hh', \
'hpp/gik/constraint/transformation-constraint.hh', \
'hpp/gik/constraint/prioritized-state-constraint.hh', \
'hpp/gik/constraint/motion-constraint.hh', \
'hpp/gik/core/bounder.hh', \
'hpp/gik/core/solver.hh', \
'hpp/gik/core/solver-basic.hh', \
'hpp/gik/core/motion-plan-column.hh', \
'hpp/gik/core/motion-plan-element.hh', \
'hpp/gik/core/motion-plan.hh', \
'hpp/gik/core/prioritized-motion.hh', \
'hpp/gik/core/motion-plan-row.hh', \
'hpp/gik/motionplanner/locomotion-plan.hh', \
'hpp/gik/motionplanner/preview-controller.hh', \
'hpp/gik/motionplanner/element/com-motion.hh', \
'hpp/gik/motionplanner/element/foot-displace-element.hh', \
'hpp/gik/motionplanner/element/interpolated-element.hh', \
'hpp/gik/motionplanner/element/locomotion-element.hh', \
'hpp/gik/motionplanner/element/no-locomotion.hh', \
'hpp/gik/motionplanner/element/ready-element.hh', \
'hpp/gik/motionplanner/element/step-element.hh', \
'hpp/gik/motionplanner/element/walk-element.hh', \
'hpp/gik/motionplanner/element/zmp-shift-element.hh', \
'hpp/gik/robot/foot-print-related.hh', \
'hpp/gik/robot/mask-factory.hh', \
'hpp/gik/robot/robot-motion.hh', \
'hpp/gik/robot/standing-robot.hh', \
'hpp/gik/robot/shape-2d.hh', \
'hpp/gik/task/generic-task.hh', \
'hpp/gik/task/whole-body-task.hh', \
'hpp/gik/task/robot-task.hh', \
'hpp/gik/task/step-back-task.hh', \
'hpp/gik/task/step-task.hh', \
'hpp/gik/task/hand-task.hh', \
'hpp/gik/task/meta-task.hh', \
'hpp/gik/task/reach-task.hh', \
'hpp/gik/task/half-sitting-task.hh'\
]

oldSources = [\
'constraints/hppGikJointStateConstraint.cpp', \
'constraints/hppGikPlaneConstraint.cpp', \
'constraints/hppGikPointingConstraint.cpp', \
'constraints/hppGikGazeConstraint.cpp', \
'constraints/hppGikParallelConstraint.cpp', \
'constraints/hppGikRotationConstraint.cpp', \
'constraints/hppGikPositionConstraint.cpp', \
'constraints/hppGikConfigurationConstraint.cpp', \
'constraints/hppGikComConstraint.cpp', \
'constraints/hppGikTransformationConstraint.cpp', \
'constraints/hppGikMotionConstraint.cpp', \
'core/hppGikBounder.cpp', \
'core/hppGikSolver.cpp', \
'core/hppGikSolverBasic.cpp', \
'core/hppGikMotionPlanColumn.cpp', \
'core/hppGikMotionPlanElement.cpp', \
'core/hppGikMotionPlan.cpp', \
'core/hppGikMotionPlanRow.cpp', \
'motionplanners/hppGikLocomotionPlan.cpp', \
'motionplanners/hppGikPreviewController.cpp', \
'motionplanners/elements/hppGikComMotion.cpp', \
'motionplanners/elements/hppGikFootDisplaceElement.cpp', \
'motionplanners/elements/hppGikInterpolatedElement.cpp', \
'motionplanners/elements/hppGikNoLocomotion.cpp', \
'motionplanners/elements/hppGikStepElement.cpp', \
'motionplanners/elements/hppGikWalkElement.cpp', \
'motionplanners/elements/hppGikZMPshiftElement.cpp', \
'tasks/hppGikGenericTask.cpp', \
'tasks/hppGikWholeBodyTask.cpp', \
'tasks/hppGikRobotTask.cpp', \
'tasks/hppGikStepBackTask.cpp', \
'tasks/hppGikStepTask.cpp', \
'tasks/hppGikHandTask.cpp', \
'tasks/hppGikMetaTask.cpp', \
'tasks/hppGikReachTask.cpp', \
'tasks/hppGikHalfSittingTask.cpp', \
'robot/hppGikFootprintRelated.cpp', \
'robot/hppGikMaskFactory.cpp', \
'robot/hppRobotMotion.cpp', \
'robot/hppGikStandingRobot.cpp', \
'hppGikTools.cpp' \
]

newSources = [\
'constraint/joint-state-constraint.cc', \
'constraint/plane-constraint.cc', \
'constraint/pointing-constraint.cc', \
'constraint/gaze-constraint.cc', \
'constraint/parallel-constraint.cc', \
'constraint/rotation-constraint.cc', \
'constraint/position-constraint.cc', \
'constraint/configuration-constraint.cc', \
'constraint/com-constraint.cc', \
'constraint/transformation-constraint.cc', \
'constraint/motion-constraint.cc', \
'core/bounder.cc', \
'core/solver.cc', \
'core/solver-basic.cc', \
'core/motion-plan-column.cc', \
'core/motion-plan-element.cc', \
'core/motion-plan.cc', \
'core/motion-plan-row.cc', \
'motionplanner/locomotion-plan.cc', \
'motionplanner/preview-controller.cc', \
'motionplanner/element/com-motion.cc', \
'motionplanner/element/foot-displace-element.cc', \
'motionplanner/element/interpolated-element.cc', \
'motionplanner/element/no-locomotion.cc', \
'motionplanner/element/step-element.cc', \
'motionplanner/element/walk-element.cc', \
'motionplanner/element/zmp-shift-element.cc', \
'task/generic-task.cc', \
'task/whole-body-task.cc', \
'task/robot-task.cc', \
'task/step-back-task.cc', \
'task/step-task.cc', \
'task/hand-task.cc', \
'task/meta-task.cc', \
'task/reach-task.cc', \
'task/half-sitting-task.cc', \
'robot/foot-print-related.cc', \
'robot/mask-factory.cc', \
'robot/robot-motion.cc', \
'robot/standing-robot.cc', \
'tools.cc' \
]

def copyAndGitAdd():
    print "#!/bin/sh\n"

    print "mkdir -p include/hpp/gik/example"
    print "mkdir -p include/hpp/gik/constraint"
    print "mkdir -p include/hpp/gik/core"
    print "mkdir -p include/hpp/gik/motionplanner/element"
    print "mkdir -p include/hpp/gik/robot"
    print "mkdir -p include/hpp/gik/task"

    print "mkdir -p src/example"
    print "mkdir -p src/constraint"
    print "mkdir -p src/core"
    print "mkdir -p src/motionplanner/element"
    print "mkdir -p src/robot"
    print "mkdir -p src/task"

    for i in range(len(oldHeaders)):
        print "cp", "include/" + oldHeaders[i], "include/" + newHeaders[i]
        print "git add include/" + newHeaders[i]

    for i in range(len(oldSources)):
        print "git mv", "src/" + oldSources[i], "src/" + newSources[i]

def printWarning():

    print "#!/bin/sh\n"

    for i in range(len(oldHeaders)):
        filename = "include/" + oldHeaders[i] + ".tmp"
        try:
            f = open(filename, 'w')
        except IOError, ex:
            print "cannot open ", filename

        f.write("#warning This header is deprecated,\n")
        f.write("#warning use "+newHeaders[i]+" instead.\n")
        f.write("")

        print "cat", filename, "include/"+oldHeaders[i], ">", "include/"+oldHeaders[i]+".tmp2"
        print "mv", "include/"+oldHeaders[i]+".tmp2", "include/"+oldHeaders[i]
        print "rm -f", filename

def generateSedCommands ():

    print "#!/bin/sh\n"

    for s in range(len(newSources)):
        for h in range(len(oldHeaders)):
            print "cat src/" + newSources[s] + " | sed \'s|#include \"" + oldHeaders[h] + "\"|#include \"" + newHeaders[h] + "\"|\' > " + "src/" + newSources[s] + ".tmp"
            print "mv", "src/" + newSources[s] + ".tmp", "src/" + newSources[s]
            print "rm -f", "src/" + newSources[s] + ".tmp"

    for s in range(len(newHeaders)):
        for h in range(len(oldHeaders)):
            print "cat include/" + newHeaders[s] + " | sed \'s|#include \"" + oldHeaders[h] + "\"|#include \"" + newHeaders[h] + "\"|\' > " + "include/" + newHeaders[s] + ".tmp"
            print "mv", "include/" + newHeaders[s] + ".tmp", "include/" + newHeaders[s]
            print "rm -f", "include/" + newHeaders[s] + ".tmp"

generateSedCommands ()
