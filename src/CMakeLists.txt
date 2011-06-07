SET(SOURCES
tools.cc
constraint/joint-state-constraint.cc
constraint/plane-constraint.cc
constraint/pointing-constraint.cc
constraint/gaze-constraint.cc
constraint/parallel-constraint.cc
constraint/rotation-constraint.cc
constraint/position-constraint.cc
constraint/configuration-constraint.cc
constraint/com-constraint.cc
constraint/transformation-constraint.cc
constraint/motion-constraint.cc
core/solver.cc
core/lne-solver.cc
core/motion-plan-column.cc
core/motion-plan-element.cc
core/motion-plan.cc
core/motion-plan-row.cc
motionplanner/locomotion-plan.cc
motionplanner/preview-controller.cc
motionplanner/element/com-motion.cc
motionplanner/element/foot-displace-element.cc
motionplanner/element/interpolated-element.cc
motionplanner/element/no-locomotion.cc
motionplanner/element/step-element.cc
motionplanner/element/walk-element.cc
motionplanner/element/zmp-shift-element.cc
task/generic-task.cc
task/whole-body-task.cc
task/robot-task.cc
task/step-back-task.cc
task/step-task.cc
task/hand-task.cc
task/meta-task.cc
task/reach-task.cc
task/half-sitting-task.cc
robot/foot-print-related.cc
robot/mask-factory.cc
robot/robot-motion.cc
robot/standing-robot.cc
)

ADD_DEFINITIONS(-DGAINDIR="${CMAKE_INSTALL_PREFIX}/share/hpp/gik")

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib)

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} abstract-gik-task)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hrp2-dynamics)
