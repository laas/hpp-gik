#
#  Copyright 2010 CNRS-LAAS
#  Author: Florent Lamiraux
#

SET(LIBRARY_NAME ${PROJECT_NAME})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

# provide path to library libabstractGikTask.so
LINK_DIRECTORIES(${ABSTRACTGIKTASK_LIBRARY_DIRS})

ADD_DEFINITIONS(-DHPP_GIK_GAINDIR="${CMAKE_INSTALL_PREFIX}/share/hpp/gik")

IF(DISABLE_TEST STREQUAL NO)
  # provide path to library libhrp2Dynamics.so
  LINK_DIRECTORIES(${HRP2DYNAMICS_LIBRARY_DIRS})
ENDIF(DISABLE_TEST STREQUAL NO)

SET(${PACKAGE_NAME}_SOURCES
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
  core/solver-basic.cc
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
ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${PACKAGE_NAME}_SOURCES}
)

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH lib)

SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
  PROPERTIES
  COMPILE_FLAGS ${${PROJECT_NAME}_src_CXXFLAGS}
  LINK_FLAGS ${${PROJECT_NAME}_src_LDFLAGS}
)

# Add abstractGikTask compilation flags and link to library libabstractGikTask.so
ADD_DEFINITIONS(${ABSTRACTGIKTASK_CFLAGS})
IF(UNIX)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${ABSTRACTGIKTASK_LIBRARIES})
ENDIF(UNIX)

IF(WIN32)
foreach(dlink ${ABSTRACTGIKTASK_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

IF(DISABLE_TEST STREQUAL NO)
  # Add hrp2Dynamics compilation flags and link to library libhrp2Dynamics.so
  ADD_DEFINITIONS(${HRP2DYNAMICS_CFLAGS})
  IF(UNIX)
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${HRP2DYNAMICS_LIBRARIES})
  ENDIF(UNIX)

  IF(WIN32)
    foreach(dlink ${HRP2DYNAMICS_LDFLAGS})
      SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
    endforeach(dlink)
  ENDIF(WIN32)
ENDIF(DISABLE_TEST STREQUAL NO)

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
)

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION lib)
