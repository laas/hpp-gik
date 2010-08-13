#
#  Copyright 2010 CNRS-LAAS
#  Author: Florent Lamiraux
#

SET(${PROJECT_NAME}_GIK_HEADERS
	hpp/gik/doc.hh
	hpp/gik/tools.hh
)
SET(${PROJECT_NAME}_GIK_EXAMPLE_HEADERS
	hpp/gik/example/example.hh
)
SET(${PROJECT_NAME}_GIK_CONSTRAINT_HEADERS
	hpp/gik/constraint/joint-state-constraint.hh
	hpp/gik/constraint/com-constraint.hh
	hpp/gik/constraint/plane-constraint.hh
	hpp/gik/constraint/vectorizable-constraint.hh
	hpp/gik/constraint/pointing-constraint.hh
	hpp/gik/constraint/gaze-constraint.hh
	hpp/gik/constraint/parallel-constraint.hh
	hpp/gik/constraint/rotation-constraint.hh
	hpp/gik/constraint/configuration-constraint.hh
	hpp/gik/constraint/position-constraint.hh
	hpp/gik/constraint/transformation-constraint.hh
	hpp/gik/constraint/prioritized-state-constraint.hh
	hpp/gik/constraint/motion-constraint.hh
)
SET(${PROJECT_NAME}_GIK_CORE_HEADERS
	hpp/gik/core/solver.hh
	hpp/gik/core/lne-solver.hh
	hpp/gik/core/motion-plan-column.hh
	hpp/gik/core/motion-plan-element.hh
	hpp/gik/core/motion-plan.hh
	hpp/gik/core/prioritized-motion.hh
	hpp/gik/core/motion-plan-row.hh
)
SET(${PROJECT_NAME}_GIK_MOTIONPLANNER_HEADERS
	hpp/gik/motionplanner/locomotion-plan.hh
	hpp/gik/motionplanner/preview-controller.hh
)
SET(${PROJECT_NAME}_GIK_MOTIONPLANNER_ELEMENT_HEADERS
	hpp/gik/motionplanner/element/com-motion.hh
	hpp/gik/motionplanner/element/foot-displace-element.hh
	hpp/gik/motionplanner/element/interpolated-element.hh
	hpp/gik/motionplanner/element/locomotion-element.hh
	hpp/gik/motionplanner/element/no-locomotion.hh
	hpp/gik/motionplanner/element/ready-element.hh
	hpp/gik/motionplanner/element/step-element.hh
	hpp/gik/motionplanner/element/walk-element.hh
	hpp/gik/motionplanner/element/zmp-shift-element.hh
)
SET(${PROJECT_NAME}_GIK_ROBOT_HEADERS
	hpp/gik/robot/foot-print-related.hh
	hpp/gik/robot/mask-factory.hh
	hpp/gik/robot/robot-motion.hh
	hpp/gik/robot/standing-robot.hh
	hpp/gik/robot/shape-2d.hh
)
SET(${PROJECT_NAME}_GIK_TASK_HEADERS
	hpp/gik/task/generic-task.hh
	hpp/gik/task/whole-body-task.hh
	hpp/gik/task/robot-task.hh
	hpp/gik/task/step-back-task.hh
	hpp/gik/task/step-task.hh
	hpp/gik/task/hand-task.hh
	hpp/gik/task/meta-task.hh
	hpp/gik/task/reach-task.hh
	hpp/gik/task/half-sitting-task.hh
)

#----------------------------------------------------
# Install procedure for the header files
#----------------------------------------------------

INSTALL(FILES ${${PROJECT_NAME}_GIK_HEADERS}
  DESTINATION include/hpp/gik
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
INSTALL(FILES ${${PROJECT_NAME}_GIK_EXAMPLE_HEADERS}
  DESTINATION include/hpp/gik/example
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
INSTALL(FILES ${${PROJECT_NAME}_GIK_CONSTRAINT_HEADERS}
  DESTINATION include/hpp/gik/constraint
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
INSTALL(FILES ${${PROJECT_NAME}_GIK_CORE_HEADERS}
  DESTINATION include/hpp/gik/core
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
INSTALL(FILES ${${PROJECT_NAME}_GIK_MOTIONPLANNER_HEADERS}
  DESTINATION include/hpp/gik/motionplanner
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
INSTALL(FILES ${${PROJECT_NAME}_GIK_MOTIONPLANNER_ELEMENT_HEADERS}
  DESTINATION include/hpp/gik/motionplanner/element
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
INSTALL(FILES ${${PROJECT_NAME}_GIK_ROBOT_HEADERS}
  DESTINATION include/hpp/gik/robot
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
INSTALL(FILES ${${PROJECT_NAME}_GIK_TASK_HEADERS}
  DESTINATION include/hpp/gik/task
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)

#----------------------------------------------------
# Install procedure for the data files
#----------------------------------------------------

SET(${PROJECT_NAME}_DATA_FILES
  hpp/gik/motionplanner/preview-controller-5ms.ini
  hpp/gik/motionplanner/preview-controller-10ms.ini
  hpp/gik/motionplanner/preview-controller-20ms.ini
  hpp/gik/motionplanner/preview-controller-50ms.ini
)
INSTALL(FILES ${${PROJECT_NAME}_DATA_FILES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/hpp/gik
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
