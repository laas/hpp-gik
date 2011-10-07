# Copyright 2010, 2011, Florent Lamiraux, Thomas Moulard (CNRS/LAAS)
#
# This file is part of hpp-gik.
# sot-motion-planner is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-motion-planner is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-motion-planner. If not, see <http://www.gnu.org/licenses/>.

SET(GIK_HEADERS
	hpp/gik/doc.hh
	hpp/gik/tools.hh
)
SET(GIK_EXAMPLE_HEADERS
	hpp/gik/example/example.hh
)
SET(GIK_CONSTRAINT_HEADERS
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
SET(GIK_CORE_HEADERS
	hpp/gik/core/solver.hh
	hpp/gik/core/solver-basic.hh
	hpp/gik/core/lne-solver.hh
	hpp/gik/core/motion-plan-column.hh
	hpp/gik/core/motion-plan-element.hh
	hpp/gik/core/motion-plan.hh
	hpp/gik/core/prioritized-motion.hh
	hpp/gik/core/motion-plan-row.hh
)
SET(GIK_MOTIONPLANNER_HEADERS
	hpp/gik/motionplanner/locomotion-plan.hh
	hpp/gik/motionplanner/preview-controller.hh
)
SET(GIK_MOTIONPLANNER_ELEMENT_HEADERS
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
SET(GIK_ROBOT_HEADERS
	hpp/gik/robot/foot-print-related.hh
	hpp/gik/robot/mask-factory.hh
	hpp/gik/robot/robot-motion.hh
	hpp/gik/robot/standing-robot.hh
	hpp/gik/robot/shape-2d.hh
)
SET(GIK_TASK_HEADERS
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

INSTALL(FILES ${GIK_HEADERS}
  DESTINATION include/hpp/gik
)
INSTALL(FILES ${GIK_EXAMPLE_HEADERS}
  DESTINATION include/hpp/gik/example
)
INSTALL(FILES ${GIK_CONSTRAINT_HEADERS}
  DESTINATION include/hpp/gik/constraint
)
INSTALL(FILES ${GIK_CORE_HEADERS}
  DESTINATION include/hpp/gik/core
)
INSTALL(FILES ${GIK_MOTIONPLANNER_HEADERS}
  DESTINATION include/hpp/gik/motionplanner
)
INSTALL(FILES ${GIK_MOTIONPLANNER_ELEMENT_HEADERS}
  DESTINATION include/hpp/gik/motionplanner/element
)
INSTALL(FILES ${GIK_ROBOT_HEADERS}
  DESTINATION include/hpp/gik/robot
)
INSTALL(FILES ${GIK_TASK_HEADERS}
  DESTINATION include/hpp/gik/task
)

#----------------------------------------------------
# Install procedure for the data files
#----------------------------------------------------

SET(DATA_FILES
  hpp/gik/motionplanner/preview-controller-5ms.ini
  hpp/gik/motionplanner/preview-controller-10ms.ini
  hpp/gik/motionplanner/preview-controller-20ms.ini
  hpp/gik/motionplanner/preview-controller-50ms.ini
)
INSTALL(FILES ${DATA_FILES}
  DESTINATION share/hpp/gik
)
