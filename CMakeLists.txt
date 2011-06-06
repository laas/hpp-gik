# Copyright 2010, Oussama Kanoun, LAAS-CNRS
#
# This file is part of hpp-gik.
# hpp-gik is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# hpp-gik is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-gik. If not, see <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Includes from jrl-cmaketools
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)

# Define properties of the project
SET(PROJECT_NAME hpp-gik)
SET(PROJECT_DESCRIPTION "FIXME")
SET(PROJECT_URL "")
ADD_REQUIRED_DEPENDENCY("fsqp >= 2.5")
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.9.0")
ADD_REQUIRED_DEPENDENCY("abstract-gik-task >= 2.4")
ADD_REQUIRED_DEPENDENCY("abstract-robot-dynamics >= 1.16")
ADD_REQUIRED_DEPENDENCY("fsqp >= 2.5")

SET(CXX_DISABLE_WERROR True)

# Initialize generic properties of the project
SETUP_PROJECT()

# To deal with the headers.
#--------------------------
# Create list of headers files
#-----------------------------
SET(${PROJECT_NAME}_HEADERS
	include/hpp/gik/doc.hh
	include/hpp/gik/tools.hh
	include/hpp/gik/example/example.hh
	include/hpp/gik/constraint/joint-state-constraint.hh
	include/hpp/gik/constraint/com-constraint.hh
	include/hpp/gik/constraint/plane-constraint.hh
	include/hpp/gik/constraint/vectorizable-constraint.hh
	include/hpp/gik/constraint/pointing-constraint.hh
	include/hpp/gik/constraint/gaze-constraint.hh
	include/hpp/gik/constraint/parallel-constraint.hh
	include/hpp/gik/constraint/rotation-constraint.hh
	include/hpp/gik/constraint/configuration-constraint.hh
	include/hpp/gik/constraint/position-constraint.hh
	include/hpp/gik/constraint/transformation-constraint.hh
	include/hpp/gik/constraint/prioritized-state-constraint.hh
	include/hpp/gik/constraint/motion-constraint.hh
	include/hpp/gik/core/solver.hh
	include/hpp/gik/core/lne-solver.hh
	include/hpp/gik/core/motion-plan-column.hh
	include/hpp/gik/core/motion-plan-element.hh
	include/hpp/gik/core/motion-plan.hh
	include/hpp/gik/core/prioritized-motion.hh
	include/hpp/gik/core/motion-plan-row.hh
	include/hpp/gik/motionplanner/locomotion-plan.hh
	include/hpp/gik/motionplanner/preview-controller.hh
	include/hpp/gik/motionplanner/element/com-motion.hh
	include/hpp/gik/motionplanner/element/foot-displace-element.hh
	include/hpp/gik/motionplanner/element/interpolated-element.hh
	include/hpp/gik/motionplanner/element/locomotion-element.hh
	include/hpp/gik/motionplanner/element/no-locomotion.hh
	include/hpp/gik/motionplanner/element/ready-element.hh
	include/hpp/gik/motionplanner/element/step-element.hh
	include/hpp/gik/motionplanner/element/walk-element.hh
	include/hpp/gik/motionplanner/element/zmp-shift-element.hh
	include/hpp/gik/robot/foot-print-related.hh
	include/hpp/gik/robot/mask-factory.hh
	include/hpp/gik/robot/robot-motion.hh
	include/hpp/gik/robot/standing-robot.hh
	include/hpp/gik/robot/shape-2d.hh
	include/hpp/gik/task/generic-task.hh
	include/hpp/gik/task/whole-body-task.hh
	include/hpp/gik/task/robot-task.hh
	include/hpp/gik/task/step-back-task.hh
	include/hpp/gik/task/step-task.hh
	include/hpp/gik/task/hand-task.hh
	include/hpp/gik/task/meta-task.hh
	include/hpp/gik/task/reach-task.hh
	include/hpp/gik/task/half-sitting-task.hh
)
SET(${PROJECT_NAME}_DATA_FILES
  include/hpp/gik/motionplanner/preview-controller-5ms.ini
  include/hpp/gik/motionplanner/preview-controller-10ms.ini
  include/hpp/gik/motionplanner/preview-controller-20ms.ini
  include/hpp/gik/motionplanner/preview-controller-50ms.ini
)
INSTALL(FILES ${${PROJECT_NAME}_DATA_FILES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/include/hpp/gik
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)


ADD_SUBDIRECTORY(src)
# Define subdirectories to explore for cmake

# Generate dependency to hpp-gik in pc file
PKG_CONFIG_APPEND_LIBS("hpp-gik")

# Ask to generate the final steps.
SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()


