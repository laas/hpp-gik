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

ADD_DEFINITIONS(-DHPP_GIK_GAINDIR="${CMAKE_INSTALL_PREFIX}/share/hpp/gik")

ADD_LIBRARY(hpp-gik
  SHARED
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
  constraint/relative-com-constraint.cc
  constraint/relative-transformation-constraint.cc
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

PKG_CONFIG_USE_DEPENDENCY(hpp-gik abstract-gik-task)
PKG_CONFIG_USE_DEPENDENCY(hpp-gik abstract-robot-dynamics)
PKG_CONFIG_USE_DEPENDENCY(hpp-gik jrl-mal)

SET_TARGET_PROPERTIES(hpp-gik PROPERTIES SOVERSION ${PROJECT_VERSION})

INSTALL(TARGETS hpp-gik DESTINATION lib)
