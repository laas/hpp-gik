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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME hpp-gik)
SET(PROJECT_DESCRIPTION "Generalized inversed kinetmatics")
SET(PROJECT_URL "http://github.com/laas/hpp-gik")

SET(CXX_DISABLE_WERROR 1)

SETUP_PROJECT()

# Add option to disable tests.
SET(DISABLE_TESTS OFF CACHE BOOL
 "Disable tests (and remove the dependency toward HRP-2 confidential packages)")

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.8.0")
ADD_REQUIRED_DEPENDENCY("abstract-gik-task >= 2.4")

IF(NOT DISABLE_TESTS)
  ADD_REQUIRED_DEPENDENCY("hrp2-dynamics >= 1.4.4")
ENDIF()

# Search for dependencies.
# Boost
SEARCH_FOR_BOOST()

# Add subdirectories.
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)

IF(NOT DISABLE_TESTS)
  ADD_SUBDIRECTORY(tests)
ENDIF()

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
