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
ADD_REQUIRED_DEPENDENCY("jrl-dynamics >= 1.16")
ADD_REQUIRED_DEPENDENCY("hrp2-dynamics")

SET(CXX_DISABLE_WERROR True)

# Initialize generic properties of the project
SETUP_PROJECT()

# To deal with the headers.
#--------------------------
# Create list of headers files
#-----------------------------


ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(tests)

# Define subdirectories to explore for cmake

# Generate dependency to hpp-gik in pc file
PKG_CONFIG_APPEND_LIBS("hpp-gik")

# Ask to generate the final steps.
SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()


