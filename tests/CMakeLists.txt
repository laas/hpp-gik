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

ADD_DEFINITIONS(-DDEBUG=2)

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# HPP_GIK_TEST(NAME)
# ------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(HPP_GIK_TEST NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} hrp2-dynamics)

  # Link against Boost.
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_LIBRARIES})

  TARGET_LINK_LIBRARIES(${NAME} hpp-gik)
ENDMACRO(HPP_GIK_TEST)

HPP_GIK_TEST(tests)
