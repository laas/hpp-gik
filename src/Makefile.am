# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

include $(top_srcdir)/build-aux/init.mk

# Add available warnings flags.
AM_CXXFLAGS += $(WARNING_CXXFLAGS)

lib_LTLIBRARIES = libhpp-gik.la

libhpp_gik_la_SOURCES = \
tools.cc \
constraint/joint-state-constraint.cc \
constraint/plane-constraint.cc \
constraint/pointing-constraint.cc \
constraint/gaze-constraint.cc \
constraint/parallel-constraint.cc \
constraint/rotation-constraint.cc \
constraint/position-constraint.cc \
constraint/configuration-constraint.cc \
constraint/com-constraint.cc \
constraint/transformation-constraint.cc \
constraint/motion-constraint.cc \
core/solver.cc \
core/lne-solver.cc \
core/motion-plan-column.cc \
core/motion-plan-element.cc \
core/motion-plan.cc \
core/motion-plan-row.cc \
motionplanner/locomotion-plan.cc \
motionplanner/preview-controller.cc \
motionplanner/element/com-motion.cc \
motionplanner/element/foot-displace-element.cc \
motionplanner/element/interpolated-element.cc \
motionplanner/element/no-locomotion.cc \
motionplanner/element/step-element.cc \
motionplanner/element/walk-element.cc \
motionplanner/element/zmp-shift-element.cc \
task/generic-task.cc \
task/whole-body-task.cc \
task/robot-task.cc \
task/step-back-task.cc \
task/step-task.cc \
task/hand-task.cc \
task/meta-task.cc \
task/reach-task.cc \
task/half-sitting-task.cc \
robot/foot-print-related.cc \
robot/mask-factory.cc \
robot/robot-motion.cc \
robot/standing-robot.cc

libhpp_gik_la_LIBADD = $(ABSTRACTGIKTASK_LIBS) $(HRP2DYNAMICS_LIBS) 
libhpp_gik_la_CPPFLAGS = \
	$(AM_CPPFLAGS) $(ABSTRACTGIKTASK_CFLAGS) $(HRP2DYNAMICS_CFLAGS) \
	-DGAINDIR='"${datadir}/hpp/gik"'
