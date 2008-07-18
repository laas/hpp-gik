/** \mainpage 
\section sec_intro Introduction

This package implements generalized inverse kinematics solver and whole body motion planning algorithms for humanoid biped robot.

*/

/**
\defgroup constraints State Constraints
\brief Various objects implementing abstract class CjrlGikStateConstraint
*/

/**
\defgroup solver Prioritized State Constraints Solvers
\brief For solving prioritized linear systems given by CjrlGikStateConstraint objects
 */

/**
\defgroup motions Prioritized Motions
\brief Various objects implementing abstract class ChppGikPrioritizedMotion.
 */

/**
\defgroup motionsplan Prioritized Motions Plan
\brief ChppGikPrioritizedMotion organizer.
The Motion Plan contains references to ChppGikPrioritizedMotion objects, organized by descending priority in motion plan rows. References to ChppGikPrioritizedMotion are stacked according to their priority.<br>
A ChppGikMotionPlan is equivalent to a matrix of ChppGikMotionPlanElement objects. <br>
A ChppGikMotionPlanElement is a container for CjrlGikStateConstraint objects that share the same priority.<br>
A ChppGikMotionPlanRow is a container for ChppGikPrioritizedMotion that share the same priority, so the horizontal direction is for time progression. <br>
A ChppGikMotionPlanColumn is a container for all the ChppGikMotionPlanElement objects that can be collected at a given time. The vertical direction stacks the constaints at a given time.
<br>
<br>
This module was originally intended for internal use.
 */

/**
\defgroup motionsolver Prioritized Motions Solver
\brief Compute a whole body joint motion ( ChppRobotMotion object ) according to entered 
ChppGikPrioritizedMotion objects and/or ChppGikLocomotionElement objects.
 */

/**
\defgroup tasks Dedicated Tasks
\brief  Dedicated Tasks are whole body joint motion planners for specific cases.
For example, a ChppGikReachTask will produce reaching motion for a humanoid robot, possibly involving a step, based on a target's cartesian position and a few more option specifications.
Dedicated tasks derive from Abstract class ChppGikRobotTask.
*/

/**
\defgroup robot Robot
\brief Additional information describing a Humnoid biped robot state.
 */

/**
\defgroup tools Tools
\brief Various mathematical function used in this packages are regrouped here.
 */
