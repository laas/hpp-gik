#ifndef HPP_GIK_MOTION_PLAN_H
#define HPP_GIK_MOTION_PLAN_H

#include "core/hppGikMotionPlanColumn.h"
#include "core/hppGikMotionPlanRow.h"

/**
\brief The motion plan contains references to planned motion constraints, organized by descending priority in motion plan rows.
 
References to motion constraints are stacked according to their priority:
A motion plan is a structure that resembles to a matrix of "motion plan elements".
A "motion plan element" is a container for state constraints that share the same priority. A "motion element" is "a task" in the scheme of prioritized task solving using generalized inverse kinematics.
A "motion plan row" is a container for motion constraints that share the same priority, so the horizontal direction is for time progression.
A "motion plan column" is a container for the motion plan elements that can be collected at a given time. The vertical direction stacks the planned tasks at a given time.
 
 */

class ChppGikMotionPlan
{
public:

    /**
    \brief Constructor
     */
    ChppGikMotionPlan(CjrlHumanoidDynamicRobot* inRobot);
    /**
    \brief Destructor
     */
    ~ChppGikMotionPlan();
    /**
    \brief Return associated robot
     */
    CjrlHumanoidDynamicRobot* robot();

    /**
    \brief Add a prioritized motion constraint.
    If a row having the same priority is found it gets the new motion constraint, otherwise a new row is created for it.
    /return a pointer to the ChppGikMotionPlanRow that holds the entered motion constraint
    /note null pointer is returned in case the adde motion is not defined for this motion plan's robot
     */
    ChppGikMotionPlanRow* addMotion(ChppGikPrioritizedMotion* inMotion);

    /**
    \brief Get number of row corresponding to given priority value
    \return false if the given priority does not have a matching row
     */
    bool getRankForPriority(unsigned int inPriority, unsigned int& outRownumber);

    /**
    \brief Get a pointer to the row at the given rank (starting from 0).
    \return null pointer if inRank is out of range
     */
    ChppGikMotionPlanRow* getRow(unsigned int inRank);
    
    /**
    \brief Tell is the motion plan is empty
     */
    bool empty();

    /**
    \brief Compute and get lower bound of definition interval.
     */
    double startTime();
    /**
    \brief Compute and get upper bound of definition interval.
     */
    double endTime();

    /**
    \brief Get the motion plan column at the given time.
     */
    ChppGikMotionPlanColumn* columnAtTime(double inTime);
    
    /**
    \brief Get the motion plan column at the given time.
     */
    std::vector<ChppGikPrioritizedMotion*> activeAtTime(double inTime);

    
private:

    void updateEndTime();
    void updateStartTime();
    CjrlHumanoidDynamicRobot* attRobot;
    double attEndTime;
    double attStartTime;
    std::vector<ChppGikMotionPlanRow*> attRows;
    ChppGikMotionPlanColumn* attWorkColumn;
    vectorN attVector;
};

#endif

