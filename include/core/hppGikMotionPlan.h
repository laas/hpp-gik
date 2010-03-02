#warning This header is deprecated,
#warning use hpp/gik/core/motion-plan.hh instead.
#ifndef HPP_GIK_MOTION_PLAN_H
#define HPP_GIK_MOTION_PLAN_H

#include "core/hppGikMotionPlanColumn.h"
#include "core/hppGikMotionPlanRow.h"

/**
The Motion Plan contains references to ChppGikPrioritizedMotion objects, sorted by descending priority in ChppGikMotionPlanRow objects.
\ingroup motionsplan
 */

class ChppGikMotionPlan
{
public:

    /**
    \brief Constructor
     */
    ChppGikMotionPlan(CjrlDynamicRobot* inRobot);
    /**
    \brief Destructor
     */
    ~ChppGikMotionPlan();
    /**
    \brief Return associated robot
     */
    CjrlDynamicRobot* robot();

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
    CjrlDynamicRobot* attRobot;
    double attEndTime;
    double attStartTime;
    std::vector<ChppGikMotionPlanRow*> attRows;
    ChppGikMotionPlanColumn* attWorkColumn;
    vectorN attVector;
};

#endif

