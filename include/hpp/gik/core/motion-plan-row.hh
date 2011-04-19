#ifndef HPP_GIK_MOTION_PLAN_ROW_H
#define HPP_GIK_MOTION_PLAN_ROW_H

#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"
#include "hpp/gik/core/motion-plan-element.hh"
#include "hpp/gik/core/prioritized-motion.hh"


/**
\brief A "motion row" is a container for motion constraints that share the same priority.
This is a low level object.
\ingroup motionsplan
 */
class ChppGikMotionPlanRow
{

public:

    /**
    \brief Constructor
     */
    ChppGikMotionPlanRow(CjrlDynamicRobot* inRobot, unsigned int priority);
    /**
    \brief Get the priority.
    */
    unsigned int priority() const;
    /**
    \brief Get the accuracy.
     */
    double accuracy() const;
    /**
    \brief Add a prioritized motion
    */
    void addMotion(ChppGikPrioritizedMotion* inMotion);
    /**
    \brief Add a prioritized motion
    \return false if the given motion was not referenced in this motion plan row
     */
    bool removeMotion(ChppGikPrioritizedMotion* inMotion);
    /**
    \brief Get the motion element at a given time.
     */
    ChppGikMotionPlanElement* elementAtTime(double inTime);

    /**
    \brief Get lower bound of definition interval.
     */
    double startTime();
    /**
    \brief Get upper bound of definition interval.
     */
    double endTime();
    /**
    \brief Tell if this row is empty
     */
    bool empty() const;

    /**
    \brief Destructor
     */
    ~ChppGikMotionPlanRow();

private:

    void updateStartTime();
    void updateEndTime();
    CjrlDynamicRobot* attRobot;
    double attEndTime;
    double attStartTime;
    std::vector<ChppGikPrioritizedMotion*> attMotions;
    unsigned int attPriority;
    ChppGikMotionPlanElement* attWorkElement;
    vectorN attVector;
};

#endif
