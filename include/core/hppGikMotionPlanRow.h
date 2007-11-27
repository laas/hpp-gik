#ifndef HPP_GIK_MOTION_PLAN_ROW_H
#define HPP_GIK_MOTION_PLAN_ROW_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikMotionConstraint.h"
#include "core/hppGikMotionPlanElement.h"


/**
\brief A "motion row" is a container for motion constraints that share the same priority.
This is a low level object.
 */
class ChppGikMotionPlanRow
{

public:

    /**
    \brief Constructor
     */
    ChppGikMotionPlanRow(CjrlHumanoidDynamicRobot* inRobot, unsigned int priority);
    /**
    \brief Get the priority.
    */
    unsigned int priority() const;
    /**
    \brief Get the accuracy.
     */
    double accuracy() const;
    /**
    \brief Add a motion constraint
    */
    void addMotionConstraint(CjrlGikMotionConstraint* inJrlMotionConstraint);
    /**
    \brief Add a motion constraint
    \return false if the given motion constraint  was not referenced in this motion plan row
     */
    bool removeMotionConstraint(CjrlGikMotionConstraint* inJrlMotionConstraint);
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

    /**
    \brief Updates Start Time according to stored motion constraints
     */
    void updateStartTime();
    /**
    \brief Updates End Time according to stored motion constraints
     */
    void updateEndTime();

    /**
    \brief Associated standing robot
    */
    CjrlHumanoidDynamicRobot* attRobot;
    /**
    \brief Motion end time
     */
    double attEndTime;
    /**
    \brief Motion start time
     */
    double attStartTime;
    /**
    \brief Vector of state constraints.
     */
    std::vector<CjrlGikMotionConstraint*> attMotionConstraints;
    /**
    \brief The priority
     */
    unsigned int attPriority;
    /**
    \brief Will be used to collect state constraints at a given time into a motion plan element (a task)
     */
    ChppGikMotionPlanElement* attWorkElement;
};

#endif
