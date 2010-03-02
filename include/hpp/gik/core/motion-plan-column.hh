#ifndef HPP_GIK_MOTION_PLAN_COLUMN_H
#define HPP_GIK_MOTION_PLAN_COLUMN_H


#include "hpp/gik/core/motion-plan-element.hh"

/**
\brief A motion plan column is a column of motion elements inside a motion plan. It is the equivalent of a prioritized stack of constraints.
\ingroup motionsplan
 */
class ChppGikMotionPlanColumn
{
public:
    /**
        \name Definition
        @{
     */
    /**
        \brief Constructor
     */
    ChppGikMotionPlanColumn(CjrlDynamicRobot* inRobot);
    /**
        \brief Add a motion plan element (task).
        The task is automatically inserted at the adequate rank of the storing vector according to the priority.
     */
    void addElement(ChppGikMotionPlanElement* inTask);

    /**
        \brief Get the number of motion plan elements in this column
     */
    unsigned int numberElements()const;

    /**
        \brief Get a mask on the configuration vector denoting the working degrees of freedom
     */
    const vectorN& workingJoints() const;

    /**
        \brief Set a mask on the configuration vector denoting the working degrees of freedom
     */
    void workingJoints(const vectorN& inVec);

    /**
        \brief Get a vector of pointers to jrl state constraints sorted in descending priority
     */
    std::vector<CjrlGikStateConstraint*> constraints();

    /**
        \brief Clear motion plan elements (clear stack)
     */
    void clear();
    /**
        @}
     */

    /**
        \brief Destructor
     */
    ~ChppGikMotionPlanColumn();

private:

    CjrlDynamicRobot* attRobot;
    std::vector<ChppGikMotionPlanElement*> attTaskStack;
    vectorN attWorkingJoints;

};

#endif
