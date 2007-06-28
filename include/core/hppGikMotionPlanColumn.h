#ifndef HPP_GIK_MOTION_PLAN_COLUMN_H
#define HPP_GIK_MOTION_PLAN_COLUMN_H


#include "core/hppGikMotionPlanElement.h"

/**
\brief A motion plan column is a column of motion elements inside a motion plan. It is the equivalent of a prioritized stack of tasks in the generalized inverse kinematics scheme.
 
This is a low level object.
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
    ChppGikMotionPlanColumn(CjrlHumanoidDynamicRobot* inRobot);
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

    /**
        \brief Pointer to the relevant robot.
     */
    CjrlHumanoidDynamicRobot* attRobot;
    /**
        \brief Vector (stack) of motion plan elements (tasks)
     */
    std::vector<ChppGikMotionPlanElement*> attTaskStack;

};

#endif
