#warning This header is deprecated,
#warning use hpp/gik/task/step-task.hh instead.
#ifndef HPP_GIK_STEP_TASK_H
#define HPP_GIK_STEP_TASK_H

#include "tasks/hppGikGenericTask.h"

/**
\brief This is a task to make the robot do a single step.
Call solve() to plan joint motion for a step given in support foot frame (origin is projection of ankle on floor, x axis pointing to the toes, y axis pointing to the left and z upward). Orientation of displaced foot is defined by the angle it will make with axis x of support foot.
\ingroup tasks
 */

class ChppGikStepTask : public ChppGikRobotTask
{
public:
    /**
        \brief constructor
     */
    ChppGikStepTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, bool inForRightFoot, double relativeTargetX, double relativeTargetY, double relativeTargetTheta);

    /**
        \brief set the foot for this Step. true = right foot, false = left foot
    */
    void forRightFoot(bool inOption);
    
    /**
        \brief Set the target in the frame of the support foot (relative step)
     */
    void relativeTarget( double relativeTargetX, double relativeTargetY, double relativeTargetTheta);
    
    void bringBackZMP(bool inChoice, double inStartTime, double inDuration)
    {
        attGenericTask->bringBackZMP( inChoice,inStartTime,inDuration );
    }
    
    /**
        \brief Destructor
     */
    ~ChppGikStepTask();

protected:
    /**
        \brief Solve for joint motion. Robot must have a double support polygon
     */
    virtual bool algorithmSolve();

private:
    
    /**
    \brief Pointer to the displaced foot
    */
    bool attForRightFoot;

    /**
    \brief The solution task plan
     */
    ChppGikGenericTask* attGenericTask;

    /**
    \name Relative target coordinates
    {@
     */
    double attRelativeTargetX;
    double attRelativeTargetY;
    double attRelativeTargetTheta;
    /**
    @}
    */

};
#endif
