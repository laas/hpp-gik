#ifndef HPP_GIK_STEPBACK_TASK_H
#define HPP_GIK_STEPBACK_TASK_H

#include "tasks/hppGikGenericTask.h"

/**
\brief This is a task to make the robot step back to have his feet parallel and symmetrical.
 */
class ChppGikStepBackTask : public ChppGikRobotTask
{
public:
    /**
        \brief constructor
     */
    ChppGikStepBackTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod);

    /**
        \brief get the target feet fistance between the feet
     */
    double targetFeetDistance();

    /**
        \brief Set the target distance between the feet
     */
    void targetFeetDistance(double inFeetDistance);

    /**
        \brief Destructor
     */
    ~ChppGikStepBackTask();

protected:
    /**
    \brief Solve for joint motion. If the feet are already at target position, this method returns true but the solution motion is empty.
     */
    virtual bool algorithmSolve();
    
private:

    /**
        \brief Target distance between feet
     */
    double attTargetFeetDistance;

    /**
        \brief Normal / Aevrage height of the robot waist
     */
    double attAverageHeight;

    /**
        \brief The solution task plan
     */
    ChppGikGenericTask* attGenericTask;


};
#endif

