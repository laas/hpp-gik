#warning This header is deprecated,
#warning use hpp/gik/task/step-back-task.hh instead.
#ifndef HPP_GIK_STEPBACK_TASK_H
#define HPP_GIK_STEPBACK_TASK_H

#include "tasks/hppGikGenericTask.h"

/**
\brief This is a task to make the robot step back to have his feet parallel and symmetrical.
\ingroup tasks
 */
class ChppGikStepBackTask : public ChppGikRobotTask
{
public:
    /**
        \brief constructor
     */
    ChppGikStepBackTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod);

    /**
        \brief Manual mode
    */
    void automaticFoot(bool mode = true, bool footIsRight = true );
    
    /**
        \brief get the target feet fistance between the feet
     */
    double targetFeetDistance();

    /**
        \brief Set the target distance between the feet
     */
    void targetFeetDistance(double inFeetDistance);

    void bringBackZMP(bool inChoice, double inStartTime, double inDuration)
    {
        attGenericTask->bringBackZMP( inChoice,inStartTime,inDuration );
    }
    
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
    
    /**
        \brief Says how the moved foot is selected
    */
    bool attAutomaticFoot;
    
    /**
        \brief Says which foot to move if in manual mode. true = right foot
     */
    bool attSelectedFootisRight;
    


};
#endif

