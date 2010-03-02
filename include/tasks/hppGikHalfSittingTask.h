#warning This header is deprecated,
#warning use hpp/gik/task/half-sitting-task.hh instead.
#ifndef HPP_GIK_HALFSITTING_TASK_H
#define HPP_GIK_HALFSITTING_TASK_H

#include "tasks/hppGikStepBackTask.h"
#include "tasks/hppGikGenericTask.h"


/**
\brief This robot task consists in going back to half-sitting stance.
\ingroup tasks
 */
class ChppGikHalfSittingTask : public ChppGikRobotTask
{

public:

    /**
    \brief constructor
     */
    ChppGikHalfSittingTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod);

    /**
    \brief Manual mode
     */
    void automaticFoot(bool mode = true, bool footIsRight = true );

    void bringBackZMP(bool inChoice, double inStartTime, double inDuration)
    {
        attBringChoice = inChoice;
        attBringStart = inStartTime;
        attBringDuration = inDuration;
    }
            
    /**
    \brief Destructor
     */
    virtual ~ChppGikHalfSittingTask();

protected :

    virtual bool algorithmSolve();
    CjrlRobotConfiguration attHalfSitting;
    ChppGikStepBackTask* attStepBackTask;
    ChppGikGenericTask* attGenericTask;
    double attBringStart,attBringDuration;
    double attBringChoice;

};
#endif

