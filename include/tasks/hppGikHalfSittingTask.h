#ifndef HPP_GIK_HALFSITTING_TASK_H
#define HPP_GIK_HALFSITTING_TASK_H

#include "tasks/hppGikStepBackTask.h"
#include "tasks/hppGikGenericTask.h"

/**
\brief This robot task consists in going back to half-sitting stance.
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

    /**
    \brief Destructor
     */
    virtual ~ChppGikHalfSittingTask();

protected :

    virtual bool algorithmSolve();
    CjrlRobotConfiguration attHalfSitting;
    ChppGikStepBackTask* attStepBackTask;
    ChppGikGenericTask* attGenericTask;

};
#endif

