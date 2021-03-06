#ifndef HPP_GIK_HALFSITTING_TASK_H
#define HPP_GIK_HALFSITTING_TASK_H

#include "hpp/gik/task/step-back-task.hh"
#include "hpp/gik/task/generic-task.hh"


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

