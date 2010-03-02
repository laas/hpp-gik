#ifndef HPP_GIK_META_TASK_H
#define HPP_GIK_META_TASK_H

#include "tasks/hppGikRobotTask.h"

/**
\brief This is a robot task  composed of a sequence of robot tasks.
\ingroup tasks
 */
class ChppGikMetaTask : public ChppGikRobotTask
{
public:
    /**
        \brief constructor
     */
    ChppGikMetaTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod);

    /**
        \brief add a robot task at the end of the current stack of tasks
     */
    void pushbackTask(ChppGikRobotTask* inRobotTask);

    /**
        \brief clear stack of robot tasks
     */
    void clearTasks();

    void bringBackZMP(bool inChoice, double inStartTime, double inDuration)
    {
        attBringChoice = inChoice;
        attBringStart = inStartTime;
        attBringDuration = inDuration;
    }
    
    /**
        \brief Destructor
     */
    ~ChppGikMetaTask();

protected:
    /**
        \brief Solve for joint motion. Robot must have a double support polygon
     */
    virtual bool algorithmSolve();

private:

    std::vector<ChppGikRobotTask*> attTasks;
    double attBringStart,attBringDuration;
    double attBringChoice;

};
#endif
