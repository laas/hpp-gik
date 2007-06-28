#include "tasks/hppGikMetaTask.h"


ChppGikMetaTask::ChppGikMetaTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod,"MetaTask")
{
}

void ChppGikMetaTask::pushbackTask(ChppGikRobotTask* inRobotTask )
{
    attTasks.push_back(inRobotTask);
}

void ChppGikMetaTask::clearTasks()
{
    attTasks.clear();
}

bool ChppGikMetaTask::algorithmSolve()
{
    if (attTasks.empty())
    {
        std::cout << "MetaTask empty, nothing to solve\n";
        return false;
    }
    
    bool solved = false;
    
    for (unsigned int i=0; i< attTasks.size(); i++ )
    {
        solved = attTasks[i]->solve();
        if (solved)
            cropMotion( attTasks[i]);
        else
            break;
    }
    
    return solved;
}

ChppGikMetaTask::~ChppGikMetaTask()
{
}
