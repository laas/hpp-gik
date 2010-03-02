#include "hpp/gik/task/meta-task.hh"


ChppGikMetaTask::ChppGikMetaTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod,"MetaTask")
{
    attBringChoice = false;
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
    
    
    bool bringZMPDone = false;
    for (unsigned int i=0; i< attTasks.size(); i++ )
    {
        if (!bringZMPDone)
        {
            attTasks[i]->bringBackZMP(attBringChoice,attBringStart,attBringDuration);
        }
        else
        {
            attTasks[i]->bringBackZMP(false,0,0);
        }
        
        solved = attTasks[i]->solve();
        cropMotion( attTasks[i]);
        if (!(attTasks[i]->solutionMotion().empty()))
            bringZMPDone = true;
        
        if (!solved)
        {
            std::cout << "MetaTask: task ranked "<< i <<" not solved. Abort.\n";
            break;
        }
    }
    
    return solved;
}

ChppGikMetaTask::~ChppGikMetaTask()
{
    cleanUp();
}
