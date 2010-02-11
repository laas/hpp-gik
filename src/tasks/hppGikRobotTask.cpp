#include "tasks/hppGikRobotTask.h"

ChppGikRobotTask::ChppGikRobotTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, const char* inTaskName)
{
    attStartTime = 0.0;
    
    attShowTime = false;

    attTaskName = inTaskName;
             
    attSamplingPeriod = inSamplingPeriod;
    
    attEps = attSamplingPeriod/2;

    attStandingRobot = inStandingRobot;
    
    attSolutionMotion = new ChppRobotMotion(inStandingRobot->robot(), attStartTime, attSamplingPeriod);

    attInitialConfiguration.resize(inStandingRobot->robot()->numberDof(),false);

    Tps = (struct timeval*) malloc(sizeof(struct timeval));
    Tpf = (struct timeval*) malloc(sizeof(struct timeval));
    Tzp = 0;
}

 void ChppGikRobotTask::showResolutionTime(bool inSwitch)
{
    attShowTime = inSwitch;
}

void ChppGikRobotTask::backupRobot()
{
    attInitialConfiguration = attStandingRobot->robot()->currentConfiguration();
}


void ChppGikRobotTask::restoreRobot()
{
    attStandingRobot->staticState(attInitialConfiguration);
}

const ChppRobotMotion& ChppGikRobotTask::solutionMotion() const
{
    return *attSolutionMotion;
}

void ChppGikRobotTask::cropMotion(ChppGikRobotTask* inRobotTask)
{
    if (!inRobotTask->solutionMotion().empty())
    {
        attSolutionMotion->appendMotion(inRobotTask->solutionMotion());
        ChppRobotMotionSample msample;
        attSolutionMotion->getSampleAtTime(attSolutionMotion->endTime(),msample);
        attStandingRobot->staticState(msample.configuration);
    }
}


bool ChppGikRobotTask::solve()
{

    gettimeofday (Tps, Tzp);

    attSolutionMotion->clear();

    backupRobot();
    
    bool isSolved = algorithmSolve();

    if (isSolved && attShowTime)
    {
        gettimeofday (Tpf, Tzp);
        printf("%s solved in: %ld ms\n", attTaskName.c_str(), (Tpf->tv_sec-Tps->tv_sec)*1000 + (Tpf->tv_usec-Tps->tv_usec)/1000);
    }

    restoreRobot();

    return isSolved;
}

void ChppGikRobotTask::cleanUp()
{
    delete attSolutionMotion;
    free(Tps);
    free(Tpf);
}
