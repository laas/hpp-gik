#include "tasks/hppGikHalfSittingTask.h"
#include "hppGikTools.h"
#include "constraints/hppGikConfigurationConstraint.h"
#include "motionplanners/elements/hppGikInterpolatedElement.h"



ChppGikHalfSittingTask::ChppGikHalfSittingTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod, "HalfsittingTask")
{
    attStepBackTask = new ChppGikStepBackTask(inStandingRobot,attSamplingPeriod);
    attGenericTask = new ChppGikGenericTask(inStandingRobot, attSamplingPeriod);
}

void ChppGikHalfSittingTask::automaticFoot(bool inAutomatic, bool inSelectedFootIsRight  )
{
    attStepBackTask->automaticFoot( inAutomatic, inSelectedFootIsRight);
}

bool ChppGikHalfSittingTask::algorithmSolve()
{
    //StepBack
    attStepBackTask->targetFeetDistance(attStandingRobot->halfsittingFeetDistance());

    bool isSolved = attStepBackTask->solve();

    cropMotion( attStepBackTask );

    if (!isSolved)
    {
        std::cout << "ChppGikHalfSittingTask::solve(): failure on phase 1.\n";
        return false;
    }

    //Config constraint
    vectorN targetConfig = attStandingRobot->halfsittingConfiguration();
    ChppGikConfigurationConstraint confc( *(attStandingRobot->robot()), targetConfig, attStandingRobot->maskFactory()->wholeBodyMask());
    
    double duration = 4.0;
    ChppGikInterpolatedElement* confe = new ChppGikInterpolatedElement(attStandingRobot->robot(), &confc, 1, 0.0, duration, attSamplingPeriod);

    attGenericTask->clearElements();
    attGenericTask->addElement( confe );
    isSolved = attGenericTask->solve();

    cropMotion( attGenericTask );

    if (!isSolved)
    {
        std::cout << "ChppGikHalfSittingTask::solve(): failure 2.\n";
        return false;
    }

    return isSolved;
}


ChppGikHalfSittingTask::~ChppGikHalfSittingTask()
{
    delete attStepBackTask;
    delete attGenericTask;
    cleanUp();
}
