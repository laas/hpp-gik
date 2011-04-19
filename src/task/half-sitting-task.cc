#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/task/half-sitting-task.hh"
#include "hpp/gik/tools.hh"
#include "hpp/gik/constraint/configuration-constraint.hh"
#include "hpp/gik/motionplanner/element/interpolated-element.hh"



ChppGikHalfSittingTask::ChppGikHalfSittingTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod, "HalfsittingTask")
{
    attStepBackTask = new ChppGikStepBackTask(inStandingRobot,attSamplingPeriod);
    attGenericTask = new ChppGikGenericTask(inStandingRobot, attSamplingPeriod);
    attBringChoice = false;
}

void ChppGikHalfSittingTask::automaticFoot(bool inAutomatic, bool inSelectedFootIsRight  )
{
    attStepBackTask->automaticFoot( inAutomatic, inSelectedFootIsRight);
}

bool ChppGikHalfSittingTask::algorithmSolve()
{
    attGenericTask->bringBackZMP( false,0,0);
            
    //StepBack
    attStepBackTask->targetFeetDistance(attStandingRobot->halfsittingFeetDistance());
    attStepBackTask->bringBackZMP( attBringChoice,attBringStart,attBringDuration);
    
    bool isSolved = attStepBackTask->solve();

    cropMotion( attStepBackTask );

    if (!isSolved)
    {
        std::cout << "ChppGikHalfSittingTask::solve(): failure on phase 1.\n";
        return false;
    }
    
    if (attStepBackTask->solutionMotion().empty())
    {
        attGenericTask->bringBackZMP( attBringChoice,attBringStart,attBringDuration);
    }
        

    //Config constraint
    vectorN targetConfig = attStandingRobot->halfsittingConfiguration();
    ChppGikConfigurationConstraint confc( *(attStandingRobot->robot()), targetConfig, attStandingRobot->maskFactory()->wholeBodyMask());
   
    double duration = 4.0;
    ChppGikInterpolatedElement* confe = new ChppGikInterpolatedElement(attStandingRobot->robot(), &confc, 3, 0.0, duration, attSamplingPeriod);

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
