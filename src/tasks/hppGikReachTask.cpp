#include "tasks/hppGikReachTask.h"

#include "constraints/hppGikParallelConstraint.h"
#include "constraints/hppGikPositionConstraint.h"
#include "constraints/hppGikGazeConstraint.h"
#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J

ChppGikReachTask::ChppGikReachTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod,"ReachTask")
{
    attWholeBodyTask = new ChppGikWholeBodyTask(inStandingRobot, inSamplingPeriod, 1);

    attTargetSet = false;
    attEnableGaze = false;
    attEnableOrientation = false;
    attEnableStepping = true;

}

ChppGikReachTask::~ChppGikReachTask()
{
    delete attWholeBodyTask;
    cleanUp();
}

void ChppGikReachTask::target(const vector3d& inReachTarget, bool isForRightHand)
{
    attReachTarget = inReachTarget;
    attForRightHand = isForRightHand;
    attTargetSet = true;
}


void ChppGikReachTask::gazeTarget(const vector3d& inGazeTarget)
{
    attEnableGaze = true;
    attGazeTarget = inGazeTarget;
}


void ChppGikReachTask::disableGaze()
{
    attEnableGaze = false;
}


void ChppGikReachTask::handOrientation(const vector3d& inTargetOrientation)
{
    attEnableOrientation = true;
    attHandTargetAxis = inTargetOrientation;
}


void ChppGikReachTask::disableOrientation()
{
    attEnableOrientation = false;
}

void ChppGikReachTask::steppingEnabled(bool inOption)
{
    attEnableStepping = inOption;
}


bool ChppGikReachTask::algorithmSolve()
{
    if (!attTargetSet)
    {
        std::cout <<"ChppGikReachTask::algorithmSolve() set a target first\n";
        return false;
    }

    //Determin hand
    CjrlJoint* joint = 0;
    CjrlHand* hand = 0;

    if (attForRightHand)
    {
        joint = attStandingRobot->robot()->rightWrist();
        hand = attStandingRobot->robot()->rightHand();
    }
    else
    {
        joint = attStandingRobot->robot()->leftWrist();
        hand = attStandingRobot->robot()->leftHand();
    }
    
    //reach
    bool solved;
    attWholeBodyTask->reset();
    vector3d lpoint,laxis;
    lpoint = hand->centerInWristFrame();
    CjrlGikPositionConstraint* psc = 0;
    CjrlGikParallelConstraint* poc = 0;
    CjrlGikGazeConstraint* gc = 0;
    psc = new ChppGikPositionConstraint(*(attStandingRobot->robot()),*joint,lpoint,attReachTarget);
    unsigned int priority = 1;
    attWholeBodyTask->addStateConstraint(psc,priority);
    if (attEnableOrientation)
    {
        laxis = hand->okayAxisInWristFrame();
        poc = new ChppGikParallelConstraint(*(attStandingRobot->robot()),*joint,laxis,attHandTargetAxis);
        priority = 2;
        attWholeBodyTask->addStateConstraint(poc,priority);
    }
    if (attEnableGaze)
    {
        gc = new ChppGikGazeConstraint(*(attStandingRobot->robot()), attGazeTarget);
        priority = 3;
        attWholeBodyTask->addStateConstraint(gc,priority);
    }
    attWholeBodyTask->enableStepping( attEnableStepping );
    solved = attWholeBodyTask->solve();
    
    delete psc;
    delete poc;
    delete gc;
    
    cropMotion(attWholeBodyTask);

    return solved;
}
