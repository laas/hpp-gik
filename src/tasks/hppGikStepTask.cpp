#include "tasks/hppGikStepTask.h"

ChppGikStepTask::ChppGikStepTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, bool inForRightFoot, double relativeTargetX, double relativeTargetY, double relativeTargetTheta):ChppGikRobotTask(inStandingRobot,inSamplingPeriod,"StepTask")
{
    attForRightFoot = inForRightFoot;
    attRelativeTargetX = relativeTargetX;
    attRelativeTargetY = relativeTargetY;
    attRelativeTargetTheta = relativeTargetTheta;

    attGenericTask = new ChppGikGenericTask(inStandingRobot, inSamplingPeriod);
}

void ChppGikStepTask::forRightFoot(bool inOption)
{
    attForRightFoot = inOption;
}


void ChppGikStepTask::relativeTarget( double relativeTargetX, double relativeTargetY, double relativeTargetTheta)
{
    attRelativeTargetX = relativeTargetX;
    attRelativeTargetY = relativeTargetY;
    attRelativeTargetTheta = relativeTargetTheta;
}

bool ChppGikStepTask::algorithmSolve()
{
    if (!attStandingRobot->supportPolygon()->isDoubleSupport())
    {
        std::cout << "ChppGikWholeBodyTask::makeStep() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    ChppGikFootprint* targetFootprint;
    const ChppGikFootprint *supportFootprint;

    if (attForRightFoot)
    {
        supportFootprint = attStandingRobot->supportPolygon()->leftFootprint();
        std::cout << "Note from task planner: stepping with the right foot\n";
    }
    else
    {
        supportFootprint = attStandingRobot->supportPolygon()->rightFootprint();
        std::cout << "Note from task planner: stepping with the left foot\n";
    }

    targetFootprint =  new ChppGikFootprint(attRelativeTargetX, attRelativeTargetY, attRelativeTargetTheta);
    ChppGikFootprint::makeAbsolute(supportFootprint, targetFootprint);

    double stepStartTime = 0.0;
    double zmpstartshifttime = 0.2;
    double zmpendshifttime = 0.4;
    double footflighttime = 0.9;
    double zmpendcoef = 0.5;

    ChppGikStepElement* stepElement = new ChppGikStepElement(stepStartTime, targetFootprint, attForRightFoot, zmpendcoef, zmpendshifttime, zmpstartshifttime, footflighttime);

    attGenericTask->clearElements();
    attGenericTask->addElement(stepElement);

    delete targetFootprint;

    //Solve
    bool isSolved = attGenericTask->solve();

    if (isSolved)
        cropMotion( attGenericTask );

    return isSolved;

}

ChppGikStepTask::~ChppGikStepTask()
{
    delete attGenericTask;
}
