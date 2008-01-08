#include "tasks/hppGikHalfSittingTask.h"
#include "hppGikTools.h"
#include "constraints/hppGikTransformationConstraint.h"

ChppGikHalfSittingTask::ChppGikHalfSittingTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod, "HalfsittingTask")
{
    attUpperBodyTask = new ChppGikConfigurationTask(inStandingRobot,inSamplingPeriod,inStandingRobot->halfsittingConfiguration());

    attStepBackTask = new ChppGikStepBackTask(inStandingRobot,attSamplingPeriod);
    
    attGenericTask = new ChppGikGenericTask(inStandingRobot, inSamplingPeriod);
}

void ChppGikHalfSittingTask::automaticFoot(bool inAutomatic, bool inSelectedFootIsRight  )
{
    attStepBackTask->automaticFoot( inAutomatic, inSelectedFootIsRight);
}
    
bool ChppGikHalfSittingTask::algorithmSolve()
{
    matrix4d &waistH = attStandingRobot->halfsittingWaistTransformation();
    matrix4d &rfootH = attStandingRobot->halfsittingRightFootTransformation();
    matrix4d &lfootH = attStandingRobot->halfsittingLeftFootTransformation();

    //StepBack
    attStepBackTask->targetFeetDistance(attStandingRobot->halfsittingFeetDistance());

    bool isSolved = attStepBackTask->solve();
    
    cropMotion( attStepBackTask );
    
    if (!isSolved)
    {
        std::cout << "ChppGikHalfSittingTask::solve(): failure on phase 1.\n";
        return false;
    }

    //Waist positioning
    matrix4d nowlFootH = attStandingRobot->robot()->leftFoot()->currentTransformation();
    double waistMotionStart = 0.0;
    double waistMotionDuration = 3.0;
    ChppGikSingleMotionElement* waisttask = extractTransformationTask(lfootH,waistH,nowlFootH,attStandingRobot->robot()->waist(),waistMotionStart,waistMotionDuration,2);
    waisttask->workingJoints(attStandingRobot->maskFactory()->wholeBodyMask());
    
    attGenericTask->clearElements();
    attGenericTask->addElement( waisttask );

    isSolved = attGenericTask->solve();

    cropMotion( attGenericTask );
    
    if (!isSolved)
    {
        std::cout << "ChppGikHalfSittingTask::solve(): failing on phase 2.\n";
        return false;
    }

    //Upper body
    isSolved = attUpperBodyTask->solve();

    cropMotion( attUpperBodyTask );
    
    if (!isSolved)
        std::cout << "ChppGikHalfSittingTask::solve(): failing on phase 3.\n";
    
    return isSolved;
}

ChppGikSingleMotionElement* ChppGikHalfSittingTask::extractTransformationTask(const matrix4d& refBase, const matrix4d& refTarget, const matrix4d& nowBase, CjrlJoint* joint, double startTime,double  duration, unsigned int priority )
{
    matrix4d targetM4;
    ChppGikTools::targetTransformationM(refBase,refTarget,nowBase,targetM4);

    ublas::zero_vector<double> zero3U(3);
    vector3d localPoint;
    ChppGikTools::UblastoVector3( zero3U, localPoint);

    ChppGikTransformationConstraint* constraint = new ChppGikTransformationConstraint(*(attStandingRobot->robot()), *joint,localPoint, targetM4);

    ChppGikSingleMotionElement* task = new ChppGikSingleMotionElement(constraint, priority, startTime, duration);

    delete constraint;
    return task;
}

ChppGikHalfSittingTask::~ChppGikHalfSittingTask()
{
    delete attStepBackTask;
    delete attUpperBodyTask;
    delete attGenericTask;
    cleanUp();
}
