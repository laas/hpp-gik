#include "tasks/hppGikHalfSittingTaskBeta.h"
#include "hppGikTools.h"
#include "constraints/hppGikTransformationConstraint.h"

bool ChppGikHalfSittingTaskBeta::algorithmSolve()
{

    matrix4d &waistH = attStandingRobot->halfsittingWaistTransformation();
    matrix4d &lfootH = attStandingRobot->halfsittingLeftFootTransformation();
    matrix4d &rwristH = attStandingRobot->halfsittingRightWristTransformation();
    matrix4d &lwristH = attStandingRobot->halfsittingLeftWristTransformation();
    matrix4d &headH = attStandingRobot->halfsittingHeadTransformation();

    //StepBack
    attStepBackTask->targetFeetDistance(attStandingRobot->halfsittingFeetDistance());

    bool isSolved = attStepBackTask->solve();
    
    cropMotion( attStepBackTask );
    
    if (!isSolved)
    {
        std::cout << "ChppGikHalfSittingTaskBeta::solve(): failure on phase 1.\n";
        return false;
    }

    //Waist positioning
    matrix4d nowlFootH = attStandingRobot->robot()->leftFoot()->currentTransformation();
    double motionStart = 0.0;
    double motionDuration = 4.0;
    ChppGikSingleMotionElement* waisttask = extractTransformationTask(lfootH,waistH,nowlFootH,attStandingRobot->robot()->waist(),motionStart,motionDuration,1);

    //Head positioning
    ChppGikSingleMotionElement* headtask = extractTransformationTask(lfootH,headH,nowlFootH,attStandingRobot->robot()->gazeJoint(),motionStart,motionDuration,1);

    //Leftwrist positioning
    ChppGikSingleMotionElement* lwristtask = extractTransformationTask(lfootH,lwristH,nowlFootH,attStandingRobot->robot()->leftWrist(),motionStart,motionDuration,1);

    //Rightwrist positioning
    ChppGikSingleMotionElement* rwristtask = extractTransformationTask(lfootH,rwristH,nowlFootH,attStandingRobot->robot()->rightWrist(),motionStart,motionDuration,1);

    attGenericTask->clearElements();
    attGenericTask->addElement( waisttask );
    attGenericTask->addElement( lwristtask );
    attGenericTask->addElement( rwristtask );
    attGenericTask->addElement( headtask );

    //NEW
    attGenericTask->motionReplanning( true );

    isSolved = attGenericTask->solve();

    cropMotion( attGenericTask );
    
    if (!isSolved)
        std::cout << "ChppGikHalfSittingTaskBeta::solve(): failing on phase 2.\n";

    return isSolved;
}
