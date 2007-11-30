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

    /**********************************************************************************/
    
    //Select the support foot
    CjrlJoint* fixedFoot = attStandingRobot->robot()->leftFoot(); //arbitrary
    //Record the fixed foot in the robot object:
    attRobot->clearFixedJoints();
    attRobot->addFixedJoint( fixedFoot );
    
    //Create a Gik solver
    ChppGikSolver* gikSolver = new ChppGikSolver(attStandingRobot->robot());

    //Create an empty stack of constraints
    std::vector<CjrlGikStateConstraint*> stack;
    
    //Concatenator
    ChppGikMotionPlanElement* concat = new ChppGikMotionPlanElement(attStandingRobot->robot(), 2);
    
    //current leftfoot
    matrix4d nowlFootH = fixedFoot->currentTransformation();
    
    //computing target transformations for end effectors based on halfsitting config
    ChppGikTransformationConstraint* waisttask = extractTransformationConstraint(lfootH,waistH,nowlFootH,attStandingRobot->robot()->waist(),motionStart,motionDuration,1);

    ChppGikTransformationConstraint* headtask = extractTransformationConstraint(lfootH,headH,nowlFootH,attStandingRobot->robot()->gazeJoint());

    ChppGikTransformationConstraint* lwristtask = extractTransformationConstraint(lfootH,lwristH,nowlFootH,attStandingRobot->robot()->leftWrist());

    ChppGikTransformationConstraint* rwristtask = extractTransformationConstraint(lfootH,rwristH,nowlFootH,attStandingRobot->robot()->rightWrist());
    
    //create the non support foot constraint
    vector3d localPoint;
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->rightFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    ChppGikTransformationConstraint* nsfc = ChppGikTransformationConstraint(*attRobot, nsfJoint, localPoint, nsfTransform);

    stack.push_back(nsfc);

    //create a CoM constraint
    vector3d com = attRobot->positionCenterOfMass();
    ChppGikComConstraint* comc = ChppGikComConstraint(*attRobot, com[0], com[1]);

    stack.push_back(comc);

    //Set the weights used in solving the inverse kinematics problem
    vectorN activated =  attStandingRobot->maskFactory()->wholeBodyMask();
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    vectorN combined = weights;
    for (unsigned int i=0;i<combined.size();i++)
        combined(i) *= activated(i);
    gikSolver->weights(combined);
    
    
    //Solving loop
    vectorN dotq = ublas::zero_vector(attStandingRobot->robot()->numberDof());
    dotq(0) = 1;
    while (ublas::norm(dotq) > 0.2)
    {
        
        
        dotq = attStandingRobot->robot()->currentVelocity();
    }

    return isSolved;
}
