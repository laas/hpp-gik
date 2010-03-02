/**
\page example1_page Example I
\section intro_sec Introduction
In this basic example, a motion for the center of the right hand is planned while projection of the center of mass on the ground and the feet's positions are maintained static.

\section code_sec Code

Set a sampling period: 5ms second is the value used on HRP2
 
\code
    attSamplingPeriod = 5e-3;
\endcode
Supposing we have already constructed a CjrlHumanoidDynamicRobot object in halfsitting configuration and for which we have stored a pointer \c attRobot , create a standing robot:
\code
    ChppGikStandingRobot attStandingRobot(attRobot);
\endcode

Create a Gik solver
\code
    ChppGikSolver gikSolver(*attRobot);
\endcode

Select the root of the robot at the right foot (reduces computation complexity)
\code
    gikSolver.rootJoint(attRobot->rightAnkle());
\endcode

Create a ChppRobotMotion (where the successive configurations will be stored)
\code
    ChppRobotMotion attSolutionMotion(attRobot, 0.0 , attSamplingPeriod);
\endcode
 
Miscelleaenous variables
\code
    vector3d targetPoint, localPoint, p;
    vector3d absZMPPla, absZMPObs, relZMPObs, relZMPPla;
\endcode
 
Create the constraints defing the tasks.
<br>
First priority: Foot on the ground
\code
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->leftAnkle());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    ChppGikTransformationConstraint nsfc(*attRobot, nsfJoint, localPoint, nsfTransform);
\endcode
    
Second priority: static Center of Mass
\code
    vector3d com = attRobot->positionCenterOfMass();
    ChppGikComConstraint comc(*attRobot, com[0], com[1]);
    absZMPPla = com;
\endcode
    
Third priority: A position constraint on a point in the right wrist frame
\code
    matrix4d curT=  attRobot->rightWrist()->currentTransformation();
    CjrlJoint& rwJoint = *(attRobot->rightWrist());
    attRobot->rightHand()->getCenter(localPoint);
    ChppGikPositionConstraint pc(*attRobot,rwJoint,localPoint, curT*localPoint);
\endcode
Stack the constraints in a vector
\code
    std::vector<CjrlGikStateConstraint*> stack;
    stack.push_back(&nsfc);
    stack.push_back(&comc);
    stack.push_back(&pc);
\endcode
 
Build the weights used in solving the pseudo inverse kinematics
\code
    vectorN activated =  attStandingRobot->maskFactory()->wholeBodyMask();// active joints
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport(); //With these weights, the more mass a joint is lifting the less it's used
    vectorN combined = weights;
    for (unsigned int i=0;i<combined.size();i++)
        combined(i) *= activated(i);
    for (unsigned int i=0;i<6;i++)
        combined(i) = 0.0; // the absolute trnasformation (freeflyer) of the robot is not controlled
\endcode

Set the weights in the solver
\code
        gikSolver.weights(combined);
\endcode
 
Start the loop where we continuously change the desired target position of the hand, thus generating a motion:
\code
    for (unsigned int j = 0; j< 500;j++)
    {
        
\endcode
Change the target position for the hand slightly:
\code
        p = pc->worldTarget();
        p[0] += 0.001;
        pc->worldTarget(p);
\endcode
Attempt solve with a single one step:
Prepare the constraints (jacobian and value computation)
\code
        gikSolver.prepare(stack);
\endcode
        Solve
\code
        gikSolver.solve( stack );
\endcode
        Apply solution to robot
\code
        attStandingRobot->updateRobot(gikSolver.solutionRootPose(), gikSolver.solutionJointConfiguration(), attSamplingPeriod);
    }
\endcode
*/


