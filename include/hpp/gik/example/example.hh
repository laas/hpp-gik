/**
\page example1_page Example I
\section intro_sec Introduction
In this basic example, a motion for the center of the right hand is planned while projection of the center of mass on the ground and the feet's positions are maintained static.

        \section code_sec Code

        Set a sampling period: 5ms second is the value used on HRP2
 
        \code
        samplingPeriod = 5e-3;
        \endcode

Assuming that we have already constructed a CjrlHumanoidDynamicRobot object in halfsitting configuration and for which we have stored a pointer \c robot, create a standing robot:
                \code

                standingRobot = new ChppGikStandingRobot ( *robot );
        \endcode
\note The degrees of freedom of the joints of the robot need to be bounded with actual values. If left to 0, the robot will not move.

                Create a Gik solver
                \code
                ChppGikSolver gikSolver(*robot);
        \endcode

                Select the root of the robot at the right foot (reduces computation complexity)
                \code
                gikSolver.rootJoint(*robot->rightAnkle());
        \endcode

                Create an empty motion (optional)
                \code
                ChppRobotMotion solutionMotion(robot, 0.0 , samplingPeriod);
        vector3d ZMPworPla;//planned ZMP in absolute frame
        vector3d ZMPwstPla;//Planned ZMP in robot waist frame
        vector3d ZMPworObs;//Observed ZMP in absolute frame
        vector3d ZMPwstObs;//Observed ZMP in robot waist frame
        ZMPworPla = com; // the ZMP is planned to remain at its inital position
        \endcode

                Create the constraints defing the tasks.
                <br>
                First priority: Foot on the ground
                \code
                vector3d localPoint;
        localPoint[0] = 0.0;
        localPoint[1] = 0.0;
        localPoint[2] = 0.0;
        CjrlJoint& nsfJoint = *(robot->leftAnkle());
        matrix4d nsfTransform = nsfJoint.currentTransformation();
        ChppGikTransformationConstraint nsfc(*robot, nsfJoint, localPoint, nsfTransform);
        \endcode
    
                Second priority: static Center of Mass
                \code
                vector3d com = robot->positionCenterOfMass();
        ChppGikComConstraint comc(*robot, com[0], com[1]);
    
        \endcode
    
                Third priority: A position constraint on a point in the right wrist frame
                \code
                matrix4d curT=  robot->rightWrist()->currentTransformation();
        CjrlJoint& rwJoint = *(robot->rightWrist());
        robot->rightHand()->getCenter(localPoint);
        ChppGikPositionConstraint pc(*robot,rwJoint,localPoint, curT*localPoint);
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
                vectorN activated =  standingRobot->maskFactory()->wholeBodyMask();// active joints
        vectorN weights = standingRobot->maskFactory()->weightsDoubleSupport(); //With these weights, the more mass a joint is lifting the less it's used
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
                vector3d p;
        matrix4d waistTransform, inverseWaistTransform;
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
            standingRobot->updateRobot(gikSolver.solutionRootPose(), gikSolver.solutionJointConfiguration(), samplingPeriod);
    \endcode
            Store the computed sample (optional)
            \code
        //compute planned and observed ZMP in absolute and waist frames
            ZMPworObs = gikSolver.zeroMomentumPoint();
    waistTransform = robot->waist()->currentTransformation();
    MAL_S4x4_INVERSE(waistTransform,inverseWaistTransform,double);
    MAL_S4x4_C_eq_A_by_B(ZMPwstObs,inverseWaistTransform,ZMPworObs);
    MAL_S4x4_C_eq_A_by_B(ZMPwstPla,inverseWaistTransform,ZMPworPla);

    motionsample.configuration = gikSolver.currentConfiguration();
    motionsample.velocity = gikSolver.currentVelocity();
    motionsample.acceleration = gikSolver.currentAcceleration();
    motionsample.rootpose = gikSolver.solutionRootPose();
    motionsample.ZMPwstPla = ZMPwstPla;
    motionsample.ZMPwstObs = ZMPwstObs;
    motionsample.ZMPworPla = ZMPworPla;
    motionsample.ZMPworObs = ZMPworObs;
        
    solutionMotion.appendSample(motionsample);
    \endcode

            \code
}
\endcode
*/

