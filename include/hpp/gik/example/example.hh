/**
\page example1_page Example I
\section intro_sec Introduction
In this basic example, a motion for the center of the right hand is planned while projection of the center of mass on the ground and the feet's positions are maintained static.

        \section code_sec Code

                Set a sampling period: 5ms second is the value used on HRP2
 
                \code
                attSamplingPeriod = 5e-3;
        \endcode

Create the shape of the feet contact area as a polygon (here we give the example of a rectangle for each foot):
                \code
                ChppGik2DShape leftFootShape, rightFootShape;

        ChppGik2DVertex RupperRightCorner, RupperLeftCorner, RlowerLeftCorner, RlowerRightCorner;
        ChppGik2DVertex LupperRightCorner, LupperLeftCorner, LlowerLeftCorner, LlowerRightCorner;

    //fill in vertices (values for HRP-2 can be computed from info in CjrlFoot)
        ...

    //Add vertices to shapes
                leftFootShape.vertices.push_back ( LupperRightCorner );
        leftFootShape.vertices.push_back ( LupperLeftCorner );
        leftFootShape.vertices.push_back ( LlowerLeftCorner );
        leftFootShape.vertices.push_back ( LlowerRightCorner );

        rightFootShape.vertices.push_back ( RupperRightCorner );
        rightFootShape.vertices.push_back ( RupperLeftCorner );
        rightFootShape.vertices.push_back ( RlowerLeftCorner );
        rightFootShape.vertices.push_back ( RlowerRightCorner );
        \endcode

Supposing we have already constructed a CjrlHumanoidDynamicRobot object in halfsitting configuration and for which we have stored a pointer \c attRobot, create a standing robot:
                \code

                attStandingRobot = new ChppGikStandingRobot ( *attRobot, leftFootShape, rightFootShape );
        \endcode

                Create a Gik solver
                \code
                ChppGikSolver gikSolver(*attRobot);
        \endcode

                Select the root of the robot at the right foot (reduces computation complexity)
                \code
                gikSolver.rootJoint(*attRobot->rightAnkle());
        \endcode

                Create an empty motion (optional)
                \code
                ChppRobotMotion attSolutionMotion(attRobot, 0.0 , attSamplingPeriod);
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
        CjrlJoint& nsfJoint = *(attRobot->leftAnkle());
        matrix4d nsfTransform = nsfJoint.currentTransformation();
        ChppGikTransformationConstraint nsfc(*attRobot, nsfJoint, localPoint, nsfTransform);
        \endcode
    
                Second priority: static Center of Mass
                \code
                vector3d com = attRobot->positionCenterOfMass();
        ChppGikComConstraint comc(*attRobot, com[0], com[1]);
    
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
            attStandingRobot->updateRobot(gikSolver.solutionRootPose(), gikSolver.solutionJointConfiguration(), attSamplingPeriod);
    \endcode
            Store the computed sample (optional)
            \code
        //compute planned and observed ZMP in absolute and waist frames
            ZMPworObs = gikSolver.zeroMomentumPoint();
    waistTransform = attRobot->waist()->currentTransformation();
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
        
    attSolutionMotion.appendSample(motionsample);
    \endcode

            \code
}
\endcode
*/

