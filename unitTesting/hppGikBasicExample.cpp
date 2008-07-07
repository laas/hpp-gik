#include "robot/hppRobotMotion.h"
#include "core/hppGikSolverHumanoid.h"
#include "hppGikBasicExample.h"


ChppGikBasicExample::ChppGikBasicExample()
{
    //Set a sampling period: 5e-3 second is the value used on HRP2
    attSamplingPeriod = 5e-3;
    //Create a humanoid robot and a standing robot
    createStandingRobot();
}


ChppGikBasicExample::~ChppGikBasicExample()
{
    delete attStandingRobot;
}


void ChppGikBasicExample::planExample()
{
    /*
    In this basic motion planning example, we :
    - create (1) a TransformationConstraint on the non-support foot
             (2) a Center of Mass constraint
             (3) a position constraint on the right hand
    - create a GikSolver
    - Solve these 3 constraints several times while the target for the hand position constraint is being slightly changed. This yields to a motion.
    */


    //set a support foot on the robot
    CjrlJoint* fixedFoot = attRobot->rightFoot(); //arbitrary
    attRobot->clearFixedJoints();
    attRobot->addFixedJoint( fixedFoot );

    //Create a Gik solver
    ChppGikSolverHumanoid gikSolverHumanoid(*attRobot);
    //Create a CppRobotMotion (where the successive configurations will be stored)
    ChppRobotMotion attSolutionMotion(attRobot, 0.0 , attSamplingPeriod);

    //some used variables
    vector3d targetPoint, localPoint, p;

    //-- Create the constraints ---------//
    //-----------------------------------//
    //Create an empty stack of constraints
    std::vector<CjrlGikStateConstraint*> stack;
    //create the non support foot constraint
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->leftFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    CjrlGikTransformationConstraint* nsfc = attGikFactory.createTransformationConstraint(*attRobot, nsfJoint, localPoint, nsfTransform);
    stack.push_back(nsfc);
    //create a CoM constraint
    vector3d com = attRobot->positionCenterOfMass();
    CjrlGikComConstraint* comc = attGikFactory.createComConstraint(*attRobot, com[0], com[1]);
    stack.push_back(comc);
    //create a position constraint on a localPoint in the right wrist
    matrix4d curT=  attRobot->rightWrist()->currentTransformation();
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = -0.18;
    CjrlJoint& rwJoint = *(attRobot->rightWrist());
    localPoint = attRobot->rightHand()->centerInWristFrame();
    CjrlGikPositionConstraint* pc = attGikFactory.createPositionConstraint(*attRobot,rwJoint,localPoint, curT*localPoint);
    stack.push_back(pc);

    //-- Solve -----------------------//
    //--------------------------------//
    //Build the weights used in solving the inverse kinematics problem
    vectorN activated =  attStandingRobot->maskFactory()->wholeBodyMask();// active joints
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport(); //With these weights, the more mass a joint is lifting the less it's used
    vectorN combined = weights;
    for (unsigned int i=0;i<combined.size();i++)
        combined(i) *= activated(i);

    vector3d absZMPPla, absZMPObs, relZMPObs, relZMPPla;
    absZMPPla = com;

    //Start the loop where we change the desired target position of the hand, thus generating a motion
    for (unsigned int j = 0; j< 500;j++)
    {
        //Slight change of hand task
        {
            //get the current target position for the hand constraint
            p = pc->worldTarget();
            //change in x coordinate
            p[0] += 0.001;
            //set the new target
            pc->worldTarget(p);
        }

        //Solve: here we do a single gradient descent step because the solution configurations are close to current configuration
        {
            //Prepare the solver and the constraints
            {
                //Set the weights in the solver
                gikSolverHumanoid.weights(combined);
                //compute the support foot jacobian
                fixedFoot->computeJacobianJointWrtConfig();
                //prepare constraints by computing jacobians and values
                for (unsigned int i = 0; i< stack.size();i++)
                {
                    stack[i]->computeValue();
                    stack[i]->computeJacobian();
                }
            }

            //Solve. The solution configuration is directly applied on the robot
            gikSolverHumanoid.solve( stack );
            gikSolverHumanoid.applySolution();
        }

        //Update robot dynamics based on new and past configurations
        attStandingRobot->updateDynamics(attSamplingPeriod, absZMPPla, absZMPObs, relZMPObs, relZMPPla);
        //Get the new configuration of the robot
        const vectorN& solutionConfig = attRobot->currentConfiguration();
        //store the solution configuration in the ChppRobotMotion object.
        attSolutionMotion.appendSample(solutionConfig,absZMPPla, absZMPObs, relZMPPla, relZMPObs);
    }

    //Dump computed motion in a set of files readable by OpenHRP
    attSolutionMotion.dumpTo( "BasicExampleOutput" );

    //Destroy created constraints
    for (unsigned int i = 0; i<stack.size();i++)
        delete stack[i];
}




void ChppGikBasicExample::createStandingRobot()
{
    CjrlRobotDynamicsObjectConstructor<
    dynamicsJRLJapan::DynamicMultiBody,
    dynamicsJRLJapan::HumanoidDynamicMultiBody,
    dynamicsJRLJapan::JointFreeflyer,
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> jrlRobotFactory;

    attRobot = jrlRobotFactory.createhumanoidDynamicRobot();

    dynamicsJRLJapan::HumanoidDynamicMultiBody *aHDMB;
    aHDMB = (dynamicsJRLJapan::HumanoidDynamicMultiBody*) attRobot;

    std::string path = "./";
    std::string name = "HRP2.wrl";
    aHDMB->parserVRML(path,name,"./HRP2LinkJointRank.xml");
    std::string aName="./HRP2Specificities.xml";
    aHDMB->SetHumanoidSpecificitiesFile(aName);
    aHDMB->SetTimeStep(attSamplingPeriod);
    aHDMB->setComputeAcceleration(true);
    aHDMB->setComputeBackwardDynamics(false);
    aHDMB->setComputeZMP(true);


    unsigned int nDof = attRobot->numberDof();
    vectorN halfsittingConf(nDof);

    //Half sitting
    double dInitPos[40] =
        {
            0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // right leg

            0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // left leg

            0.0, 0.0, // chest

            0.0, 0.0, // head

            15.0, -10.0, 0.0, -30.0, 0.0, 0.0, // right arm

            10.0, // right hand clench

            15.0,  10.0, 0.0, -30.0, 0.0, 0.0, // left arm

            10.0, // left hand clench

            -10.0, 10.0, -10.0, 10.0, -10.0, // right hand parallel mechanism
            -10.0, 10.0, -10.0, 10.0, -10.0  // left hand parallel mechanism
        };

    //waist x y z
    halfsittingConf(0) = 0.0;
    halfsittingConf(1) = 0.0;
    halfsittingConf(2) = 0.6487;
    //waist roll pitch yaw
    halfsittingConf(3) = 0.0;
    halfsittingConf(4) = 0.0;
    halfsittingConf(5) = 0.0;

    //joints
    for(unsigned int i=6;i<nDof;i++)
        halfsittingConf(i) = dInitPos[i-6]*M_PI/180;

    ublas::zero_vector<double> zeros(attRobot->numberDof());
    attRobot->currentConfiguration(halfsittingConf);
    attRobot->currentVelocity(zeros);
    attRobot->currentAcceleration(zeros);
    attRobot->computeForwardKinematics();

    //set gaze origin and direction
    vector3d gazeDir,gazeOrigin;
    gazeDir[0] = 1;
    gazeDir[1] = 0;
    gazeDir[2] = 0;

    gazeOrigin[0] = 0;
    gazeOrigin[1] = 0;
    gazeOrigin[2] = 0.118;

    aHDMB->gaze((const vector3d&)gazeDir,(const vector3d&)gazeOrigin);

    attStandingRobot = new ChppGikStandingRobot(attRobot);
    attStandingRobot->staticState(halfsittingConf);
}
