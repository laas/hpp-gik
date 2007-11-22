#include <stdio.h>
#include <string.h>
#include <iostream>
#include "hppGikTest.h"
#include "hppGikTools.h"
#include <time.h>
#include <sys/time.h>

using namespace std;


ChppGikTest::ChppGikTest() : attSamplingPeriod(5e-3)
{
    attLastRobotTask = 0;

    strcpy(attMotionName,"usermotion");

    createHumanoidRobot();

    attWholeBodyTask = new ChppGikWholeBodyTask(attStandingRobot,attSamplingPeriod);

    attHalfSittingTask = new ChppGikHalfSittingTask(attStandingRobot, attSamplingPeriod);

    attHandTask = new ChppGikHandTask(attStandingRobot,attSamplingPeriod);

    attStepBackTask = new ChppGikStepBackTask(attStandingRobot, attSamplingPeriod);

    attMotion = new ChppRobotMotion(attRobot, 0.0, attSamplingPeriod);

}

ChppGikTest::~ChppGikTest()
{
    delete attStandingRobot;
    delete attWholeBodyTask;
    delete attHalfSittingTask;
    delete attStepBackTask;
    delete attHandTask;
    delete attMotion;
}

void ChppGikTest::createHumanoidRobot()
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
    std::string name = "HRP2JRLmain.wrl.longneck.clean";
    aHDMB->parserVRML(path,name,"./HRP2LinkJointRank.xml");
    std::string aName="./HRP2Specificities.xml";
    aHDMB->SetHumanoidSpecificitiesFile(aName);

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

    /* apply initial static state*/
    attRobot->staticState(halfsittingConf);

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
}

void ChppGikTest::waist2worldPosition(vector3d& inWaistPosition, vector3d& outWorldPosition)
{
    matrix4d m;

    m = attRobot->rootJoint()->currentTransformation();
    outWorldPosition = m * inWaistPosition;
}

void ChppGikTest::leftFoot2worldPosition(vector3d& inLFootPosition, vector3d& outWorldPosition)
{
    matrix4d m = attRobot->leftFoot()->currentTransformation();
    MAL_S4x4_MATRIX_ACCESS_I_J(m,2,3) = MAL_S4x4_MATRIX_ACCESS_I_J(m,2,3) - attRobot->footHeight();
    outWorldPosition = m * inLFootPosition;
}

void ChppGikTest::dumpFilesAndGetLastConfig(ChppGikRobotTask* robotTask,const char* inFilename, vectorN& outLastConfig)
{
    //dump files
    robotTask->solutionMotion().dumpTo( inFilename );
    //update robot last config
    vectorN newConfiguration(attRobot->numberDof());

    double eT = robotTask->solutionMotion().endTime();
    bool assertion = robotTask->solutionMotion().configAtTime(eT, outLastConfig);

    if (!assertion)
        std::cout << "Mysterious debug: an unexpected problem happended !! abort now !!\n";

}

void  ChppGikTest::interprete()
{
    attModeWaist = true;

    bool do_quit = false, printit = true;
    char command[256];
    bool SideIsRight=true;
    string line;
    std::istringstream stream;

    //store half sitting configuration
    vectorN halfSittingConfig = attRobot->currentConfiguration();
    vectorN currentConfig = attRobot->currentConfiguration();
    vectorN resultConfig = currentConfig;
    vector3d position;
    vector3d orientation;

    string motion_file = "GIK_motion";

    while (!do_quit)
    {
        if (printit)
            printMenu();
        printit = true;

        getline(cin,line);
        stream.clear();
        stream.str(line);
        stream >> command;

        if (strcmp(command, "look")==0)
        {
            if (!read3coordinates(stream, position))
                continue;
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            lookat(position,motion_file.c_str(),currentConfig, resultConfig);
            continue;
        }

        if (strcmp(command, "hand")==0)
        {

            if (!determineSide(stream, SideIsRight))
                continue;
            if (!read3coordinates(stream, position))
                continue;

            bool doOrientation = !stream.eof();
            if (doOrientation)
                if (!read3coordinates(stream, orientation))
                    continue;

            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }

            handat(SideIsRight,doOrientation,position,orientation,motion_file.c_str(),currentConfig, resultConfig);
            continue;
        }

        if (strcmp(command, "lookhand")==0)
        {

            if (!determineSide(stream, SideIsRight))
                continue;
            if (!read3coordinates(stream, position))
                continue;

            bool doOrientation = !stream.eof();
            if (doOrientation)
                if (!read3coordinates(stream, orientation))
                    continue;

            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }

            lookhandat(SideIsRight,doOrientation,position,orientation,motion_file.c_str(),currentConfig, resultConfig);
            continue;
        }

        if (strcmp(command, "closehand")==0)
        {
            if (!determineSide(stream, SideIsRight))
                continue;
            if (stream.eof())
                continue;

            double valTighten;
            stream >> valTighten;

            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }

            handGrasp(SideIsRight, valTighten, motion_file.c_str(),currentConfig, resultConfig);
            continue;
        }

        if (strcmp(command, "openhand")==0)
        {
            if (!determineSide(stream, SideIsRight))
                continue;
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            double valTighten = 0.0;
            handGrasp(SideIsRight, valTighten,motion_file.c_str(),currentConfig, resultConfig);
            continue;
        }

        if (strcmp(command, "stepback")==0)
        {
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            stepback(motion_file.c_str(),currentConfig, resultConfig);
            continue;
        }

        if (strcmp(command, "halfsitting")==0)
        {
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            halfsitting(motion_file.c_str(),currentConfig, resultConfig);
            continue;
        }

        if (strcmp(command, "hrp")==0)
        {
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            string cmd;

            cmd = "scp " + motion_file + ".* hrp2014c:/home/okanoun/Demo16Avril2007/.";
            system(cmd.c_str());

            currentConfig = resultConfig;
        }

        if (strcmp(command, "apply")==0)
        {
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            currentConfig = resultConfig;
        }

        if (strcmp(command, "switch")==0)
        {
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            attModeWaist = !attModeWaist;
        }

        if (strcmp(command, "motionclear")==0)
        {
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            clearMotion();
        }

        if (strcmp(command, "motiondump")==0)
        {
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            dumpMotion();
        }

        if (strcmp(command, "motionkeep")==0)
        {
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            keepMotion();
        }

        if (strcmp(command, "motionname")==0)
        {
            setMotionName(stream);
        }


        if (strcmp(command, "help")==0)
        {
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            printHelp();
            printit = false;
            continue;
        }

        if (strcmp(command, "quit")==0)
        {
            if (!stream.eof())
            {
                std::cout << "check arguments\n";
                continue;
            }
            std::cout << "\n\n";
            break;
        }
    }
}



void ChppGikTest::lookat(vector3d& targetPoint, const char* filename, vectorN& curConfig, vectorN& resultConfig)
{
    attLastRobotTask = attWholeBodyTask;

    unsigned int priority;
    //the robot is static at the current configuration
    attRobot->staticState(curConfig);
    //Reset planner
    attWholeBodyTask->reset();

    CjrlGikGazeConstraint* gc = attGikFactory.createGazeConstraint(*attRobot, targetPoint);
    priority = 1;

    attWholeBodyTask->addStateConstraint(gc,priority);

    bool solved = attWholeBodyTask->solve();

    if(solved)
        dumpFilesAndGetLastConfig(attWholeBodyTask, filename, resultConfig);
    else
        std::cout << "\nNot solved.\n";

    delete gc;
}

void ChppGikTest::handat(bool taskIsForRightHand, bool doOrientation,vector3d& targetPoint, vector3d& targetOrientation,const char* filename, vectorN& curConfig, vectorN& resultConfig)
{
    attLastRobotTask = attWholeBodyTask;

    unsigned int priority;
    //the robot is static at the current configuration
    attRobot->staticState(curConfig);
    //Reset planner
    attWholeBodyTask->reset();

    CjrlJoint* joint = 0;
    CjrlHand* hand = 0;

    if (taskIsForRightHand)
    {
        joint = attRobot->rightWrist();
        hand = attRobot->rightHand();
    }
    else
    {
        joint = attRobot->leftWrist();
        hand = attRobot->leftHand();
    }

    vector3d lpoint,laxis;

    lpoint = hand->centerInWristFrame();
    laxis = hand->okayAxisInWristFrame();

    CjrlGikPositionConstraint* psc = 0;
    CjrlGikParallelConstraint* poc = 0;

    psc = attGikFactory.createPositionConstraint(*attRobot,*joint,lpoint,targetPoint);
    priority = 1;
    attWholeBodyTask->addStateConstraint(psc,priority);

    if (doOrientation)
    {
        poc = attGikFactory.createParallelConstraint(*attRobot,*joint,laxis,targetOrientation);
        priority = 1;
        attWholeBodyTask->addStateConstraint(poc,priority);
    }

    bool solved = attWholeBodyTask->solve();

    if(solved)
        dumpFilesAndGetLastConfig(attWholeBodyTask,filename, resultConfig);
    else
        std::cout<< "\nNot solved.\n";

    delete psc,poc;

}

void ChppGikTest::lookhandat(bool taskIsForRightHand, bool doOrientation,vector3d& targetPoint, vector3d& targetOrientation,const char* filename, vectorN& curConfig, vectorN& resultConfig)
{
    attLastRobotTask = attWholeBodyTask;

    unsigned int priority;
    //the robot is static at the current configuration
    attRobot->staticState(curConfig);
    //Reset planner
    attWholeBodyTask->reset();

    CjrlJoint* joint = 0;
    CjrlHand* hand = 0;

    if (taskIsForRightHand)
    {
        joint = attRobot->rightWrist();
        hand = attRobot->rightHand();
    }
    else
    {
        joint = attRobot->leftWrist();
        hand = attRobot->leftHand();
    }

    vector3d lpoint,laxis;

    lpoint = hand->centerInWristFrame();
    laxis = hand->okayAxisInWristFrame();

    CjrlGikPositionConstraint* psc = 0;
    CjrlGikParallelConstraint* poc = 0;

    psc = attGikFactory.createPositionConstraint(*attRobot,*joint,lpoint,targetPoint);
    priority = 1;
    attWholeBodyTask->addStateConstraint(psc,priority);

    if (doOrientation)
    {
        poc = attGikFactory.createParallelConstraint(*attRobot,*joint,laxis,targetOrientation);
        priority = 1;
        attWholeBodyTask->addStateConstraint(poc,priority);
    }

    //add gaze constraint: look at the hand target
    CjrlGikGazeConstraint* gc = attGikFactory.createGazeConstraint(*attRobot, targetPoint);
    priority = 2;
    attWholeBodyTask->addStateConstraint(gc,priority);

    bool solved = attWholeBodyTask->solve();

    if(solved)
        dumpFilesAndGetLastConfig(attWholeBodyTask,filename, resultConfig);
    else
        std::cout<< "\nNot solved.\n";

    delete psc,poc,gc;

}


void ChppGikTest::handGrasp(bool taskIsForRightHand, double valTighten, const char* filename, vectorN& curConfig, vectorN& resultConfig)
{
    attLastRobotTask = attHandTask;

    //the robot is static at the current configuration
    attRobot->staticState(curConfig);

    attHandTask->forRightHand(taskIsForRightHand);
    attHandTask->targetClench(valTighten);

    bool solved = attHandTask->solve();

    if(solved)
        dumpFilesAndGetLastConfig(attHandTask, filename, resultConfig);
    else
        std::cout <<"\nNot solved.\n";
}


void ChppGikTest::stepback(const char* filename, vectorN& curConfig, vectorN& resultConfig)
{
    attLastRobotTask = attStepBackTask;

    //the robot is static at the current configuration
    attRobot->staticState(curConfig);

    bool solved = attStepBackTask->solve();

    if (solved)
    {
        if (!attStepBackTask->solutionMotion().empty())
            dumpFilesAndGetLastConfig(attStepBackTask,filename, resultConfig);
    }
    else
        std::cout <<"\nNot solved.\n";
}

void ChppGikTest::halfsitting(const char* filename, vectorN& curConfig, vectorN& resultConfig)
{
    attLastRobotTask = attHalfSittingTask;

    //the robot is static at the current configuration
    attRobot->staticState(curConfig);

    bool solved = attHalfSittingTask->solve();

    if (solved)
        dumpFilesAndGetLastConfig(attHalfSittingTask, filename, resultConfig);
    else
        std::cout <<"\nNot solved.\n";
}

void  ChppGikTest::printMenu()
{
    std::string message;


    std::cout << "\n------------------------------\n";

    if (attModeWaist)
        std::cout <<  "-- Coordinates in: WAIST -----\n";
    else
        std::cout <<  "-- Coordinates in: LEFTFOOT --\n";


    std::cout << "------------------------------\n";
    std::cout << "-- Commands -- \n";
    std::cout << "------------------------------\n";
    std::cout << "- look x y z\n";
    std::cout << "- hand r/l x y z [vx vy vz]\n";
    std::cout << "- lookhand r/l x y z [vx vy vz]\n";
    std::cout << "- closehand r/l T\n";
    std::cout << "- openhand r/l\n";
    std::cout << "- stepback\n";
    std::cout << "- halfsitting\n";
    std::cout << "------------------------------\n";
    std::cout << "- hrp\n";
    std::cout << "- apply\n";
    std::cout << "- switch\n";
    std::cout << "------------------------------\n";
    std::cout << "- motionname filename\n";
    std::cout << "- motionclear\n";
    std::cout << "- motionkeep\n";
    std::cout << "- motiondump\n";
    std::cout << "------------------------------\n";
    std::cout << "- help\n";
    std::cout << "- quit\n";
    std::cout << "------------------------------\n";
    std::cout << "\n";
}

void  ChppGikTest::printHelp()
{
    std::cout << "\n";
    std::cout << "-- Help --\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"look x y z\" to look at the given point\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"hand r(l) x y z vx vy vz\" to: \nbring the right(left) hand center at point [x y z]\nand(optional) make hand axis (orthogonal to the forearm) parallel to [vx vy vz]\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"lookhand r(l) x y z vx vy vz\" to combine lookat and handat\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"closehand r(l) T\" to to close the hand. T determines how tight the hand is closed, and it goes from 0 (open) to 1 (closed)\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"openhand r(l)\" to open the selected hand\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"stepback\" to make the robot step back using the indicated foot\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"halfsitting\" to go to half sitting stance\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"apply\" to make last computed configuration the current\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"hrp\" to send motion to HRP2 and make last computed configuration the current\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"motionname filename\" to specifiy the filename used to dump the set of computed motions. Keep the name short. Default filename is \"usermotion\"\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"motionclear\" to clear the so-far accumulated motion.\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"motionkeep\" to append the last computed motion to the so-far accumulated motion.\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"motiondump\" to dump the accumulated motion to a set of files compatible with OpenHRP.\n";
    std::cout << "-------------------\n";
    std::cout << "- use \"switch\" to change input coordinates frame (Waist or LeftFoot)\n";
    std::cout << "-------------------\n";
    std::cout << "\n";
}

bool ChppGikTest::read3coordinates(std::istringstream& stream, vector3d& coordinates)
{
    vector3d inputVec;
    //read target point in waist frame
    for (unsigned int i=0; i<3;i++)
    {
        if (!stream.eof())
            stream >> inputVec[i];
        else
        {
            std::cout << "Incorrect number of arguments\n";
            return false;
        }
    }

    if (attModeWaist)
        waist2worldPosition(inputVec, coordinates);
    else
        leftFoot2worldPosition(inputVec, coordinates);

    return true;
}


bool ChppGikTest::determineSide(std::istringstream& stream, bool& isRight)
{
    char whichside[16];
    if (!stream.eof())
        stream >> whichside;
    else
    {
        std::cout << "Incorrect number of arguments\n";
        return false;
    }

    if (strcmp(whichside, "r")==0)
        isRight =  true;
    else if (strcmp(whichside, "l")==0)
        isRight = false;
    else
        return false;
    return true;
}

void ChppGikTest::clearMotion()
{
    attMotion->clear();
}

void ChppGikTest::keepMotion()
{
    if (attLastRobotTask)
    {
        attMotion->appendMotion(attLastRobotTask->solutionMotion());
        attLastRobotTask = 0;
    }
}

void ChppGikTest::dumpMotion()
{
    attMotion->dumpTo(attMotionName);
}

void ChppGikTest::setMotionName(std::istringstream& stream)
{
    if (!stream.eof())
        stream >> attMotionName;
    else
    {
        std::cout << "Specify a name for the result motion\n";
        return;
    }
}

void ChppGikTest::goDownChain(const CjrlJoint* startJoint)
{
    std::cout << "joint ranked " << startJoint->rankInConfiguration() << std::endl;
    std::cout << startJoint->currentTransformation() << std::endl;

    if (startJoint->countChildJoints() != 0)
    {
        const CjrlJoint* childJoint = startJoint->childJoint(0);
        goDownChain(childJoint);
    }
}

void ChppGikTest::goDownTree(const CjrlJoint* startJoint)
{
    std::cout << "joint ranked " << startJoint->rankInConfiguration() << std::endl;
    std::cout << startJoint->currentTransformation() << std::endl;

    for (unsigned int i = 0; i<startJoint->countChildJoints(); i++)
    {
        const CjrlJoint* childJoint = startJoint->childJoint(i);
        goDownTree(childJoint);
    }
}



void ChppGikTest::basicExample()
{
    //We have a robot at some initial configuration

    //all coordinates are in world frame (when the robot is in half sitting configuration, the world frame is located at the projection of the waist center on the ground, x axis pointing forwards, z axis upwards)

    //In this example, target points and transformation are fed to the constraint constructors. They are equivalent to velocities when we knoe the sampling period of the problem.

    //Note that the following constrants are being created and stacked in decreasing priority

    
    //Select the support foot
    CjrlJoint* fixedFoot = attRobot->rightFoot(); //arbitrary
    //Record the fixed foot in the robot object:
    attRobot->clearFixedJoints();
    attRobot->addFixedJoint( fixedFoot );
    
    //Create a Gik solver
    ChppGikSolver* gikSolver = new ChppGikSolver(attRobot);

    //Create an empty stack of constraints
    std::vector<CjrlGikStateConstraint*> stack;

    //some variables
    vector3d targetPoint, localPoint;

    //-- Stability-related constraints --//
    //-----------------------------------//


    //create the non support foot constraint
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->leftFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    CjrlGikStateConstraint* nsfc = attGikFactory.createTransformationConstraint(*attRobot, nsfJoint, localPoint, nsfTransform);

    stack.push_back(nsfc);

    //create a CoM constraint
    vector3d com = attRobot->positionCenterOfMass();
    CjrlGikStateConstraint* comc = attGikFactory.createComConstraint(*attRobot, com[0], com[1]);

    stack.push_back(comc);


    //-- User constraints --//
    //----------------------//
    //create a position constraint on a localPoint in the right wrist
    matrix4d curT=  attRobot->rightWrist()->currentTransformation();
    targetPoint[0] = 0.5;
    targetPoint[1] = -0.3;
    targetPoint[2] = 1.0;
    CjrlJoint& rwJoint = *(attRobot->rightWrist());
    localPoint = attRobot->rightHand()->centerInWristFrame();
    CjrlGikStateConstraint* pc = attGikFactory.createPositionConstraint(*attRobot,rwJoint,localPoint, curT*localPoint);
       
    
    stack.push_back(pc);

    //create a gaze constraint
    targetPoint[0] = 1;
    targetPoint[1] = -1;
    targetPoint[2] = 0.0;
    CjrlGikStateConstraint* gc = attGikFactory.createGazeConstraint(*attRobot, targetPoint);

    //stack.push_back(gc);

    //-- Do one solving step --//
    //-------------------------//
            //Set the weights used in solving the inverse kinematics problem
        //GikSolver treats all joints equally by default. Toy with these weights for realism.
    vectorN activated =  attStandingRobot->maskFactory()->wholeBodyMask();
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    vectorN combined = weights;

    for (unsigned int i=0;i<combined.size();i++)
        combined(i) *= activated(i);
    gikSolver->weights(combined);

    
    vector3d p;
    ChppRobotMotion attSolutionMotion(attRobot, 0.0 , attSamplingPeriod);
    
    struct timeval *Tps, *Tpf;
    struct timezone *Tzp;
    Tps = (struct timeval*) malloc(sizeof(struct timeval));
    Tpf = (struct timeval*) malloc(sizeof(struct timeval));
    Tzp = 0;
    gettimeofday (Tps, Tzp);
    
    for (unsigned int j = 0; j< 500;j++)
    {
        p = ((CjrlGikPositionConstraint*)pc)->worldTarget();
        p[0] += 0.001;
        ((CjrlGikPositionConstraint*)pc)->worldTarget(p);

        //compute the support foot jacobian (this is done to avoid computing this jacobian several times as it is needed to computes the jacobians of every constraint)
        fixedFoot->computeJacobianJointWrtConfig();

        //compute constraints jacobians and values
        for (unsigned int i = 0; i< stack.size();i++)
        {
            stack[i]->computeValue();
            stack[i]->computeJacobian();
        }

        gikSolver->weights(combined);

        //Account for joint limits: modifies the weights according to current configuration
        gikSolver->accountForJointLimits();

        //1-step Solve (might not fulfill the desired constraints)
        bool ok = gikSolver->gradientStep( stack ); //There is a more singularity-robust but less accurate method with one more argument (see GikSolver.h)

        //-- Retrieve solution --//
        //-----------------------//
        vectorN solutionConfig = attRobot->currentConfiguration();

        //store
        attRobot->FiniteDifferenceStateUpdate(attSamplingPeriod);
        attSolutionMotion.appendSample(solutionConfig,p,p,p,p);
    }

    gettimeofday (Tpf, Tzp);
    printf("Basic example solved in: %ld ms\n", (Tpf->tv_sec-Tps->tv_sec)*1000 + (Tpf->tv_usec-Tps->tv_usec)/1000);
    free(Tps);
    free(Tpf);
    
    attSolutionMotion.dumpTo( "test" );

    //clean the mess
    delete gikSolver;
    for (unsigned int i = 0; i<stack.size();i++)
        delete stack[i];
}
