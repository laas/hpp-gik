/*
 * Copyright (c) 2008
 *      IS/AIST-ST2I/CNRS Joint Japanese-French Robotics Laboratory (JRL).
 * All rights reserved.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <time.h>
#include <sys/time.h>

#if defined(HAVE_X11_XLIB_H) && defined(HAVE_X11_XUTIL_H)
# include <X11/Xlib.h>
# include <X11/Xutil.h>
#endif

#include "hppGikTest.h"
#include "hppGikTools.h"

#include "motionplanners/hppGikStepPlanner.h"

#define M3_IJ MAL_S3x3_MATRIX_ACCESS_I_J
#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J

//#include "tasks/hppGikCmpTask.h"
using namespace std;


ChppGikTest::ChppGikTest() : attSamplingPeriod(5e-3)
{

    attLastRobotTask = 0;

    strcpy(attMotionName,"usermotion");

    createHumanoidRobot();

    attWholeBodyTask = new ChppGikWholeBodyTask(attStandingRobot,attSamplingPeriod);
    
    attWholeBodyTask->showResolutionTime( true );

    attHalfSittingTask = new ChppGikHalfSittingTask(attStandingRobot, attSamplingPeriod);
    
    attHalfSittingTask->showResolutionTime( true );
    
    attHandTask = new ChppGikHandTask(attStandingRobot,attSamplingPeriod);

    attStepBackTask = new ChppGikStepBackTask(attStandingRobot, attSamplingPeriod);

    attMotion = new ChppRobotMotion(attRobot, 0.0, attSamplingPeriod);

    attViewerHealthy = false;
    attViewer = new ChppGikViewer(attViewerHealthy);

}

ChppGikTest::~ChppGikTest()
{
    delete attStandingRobot;
    delete attWholeBodyTask;
    delete attHalfSittingTask;
    delete attStepBackTask;
    delete attHandTask;
    delete attMotion;
    delete attViewer;
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
    attStandingRobot->staticState(curConfig);
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
    attStandingRobot->staticState(curConfig);
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

    delete psc;
    delete poc;

}

void ChppGikTest::lookhandat(bool taskIsForRightHand, bool doOrientation,vector3d& targetPoint, vector3d& targetOrientation,const char* filename, vectorN& curConfig, vectorN& resultConfig)
{
    attLastRobotTask = attWholeBodyTask;

    unsigned int priority;
    //the robot is static at the current configuration
    attStandingRobot->staticState(curConfig);
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

    delete psc;
    delete poc;
    delete gc;

}


void ChppGikTest::handGrasp(bool taskIsForRightHand, double valTighten, const char* filename, vectorN& curConfig, vectorN& resultConfig)
{
    attLastRobotTask = attHandTask;

    //the robot is static at the current configuration
    attStandingRobot->staticState(curConfig);

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
    attStandingRobot->staticState(curConfig);

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
    attStandingRobot->staticState(curConfig);

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



void ChppGikTest::locoPlannerTest()
{

    ChppGikStepPlanner lplanner(attStandingRobot);
    std::vector<ChppGikVectorizableConstraint*> constraints;
    vector3d targetPoint, localPoint, normalVec;

    ///*
    //create the non support foot constraint
    targetPoint[0] = -2;
    targetPoint[1] = -1;
    targetPoint[2] = 0.9;
    ChppGikGazeConstraint gc( *(attStandingRobot->robot()), targetPoint);
    constraints.push_back(&gc);
    //*/

    /*
    localPoint = attStandingRobot->robot()->leftHand()->centerInWristFrame();
    targetPoint[0] = 1;
    targetPoint[1] = 2;
    targetPoint[2] = 0.9;
    ChppGikPositionConstraint posc2( *(attStandingRobot->robot()), *(attStandingRobot->robot()->leftWrist()), localPoint, targetPoint);
    constraints.push_back(&posc2);
    */

    //ChppGikParallelConstraint wvert(*attRobot, *(attRobot->waist()), normalVec, normalVec);
    //constraints.push_back(&wvert);

    /*
    //create the non support foot constraint
    localPoint[0] =localPoint[1] = localPoint[2] = 0.0;
    targetPoint[0] = 1.7;
    targetPoint[1] = 1.0;
    targetPoint[2] = 0.7;
    ChppGikPositionConstraint posc( *(attStandingRobot->robot()), *(attStandingRobot->robot()->waist()), localPoint, targetPoint);
    constraints.push_back(&posc);
    */



    /*
        //-- gaze at [-1 1 1.7] --//
        //-----------------------------------//
        vector3d lp, pp, pn;
        lp[0] = 0;
        lp[1] = 0;
        lp[2] = 0;
        pp = lp;
        pn = pp;
        pn[0] = 1;
        ChppGikPlaneConstraint gazeplanelat(*attRobot, *(attRobot->gazeJoint()), lp, pp, pn);
        pn[0] = 0;
        pn[1] = 1;
        ChppGikPlaneConstraint gazeplanesag(*attRobot, *(attRobot->gazeJoint()), lp, pp, pn);
     
     
            
    //     constraints.push_back(&gazeplanelat);
    //     constraints.push_back(&gazeplanesag);
        */

    lplanner.constraints( constraints );
    lplanner.bigSolve( 40 );
    lplanner.dumpBigSolutionTo( "ankleprints" );

}

#if defined(HAVE_X11_XLIB_H) && defined(HAVE_X11_XUTIL_H)
void ChppGikTest::locoPlannerTestInteractive()
{
    /* perform an events loop */

    int done = 0;
    double Xincr = 0.1;
    double Yincr = 0.1;
    double Tincr = 0.1;

    vector3d waistTarget, localPoint, localVector, waistPointer, gazeTarget;
    waistTarget[0] = 0.0;
    waistTarget[1] = 0.0;
    waistTarget[2] = 0.6487;
    waistPointer[0] = 1;
    waistPointer[1] = 0;
    waistPointer[2] = 0.6487;
    localVector[0] = 1;
    localVector[1] = 0;
    localVector[2] = 0;
    gazeTarget[0] = 0.0;
    gazeTarget[1] = 0.0;
    gazeTarget[2] = 1.4;

    double curTheta;

    /**the constraint*/
    ChppGikStepPlanner lplanner(attStandingRobot);
    std::vector<ChppGikVectorizableConstraint*> constraints;

    localPoint[0] =localPoint[1] = localPoint[2] = 0.0;
    ChppGikPositionConstraint posw( *(attStandingRobot->robot()), *(attStandingRobot->robot()->waist()), localPoint, waistTarget);
    constraints.push_back(&posw);
    matrix3d rot;
    for (unsigned int i=0;i<3;i++)
        for (unsigned int j=0;j<3;j++)
            M3_IJ(rot,i,j) = 0.0;
    M3_IJ(rot,2,2) = 1;
    ChppGikRotationConstraint oriow( *(attStandingRobot->robot()), *(attStandingRobot->robot()->waist()), rot);

    ChppGikGazeConstraint gazec( *(attStandingRobot->robot()), gazeTarget);


    if (!attViewerHealthy)
        return;

    attViewer->showframe();
    draw2DRobot();
    attViewer->flush();
    usleep(100);

    while (!done)
    {
        KeySym* key = attViewer->nextKey();
        //std::cout << "here \n";
        if (key)
        {

            switch (*key)
            {
            case XK_c:
                constraints.clear();
                lplanner.constraints( constraints );
                break;
            case XK_q:
                done = 1;
                break;

            case XK_p:
                std::cout << attRobot->currentConfiguration() << "\n";
                break;

            case XK_Up:
                waistTarget[0] = attRobot->currentConfiguration()(0) + Xincr;
                waistTarget[1] = attRobot->currentConfiguration()(1);
                posw.worldTarget(waistTarget);
                //std::cout << "Up\n";
                constraints.clear();
                constraints.push_back(&posw);
                lplanner.constraints( constraints );
                break;

            case XK_Left:
                waistTarget[0] = attRobot->currentConfiguration()(0);
                waistTarget[1] = attRobot->currentConfiguration()(1) + Yincr;
                posw.worldTarget(waistTarget);
                //std::cout << "Left\n";
                constraints.clear();
                constraints.push_back(&posw);
                lplanner.constraints( constraints );
                break;

            case XK_Right:
                waistTarget[0] = attRobot->currentConfiguration()(0);
                waistTarget[1] = attRobot->currentConfiguration()(1) - Yincr;
                posw.worldTarget(waistTarget);
                //std::cout << "Right\n";
                constraints.clear();
                constraints.push_back(&posw);
                lplanner.constraints( constraints );
                break;

            case XK_Down:
                waistTarget[0] = attRobot->currentConfiguration()(0) - Xincr;
                waistTarget[1] = attRobot->currentConfiguration()(1);
                posw.worldTarget(waistTarget);
                //std::cout << "Down\n";
                constraints.clear();
                constraints.push_back(&posw);
                lplanner.constraints( constraints );
                break;

            case XK_s:
                curTheta = attRobot->currentConfiguration()(5) + Tincr;
                M3_IJ(rot,0,0) = M3_IJ(rot,1,1) = cos(curTheta);
                M3_IJ(rot,1,0) = sin(curTheta);
                M3_IJ(rot,0,1) = -M3_IJ(rot,0,1);
                oriow.targetOrientation( rot);
                //std::cout << "Turn left\n";
                constraints.clear();
                constraints.push_back(&oriow);
                lplanner.constraints( constraints );
                break;

            case XK_d:
                curTheta = attRobot->currentConfiguration()(5) - Tincr;
                M3_IJ(rot,0,0) = M3_IJ(rot,1,1) = cos(curTheta);
                M3_IJ(rot,1,0) = sin(curTheta);
                M3_IJ(rot,0,1) = -M3_IJ(rot,0,1);
                oriow.targetOrientation( rot);
                //std::cout << "Turn right\n";
                constraints.clear();
                constraints.push_back(&oriow);
                lplanner.constraints( constraints );
                break;


            case XK_e:
                curTheta = attRobot->currentConfiguration()(5) - Tincr;
                gazeTarget[0] = attRobot->currentConfiguration()(0) + cos(curTheta);
                gazeTarget[1] = attRobot->currentConfiguration()(1) - sin(curTheta);
                gazec.worldTarget( gazeTarget );
                //std::cout << "look right\n";
                constraints.clear();
                constraints.push_back(&gazec);
                lplanner.constraints( constraints );
                break;

            case XK_w:
                curTheta = attRobot->currentConfiguration()(5) + Tincr;
                gazeTarget[0] = attRobot->currentConfiguration()(0) + cos(curTheta);
                gazeTarget[1] = attRobot->currentConfiguration()(1) - sin(curTheta);
                gazec.worldTarget( gazeTarget );
                //std::cout << "look right\n";
                constraints.clear();
                constraints.push_back(&gazec);
                lplanner.constraints( constraints );
                break;

            default:
                break;
            }
        }


        while (lplanner.littleSolve())
        {
            attViewer->erase();
            draw2DRobot();
            attViewer->flush();
            //             std::cout << "After little solve:\n";
            //             attStandingRobot->supportPolygon()->print();
        }
    }
}
#endif /* defined(HAVE_X11_XLIB_H) && defined(HAVE_X11_XUTIL_H) */

void ChppGikTest::draw2DRobot()
{
    ChppGikSupportPolygon* curSup = attStandingRobot->supportPolygon();

    attViewer->draw2DShape(attStandingRobot->waistShape(), attRobot->currentConfiguration()(0),attRobot->currentConfiguration()(1),attRobot->currentConfiguration()(5));

    attViewer->draw2DShape(attStandingRobot->rightFootShape(), curSup->rightFootprint()->x(),curSup->rightFootprint()->y(),curSup->rightFootprint()->th());

    attViewer->draw2DShape(attStandingRobot->leftFootShape(), curSup->leftFootprint()->x(),curSup->leftFootprint()->y(), curSup->leftFootprint()->th());

}


