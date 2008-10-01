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

#include "hppGikTest.h"
#include "hppGikTools.h"
#include "hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h"

#define M3_IJ MAL_S3x3_MATRIX_ACCESS_I_J
#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J
#define V3_I  MAL_S3_VECTOR_ACCESS

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

void ChppGikTest::accel()
{
    /**debug*/
    
    CjrlJoint* fixedFoot = attRobot->leftFoot();
    attRobot->clearFixedJoints();
    attRobot->addFixedJoint( fixedFoot );
    ChppGikSolverRobotAttached gikSolver(*attRobot);
    
    vector3d targetPoint, localPoint, p;
    vector3d absZMPPla, absZMPObs, relZMPObs, relZMPPla;
    vector3d normalVec;

    std::vector<CjrlGikStateConstraint*> stack;
    
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->rightFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    matrix4d sfTransform = fixedFoot->currentTransformation();
    ChppGikTransformationConstraint nsfc(*attRobot, nsfJoint, localPoint, nsfTransform);
    stack.push_back(&nsfc);
    
    
    matrix4d m;
    M4_IJ(m,0,3) = 0.0001;
    M4_IJ(m,1,3) = 0;
    M4_IJ(m,2,3) = 0.6487;
    localPoint[0] = localPoint[1] = localPoint[2] = 0;
    ChppGikTransformationConstraint trans (*(attStandingRobot->robot()), *(attStandingRobot->robot()->waist()), localPoint, m);
    stack.push_back(&trans);
    
    
    vectorN activated =  attStandingRobot->maskFactory()->legsMask();
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    vectorN combined = weights;
    for (unsigned int i=0;i<combined.size();i++)
        combined(i) *= activated(i);
    ChppGikSupportPolygon* curSupportPolygon;
    gikSolver.weights(combined);
    
    fixedFoot->computeJacobianJointWrtConfig();
    for (unsigned int i = 0; i< stack.size();i++)
    {
        stack[i]->computeValue();
        stack[i]->computeJacobian();
    }
    gikSolver.solve( stack );
    gikSolver.applySolution();

    attStandingRobot->updateDynamics(attSamplingPeriod, absZMPPla, absZMPObs, relZMPObs, relZMPPla);
    
    std::cout << absZMPObs << "\n";
    std::cout << attRobot->currentConfiguration() << "\n";
    
}

void ChppGikTest::springTest()
{
    CjrlJoint* fixedFoot = attRobot->leftFoot();
    attRobot->clearFixedJoints();
    attRobot->addFixedJoint( fixedFoot );
    ChppGikSolverRobotAttached gikSolver(*attRobot);

    ChppRobotMotion motion(attRobot, 0.0 , attSamplingPeriod);
    vector3d targetPoint, localPoint, p;
    vector3d absZMPPla, absZMPObs, relZMPObs, relZMPPla;

    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->rightFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    matrix4d sfTransform = fixedFoot->currentTransformation();
    ChppGikTransformationConstraint nsfc(*attRobot, nsfJoint, localPoint, nsfTransform);

    vector3d com = attRobot->positionCenterOfMass();

    double D = 3.0;
    unsigned int n1 = ChppGikTools::timetoRank(0,D,attSamplingPeriod) +1;
    unsigned int n2 = ChppGikTools::timetoRank(0,2*D,attSamplingPeriod) +1;
    matrixNxP comData1(2,n1);
    matrixNxP comData2(2,n2);
    matrixNxP comDataAll(2,2*n1+n2);
    vectorN interpolationdata1(n1);
    vectorN interpolationdata2(n2);

    ChppGikTools::minJerkCurve( D, attSamplingPeriod, com[0], 0, 0, M4_IJ(sfTransform,0,3),interpolationdata1 );
    row(comData1,0) = interpolationdata1;
    ChppGikTools::minJerkCurve( D, attSamplingPeriod, com[1], 0, 0, M4_IJ(sfTransform,1,3),interpolationdata1 );
    row(comData1,1) = interpolationdata1;
    subrange(comDataAll,0,2,0,n1) = comData1;
    
    ChppGikTools::minJerkCurve( 2*D, attSamplingPeriod, M4_IJ(sfTransform,0,3), 0, 0, M4_IJ(nsfTransform,0,3),interpolationdata2 );
    row(comData2,0) = interpolationdata2;
    ChppGikTools::minJerkCurve( 2*D, attSamplingPeriod, M4_IJ(sfTransform,1,3), 0, 0, M4_IJ(nsfTransform,1,3),interpolationdata2 );
    row(comData2,1) = interpolationdata2;
    subrange(comDataAll,0,2,n1, n1+n2 ) = comData2;
    
    ChppGikTools::minJerkCurve( D, attSamplingPeriod, M4_IJ(nsfTransform,0,3), 0, 0, com[0] ,interpolationdata1 );
    row(comData1,0) = interpolationdata1;
    ChppGikTools::minJerkCurve( D, attSamplingPeriod, M4_IJ(nsfTransform,1,3), 0, 0, com[1] ,interpolationdata1 );
    row(comData1,1) = interpolationdata1;
    subrange(comDataAll,0,2,n1+n2,2*n1+n2) = comData1;
    
    ChppGikComConstraint comc(*attRobot, com[0], com[1]);

    vector3d zAxis;
    zAxis = localPoint;
    zAxis[2] = 1;
    ChppGikParallelConstraint parc(*attRobot,*(attRobot->waist()),zAxis, zAxis);
    
    vector3d worldZ;
    worldZ = localPoint;
    worldZ[2] = 0.6487;
    ChppGikPlaneConstraint plc(*attRobot,*(attRobot->waist()),localPoint, worldZ, zAxis);
    std::vector<CjrlGikStateConstraint*> stack;
    
    stack.push_back(&nsfc);
    stack.push_back(&comc);
    stack.push_back(&parc);
    stack.push_back(&plc);
    
    vectorN activated =  attStandingRobot->maskFactory()->legsMask();
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    vectorN combined = weights;
    for (unsigned int i=0;i<combined.size();i++)
        combined(i) *= activated(i);
    ChppGikSupportPolygon* curSupportPolygon;
    gikSolver.weights(combined);
    for (unsigned int j = 1; j< comDataAll.size2();j++)
    {
        comc.targetXY( comDataAll(0,j), comDataAll(1,j));
        absZMPPla[0] = comDataAll(0,j);
        absZMPPla[1] = comDataAll(1,j);

        fixedFoot->computeJacobianJointWrtConfig();
        for (unsigned int i = 0; i< stack.size();i++)
        {
            stack[i]->computeValue();
            stack[i]->computeJacobian();
        }
        gikSolver.solve( stack );
        gikSolver.applySolution();

        attStandingRobot->updateDynamics(attSamplingPeriod, absZMPPla, absZMPObs, relZMPObs, relZMPPla);

        curSupportPolygon = attStandingRobot->supportPolygon();
        if (!curSupportPolygon->isPointInside(V3_I(absZMPObs,0), V3_I(absZMPObs,1)))
        {
            std::cout << "ZMP: " << V3_I(absZMPObs,0) <<" , "<< V3_I(absZMPObs,1) <<" out of polygon at iteration " << j << "\n";
            curSupportPolygon->print();
            return;
        }

        const vectorN& solutionConfig = attRobot->currentConfiguration();
        motion.appendSample(solutionConfig,relZMPPla, relZMPObs,absZMPPla, absZMPObs );
    }
    motion.dumpTo( "SpringMotion" );
}

void ChppGikTest::createHumanoidRobot()
{
    CjrlRobotDynamicsObjectConstructor<
    dynamicsJRLJapan::DynamicMultiBody,
    //dynamicsJRLJapan::HumanoidDynamicMultiBody,
    Chrp2OptHumanoidDynamicRobot,
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
