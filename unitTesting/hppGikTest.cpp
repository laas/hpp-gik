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

#include "constraints/hppGikConfigurationConstraint.h"
#include "tasks/hppGikGenericTask.h"
#include "motionplanners/elements/hppGikStepElement.h"
#include "motionplanners/elements/hppGikInterpolatedElement.h"
#include "tasks/hppGikReachTask.h"
#include "tasks/hppGikStepTask.h"

#define M3_IJ MAL_S3x3_MATRIX_ACCESS_I_J
#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J
#define V3_I  MAL_S3_VECTOR_ACCESS

using namespace  boost::numeric::ublas;

ChppGikTest::ChppGikTest() : attSamplingPeriod ( 5e-3 )
{

    attLastRobotTask = 0;

    strcpy ( attMotionName,"usermotion" );

    createHumanoidRobot();

    attWholeBodyTask = new ChppGikWholeBodyTask ( attStandingRobot,attSamplingPeriod );

    attWholeBodyTask->showResolutionTime ( true );

    attHalfSittingTask = new ChppGikHalfSittingTask ( attStandingRobot, attSamplingPeriod );

    attHalfSittingTask->showResolutionTime ( true );

    attHandTask = new ChppGikHandTask ( attStandingRobot,attSamplingPeriod );

    attStepBackTask = new ChppGikStepBackTask ( attStandingRobot, attSamplingPeriod );

    attMotion = new ChppRobotMotion ( attRobot, 0.0, attSamplingPeriod );

/*



    //elementReach(true,vector3d(0.8,-0.2,1.0),vector3d(1.8,-0.2,1.0));


    ChppGikSolver solver ( *attRobot );
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    subrange ( weights,0,6 ) = zero_vector<double> ( 6 );
    for ( unsigned int i=0;i<46;i++ )
        if ( attStandingRobot->maskFactory()->wholeBodyMask() ( i ) ==0 )
            weights ( i ) =0;
    solver.weights ( weights );
    solver.rootJoint ( *attRobot->rightFoot() );

    double config[] =
        {2.61043,-0.0101041,0.59099,-0.473436,1.0366,-2.85369,-0.373352,0.214087,-1.28175,0.65467,-0.284436,0.43532,-0.447899,0.535548,-0.707108,2.17918,-0.115944,-0.187288,0.0690899,0.327885,-0.515242,0.483151,-1.71429,-1.29046,-1.16917,-1.48978,0.972735,0.703203,0,0.128455,0.799528,1.21547,-2.15955,-0.350344,-1.00442,0,0,0,0,0,0,0,0,0,0,0};

    double velo[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    vectorN strConfig ( 46 );
    vectorN strVelo ( 46 );
    for ( unsigned int i=0;i<46;i++ )
    {
        strConfig ( i ) = config[i];
        strVelo ( i ) = velo[i];
    }

    attRobot->currentConfiguration ( strConfig );
    attRobot->currentVelocity ( strVelo );
    attRobot->computeForwardKinematics();

    matrix4d m;
    M4_IJ ( m,0,0 ) = -0.954725;
    M4_IJ ( m,0,1 ) =  0.297489;
    M4_IJ ( m,0,2 ) = 5.55654e-18;
    M4_IJ ( m,0,3 ) =  2.5253;

    M4_IJ ( m,1,0 ) = -0.297489;
    M4_IJ ( m,1,1 ) =  -0.954725;
    M4_IJ ( m,1,2 ) = 0;
    M4_IJ ( m,1,3 ) =  0.104594;

    M4_IJ ( m,2,0 ) = -5.55654e-18;
    M4_IJ ( m,2,1 ) =  0;
    M4_IJ ( m,2,2 ) = 1;
    M4_IJ ( m,2,3 ) =  0.104998;



    std::vector<CjrlGikStateConstraint*> tasks;
    ChppGikTransformationConstraint fc ( *attRobot,*attRobot->leftFoot(),vector3d ( 0,0,0 ),m );
    tasks.push_back ( &fc );
    ChppGikComConstraint comc ( *attRobot,2.49362,0.192589 );
    tasks.push_back ( &comc );

    ChppGikPositionConstraint posc ( *attRobot,*attRobot->rightWrist(),vector3d ( 0,0,-0.17 ),vector3d ( 1.78656,0,0.0294494 ) );
    tasks.push_back ( &posc );


    double cfgcstr[] = {3.44955,0.147283,0.668866,0,1.27224,0,-0.373352,0.214087,-1.28175,0.65467,-0.284436,0.43532,-0.447899,0.535548,-0.707108,2.17918,-0.115944,-0.187288,0.0690899,0.327885,-0.515242,0.483151,-1.71429,-1.29046,-1.16917,-1.48978,0.972735,0.703203,0,0.128455,0.799528,1.21547,-2.15955,-0.350344,-1.00442,0,0,0,0,0,0,0,0,0,0,0};

    vectorN cfcConfig ( 46 );
    for ( unsigned int i=0;i<46;i++ )
cfcConfig ( i ) = cfgcstr[i];

ChppGikConfigurationConstraint cfc ( *attRobot, cfcConfig, attStandingRobot->maskFactory()->wholeBodyMask() );
    tasks.push_back ( &cfc );


    solver.prepare ( tasks );
    solver.solve ( tasks );

//    attStandingRobot->updateRobot ( solver.solutionRootPose(),solver.solutionJointConfiguration(),0.005 );
    attRobot->currentConfiguration(solver.solution());
    attRobot->computeForwardKinematics();

    std::cout << attRobot->currentConfiguration() << std::endl;
    */
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

void ChppGikTest::elementReach ( bool taskIsForRightHand,
                                 const vector3d& inReachTarget,
                                 const vector3d& inGazeTarget )
{
    vectorN BackupConfig = attStandingRobot->robot()->currentConfiguration();

    std::vector<CjrlGikStateConstraint*> tasks;
    CjrlJoint* joint;
    vector3d lpoint;
    CjrlHand* hand;
    if ( taskIsForRightHand )
    {
        joint = attStandingRobot->robot()->rightWrist();
        hand =  attStandingRobot->robot()->rightHand();

    }
    else
    {
        joint = attStandingRobot->robot()->leftWrist();
        hand =  attStandingRobot->robot()->leftHand();
    }
    lpoint = hand->centerInWristFrame();


    tasks.push_back ( new ChppGikTransformationConstraint ( * ( attStandingRobot->robot() ),*attStandingRobot->robot()->rightFoot(),vector3d ( 0,0,0 ),attStandingRobot->robot()->rightFoot()->currentTransformation() ) );

    tasks.push_back ( new ChppGikComConstraint ( * ( attStandingRobot->robot() ),attStandingRobot->robot()->positionCenterOfMass() [0],attStandingRobot->robot()->positionCenterOfMass() [1] ) );

    tasks.push_back ( new ChppGikPositionConstraint ( * ( attStandingRobot->robot() ),*joint,lpoint,inReachTarget ) );

    tasks.push_back ( new ChppGikGazeConstraint ( * ( attStandingRobot->robot() ), inGazeTarget ) );

    std::vector<double> dampers ( tasks.size(),1.0 );
    dampers[0] = dampers[1] = 0.0;

    solveStack ( tasks, dampers );

    for ( unsigned int i=0;i<tasks.size();i++ )
        delete tasks[i];


    attStandingRobot->staticState ( BackupConfig );
}

void ChppGikTest::solveStack ( std::vector<CjrlGikStateConstraint*>& inConstraints, const std::vector<double>& inDamp )
{
    ChppRobotMotion gMotion ( attStandingRobot->robot(),0.0,attSamplingPeriod );
    vector3d dummy;
    std::vector<double> prevVals, curVals;
    std::vector<double> dampers = inDamp;



    ChppGikSolver attGikSolver ( *attStandingRobot->robot() );

    attGikSolver.rootJoint ( *attStandingRobot->robot()->leftFoot() );

    vectorN weights = attStandingRobot->maskFactory()->wholeBodyMask();
    subrange ( weights,0,6 ) = zero_vector<double> ( 6 );
    attGikSolver.weights ( weights );

    attGikSolver.prepare ( inConstraints );

    bool breakLoop = false;
    double minProgress = 1e-4;
    while ( !breakLoop )
    {
        for ( unsigned int i = 0; i<inConstraints.size();i++ )
        {
            prevVals.push_back ( norm_2 ( inConstraints[i]->value() ) );
            curVals.push_back ( prevVals[i] );
        }

        //attGikSolver->solve( inConstraints );
        attGikSolver.solve ( inConstraints, dampers );
        attStandingRobot->updateRobot ( attGikSolver.solutionRootPose(), attGikSolver.solutionJointConfiguration(), attSamplingPeriod );
        gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,dummy,dummy,dummy );
        for ( unsigned int i = 0; i<inConstraints.size();i++ )
        {
            inConstraints[i]->computeValue();
            inConstraints[i]->value();
            curVals[i] = norm_2 ( inConstraints[i]->value() );
        }

        if ( norm_2 ( attStandingRobot->robot()->currentVelocity() ) <1e-4 )
            breakLoop = true;
        else
            breakLoop = false;

        /*
        breakLoop = true;
        for (unsigned int i = 0; i<inConstraints.size();i++)
        {
            if (curVals[i] > prevVals[i])//diverges
            {
                break;
            }
            else if (curVals[i] < prevVals[i] - minProgress) //significant progress
            {
                breakLoop = false;
                break;
            }
        }
        if (breakLoop)
        {
            std::cout << "Diverged." << std::endl;
            break;
        }

        breakLoop = true;
        */
    }



    gMotion.dumpTo ( "damped" );
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
    aHDMB = ( dynamicsJRLJapan::HumanoidDynamicMultiBody* ) attRobot;

    std::string path = "./";
    std::string name = "HRP2.wrl";
    aHDMB->parserVRML ( path,name,"./HRP2LinkJointRank.xml" );
    std::string aName="./HRP2Specificities.xml";
    aHDMB->SetHumanoidSpecificitiesFile ( aName );
    aHDMB->SetTimeStep ( attSamplingPeriod );
    aHDMB->setComputeVelocity ( true );
    aHDMB->setComputeMomentum ( true );
    aHDMB->setComputeCoM ( true );
    aHDMB->setComputeAcceleration ( true );
    aHDMB->setComputeZMP ( true );

    aHDMB->setComputeSkewCoM ( false );
    aHDMB->setComputeAccelerationCoM ( false );
    aHDMB->setComputeBackwardDynamics ( false );



    unsigned int nDof = attRobot->numberDof();
    vectorN halfsittingConf ( nDof );

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
    halfsittingConf ( 0 ) = 0.0;
    halfsittingConf ( 1 ) = 0.0;
    halfsittingConf ( 2 ) = 0.6487;
    //waist roll pitch yaw
    halfsittingConf ( 3 ) = 0.0;
    halfsittingConf ( 4 ) = 0.0;
    halfsittingConf ( 5 ) = 0.0;

    //joints
    for ( unsigned int i=6;i<nDof;i++ )
        halfsittingConf ( i ) = dInitPos[i-6]*M_PI/180;

    zero_vector<double> zeros ( attRobot->numberDof() );
    attRobot->currentConfiguration ( halfsittingConf );
    attRobot->currentVelocity ( zeros );
    attRobot->currentAcceleration ( zeros );
    attRobot->computeForwardKinematics();

    //set gaze origin and direction
    vector3d gazeDir,gazeOrigin;
    gazeDir[0] = 1;
    gazeDir[1] = 0;
    gazeDir[2] = 0;

    gazeOrigin[0] = 0;
    gazeOrigin[1] = 0;
    gazeOrigin[2] = 0.118;

    aHDMB->gaze ( ( const vector3d& ) gazeDir, ( const vector3d& ) gazeOrigin );

    ChppGik2DShape leftFootShape, rightFootShape;
    
    ChppGik2DVertex vert;

    vert.x = 0.135;
    vert.y = 0.079;
    leftFootShape.vertices.push_back ( vert );
    vert.x = -0.105;
    leftFootShape.vertices.push_back ( vert );
    vert.y = -0.059;
    leftFootShape.vertices.push_back ( vert );
    vert.x = 0.135;
    leftFootShape.vertices.push_back ( vert );
    
    vert.x = 0.135;
    vert.y = 0.059;
    rightFootShape.vertices.push_back ( vert );
    vert.x = -0.105;
    rightFootShape.vertices.push_back ( vert );
    vert.y = -0.079;
    rightFootShape.vertices.push_back ( vert );
    vert.x = 0.135;
    rightFootShape.vertices.push_back ( vert );
    
    attStandingRobot = new ChppGikStandingRobot ( *attRobot, leftFootShape, rightFootShape );
    attStandingRobot->staticState ( halfsittingConf );
}

void ChppGikTest::waist2worldPosition ( vector3d& inWaistPosition, vector3d& outWorldPosition )
{
    matrix4d m;

    m = attRobot->rootJoint()->currentTransformation();
    outWorldPosition = m * inWaistPosition;
}

void ChppGikTest::leftFoot2worldPosition ( vector3d& inLFootPosition, vector3d& outWorldPosition )
{
    matrix4d m = attRobot->leftFoot()->currentTransformation();
    MAL_S4x4_MATRIX_ACCESS_I_J ( m,2,3 ) = MAL_S4x4_MATRIX_ACCESS_I_J ( m,2,3 ) - attRobot->footHeight();
    outWorldPosition = m * inLFootPosition;
}

void ChppGikTest::dumpFilesAndGetLastConfig ( ChppGikRobotTask* robotTask,const char* inFilename, vectorN& outLastConfig )
{
    //dump files
    robotTask->solutionMotion().dumpTo ( inFilename );

    double eT = robotTask->solutionMotion().endTime();
    bool assertion = robotTask->solutionMotion().configAtTime ( eT, outLastConfig );

    if ( !assertion )
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

    while ( !do_quit )
    {
        if ( printit )
            printMenu();
        printit = true;

        getline ( cin,line );
        stream.clear();
        stream.str ( line );
        stream >> command;

        if ( strcmp ( command, "look" ) ==0 )
        {
            if ( !read3coordinates ( stream, position ) )
                continue;
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            lookat ( position,motion_file.c_str(),currentConfig, resultConfig );
            continue;
        }

        if ( strcmp ( command, "hand" ) ==0 )
        {

            if ( !determineSide ( stream, SideIsRight ) )
                continue;
            if ( !read3coordinates ( stream, position ) )
                continue;

            bool doOrientation = !stream.eof();
            if ( doOrientation )
                if ( !read3coordinates ( stream, orientation ) )
                    continue;

            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }

            handat ( SideIsRight,doOrientation,position,orientation,motion_file.c_str(),currentConfig, resultConfig );
            continue;
        }

        if ( strcmp ( command, "step" ) ==0 )
        {

            if ( !determineSide ( stream, SideIsRight ) )
                continue;

            for ( unsigned int k=0; k<3;k++ )
            {
                if ( !stream.eof() )
                    stream >> position[k];
                else
                {
                    std::cout << "Incorrect number of arguments\n";
                    continue;
                }
            }


            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }

            step ( SideIsRight,position,motion_file.c_str(),currentConfig, resultConfig );
            continue;
        }

        if ( strcmp ( command, "lookhand" ) ==0 )
        {

            if ( !determineSide ( stream, SideIsRight ) )
                continue;
            if ( !read3coordinates ( stream, position ) )
                continue;

            bool doOrientation = !stream.eof();
            if ( doOrientation )
                if ( !read3coordinates ( stream, orientation ) )
                    continue;

            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }

            lookhandat ( SideIsRight,doOrientation,position,orientation,motion_file.c_str(),currentConfig, resultConfig );
            continue;
        }

        if ( strcmp ( command, "closehand" ) ==0 )
        {
            if ( !determineSide ( stream, SideIsRight ) )
                continue;
            if ( stream.eof() )
                continue;

            double valTighten;
            stream >> valTighten;

            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }

            handGrasp ( SideIsRight, valTighten, motion_file.c_str(),currentConfig, resultConfig );
            continue;
        }

        if ( strcmp ( command, "openhand" ) ==0 )
        {
            if ( !determineSide ( stream, SideIsRight ) )
                continue;
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            double valTighten = 0.0;
            handGrasp ( SideIsRight, valTighten,motion_file.c_str(),currentConfig, resultConfig );
            continue;
        }

        if ( strcmp ( command, "stepback" ) ==0 )
        {
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            stepback ( motion_file.c_str(),currentConfig, resultConfig );
            continue;
        }

        if ( strcmp ( command, "halfsitting" ) ==0 )
        {
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            halfsitting ( motion_file.c_str(),currentConfig, resultConfig );
            continue;
        }

        if ( strcmp ( command, "hrp" ) ==0 )
        {
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            string cmd;

            cmd = "scp " + motion_file + ".* hrp2014c:/home/okanoun/Demo16Avril2007/.";
            system ( cmd.c_str() );

            currentConfig = resultConfig;
        }

        if ( strcmp ( command, "apply" ) ==0 )
        {
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            currentConfig = resultConfig;
        }

        if ( strcmp ( command, "switch" ) ==0 )
        {
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            attModeWaist = !attModeWaist;
        }

        if ( strcmp ( command, "motionclear" ) ==0 )
        {
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            clearMotion();
        }

        if ( strcmp ( command, "motiondump" ) ==0 )
        {
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            dumpMotion();
        }

        if ( strcmp ( command, "motionkeep" ) ==0 )
        {
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            keepMotion();
        }

        if ( strcmp ( command, "motionname" ) ==0 )
        {
            setMotionName ( stream );
        }


        if ( strcmp ( command, "help" ) ==0 )
        {
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            printHelp();
            printit = false;
            continue;
        }

        if ( strcmp ( command, "quit" ) ==0 )
        {
            if ( !stream.eof() )
            {
                std::cout << "check arguments\n";
                continue;
            }
            std::cout << "\n\n";
            break;
        }
    }
}



void ChppGikTest::lookat ( vector3d& targetPoint, const char* filename, vectorN& curConfig, vectorN& resultConfig )
{
    attLastRobotTask = attWholeBodyTask;

    unsigned int priority;
    //the robot is static at the current configuration
    attStandingRobot->staticState ( curConfig );
    //Reset planner
    attWholeBodyTask->reset();

    CjrlGikGazeConstraint* gc = attGikFactory.createGazeConstraint ( *attRobot, targetPoint );
    priority = 1;

    attWholeBodyTask->addStateConstraint ( gc,priority );

    bool solved = attWholeBodyTask->solve();

    if ( solved )
        dumpFilesAndGetLastConfig ( attWholeBodyTask, filename, resultConfig );
    else
        std::cout << "\nNot solved.\n";

    delete gc;
}

void ChppGikTest::handat ( bool taskIsForRightHand, bool doOrientation,vector3d& targetPoint, vector3d& targetOrientation,const char* filename, vectorN& curConfig, vectorN& resultConfig )
{
    attLastRobotTask = attWholeBodyTask;

    unsigned int priority;
    //the robot is static at the current configuration
    attStandingRobot->staticState ( curConfig );
    //Reset planner
    attWholeBodyTask->reset();

    CjrlJoint* joint = 0;
    CjrlHand* hand = 0;

    if ( taskIsForRightHand )
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

    psc = attGikFactory.createPositionConstraint ( *attRobot,*joint,lpoint,targetPoint );
    priority = 1;
    attWholeBodyTask->addStateConstraint ( psc,priority );

    if ( doOrientation )
    {
        poc = attGikFactory.createParallelConstraint ( *attRobot,*joint,laxis,targetOrientation );
        priority = 1;
        attWholeBodyTask->addStateConstraint ( poc,priority );
    }

    bool solved = attWholeBodyTask->solve();

    if ( solved )
        dumpFilesAndGetLastConfig ( attWholeBodyTask,filename, resultConfig );
    else
        std::cout<< "\nNot solved.\n";

    delete psc;
    delete poc;

}

void ChppGikTest::lookhandat ( bool taskIsForRightHand, bool doOrientation,vector3d& targetPoint, vector3d& targetOrientation,const char* filename, vectorN& curConfig, vectorN& resultConfig )
{
    attLastRobotTask = attWholeBodyTask;

    unsigned int priority;
    //the robot is static at the current configuration
    attStandingRobot->staticState ( curConfig );
    //Reset planner
    attWholeBodyTask->reset();

    CjrlJoint* joint = 0;
    CjrlHand* hand = 0;

    if ( taskIsForRightHand )
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

    psc = attGikFactory.createPositionConstraint ( *attRobot,*joint,lpoint,targetPoint );
    priority = 1;
    attWholeBodyTask->addStateConstraint ( psc,priority );

    if ( doOrientation )
    {
        poc = attGikFactory.createParallelConstraint ( *attRobot,*joint,laxis,targetOrientation );
        priority = 1;
        attWholeBodyTask->addStateConstraint ( poc,priority );
    }

    //add gaze constraint: look at the hand target
    CjrlGikGazeConstraint* gc = attGikFactory.createGazeConstraint ( *attRobot, targetPoint );
    priority = 2;
    attWholeBodyTask->addStateConstraint ( gc,priority );

    bool solved = attWholeBodyTask->solve();

    if ( solved )
        dumpFilesAndGetLastConfig ( attWholeBodyTask,filename, resultConfig );
    else
        std::cout<< "\nNot solved.\n";

    delete psc;
    delete poc;
    delete gc;

}


void ChppGikTest::handGrasp ( bool taskIsForRightHand, double valTighten, const char* filename, vectorN& curConfig, vectorN& resultConfig )
{
    attLastRobotTask = attHandTask;

    //the robot is static at the current configuration
    attStandingRobot->staticState ( curConfig );

    attHandTask->forRightHand ( taskIsForRightHand );
    attHandTask->targetClench ( valTighten );

    bool solved = attHandTask->solve();

    if ( solved )
        dumpFilesAndGetLastConfig ( attHandTask, filename, resultConfig );
    else
        std::cout <<"\nNot solved.\n";
}


void ChppGikTest::stepback ( const char* filename, vectorN& curConfig, vectorN& resultConfig )
{
    attLastRobotTask = attStepBackTask;

    //the robot is static at the current configuration
    attStandingRobot->staticState ( curConfig );

    bool solved = attStepBackTask->solve();

    if ( solved )
    {
        if ( !attStepBackTask->solutionMotion().empty() )
            dumpFilesAndGetLastConfig ( attStepBackTask,filename, resultConfig );
    }
    else
        std::cout <<"\nNot solved.\n";
}

void ChppGikTest::step ( bool inForRight, const vector3d& targetFoot, const char* filename, vectorN& curConfig, vectorN& resultConfig )
{
    attLastRobotTask = attStepBackTask;

    //the robot is static at the current configuration
    attStandingRobot->staticState ( curConfig );

    ChppGikSupportPolygon* curSP = attStandingRobot->supportPolygon();
    if ( !curSP )
    {
        std::cout << "Could not step, incorrect support polygon" << std::endl;
        return;
    }
    if ( ! ( curSP->isDoubleSupport() ) )
    {
        std::cout << "Could not step, support polygon not double" << std::endl;
        return;
    }

    ChppGikStepTask stepTask ( attStandingRobot,attSamplingPeriod,inForRight,targetFoot[0],targetFoot[1],targetFoot[2] );

    stepTask.showResolutionTime ( true );
    bool solved = stepTask.solve();

    if ( solved )
    {
        if ( !stepTask.solutionMotion().empty() )
            dumpFilesAndGetLastConfig ( &stepTask,filename, resultConfig );
    }
    else
        std::cout <<"\nNot solved.\n";
}

void ChppGikTest::halfsitting ( const char* filename, vectorN& curConfig, vectorN& resultConfig )
{
    attLastRobotTask = attHalfSittingTask;

    //the robot is static at the current configuration
    attStandingRobot->staticState ( curConfig );

    bool solved = attHalfSittingTask->solve();

    if ( solved )
        dumpFilesAndGetLastConfig ( attHalfSittingTask, filename, resultConfig );
    else
        std::cout <<"\nNot solved.\n";

}

void  ChppGikTest::printMenu()
{
    std::string message;


    std::cout << "\n------------------------------\n";

    if ( attModeWaist )
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
    std::cout << "- step r/l dx dy dtheta\n";
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
    std::cout << "- use \"step r/l dx dy dtheta\" to make the robot step with right/left foot to given coordinates in left/right frame\n";
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

bool ChppGikTest::read3coordinates ( std::istringstream& stream, vector3d& coordinates )
{
    vector3d inputVec;
    //read target point in waist frame
    for ( unsigned int i=0; i<3;i++ )
    {
        if ( !stream.eof() )
            stream >> inputVec[i];
        else
        {
            std::cout << "Incorrect number of arguments\n";
            return false;
        }
    }

    if ( attModeWaist )
        waist2worldPosition ( inputVec, coordinates );
    else
        leftFoot2worldPosition ( inputVec, coordinates );

    return true;
}


bool ChppGikTest::determineSide ( std::istringstream& stream, bool& isRight )
{
    char whichside[16];
    if ( !stream.eof() )
        stream >> whichside;
    else
    {
        std::cout << "Incorrect number of arguments\n";
        return false;
    }

    if ( strcmp ( whichside, "r" ) ==0 )
        isRight =  true;
    else if ( strcmp ( whichside, "l" ) ==0 )
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
    if ( attLastRobotTask )
    {
        attMotion->appendMotion ( attLastRobotTask->solutionMotion() );
        attLastRobotTask = 0;
    }
}

void ChppGikTest::dumpMotion()
{
    attMotion->dumpTo ( attMotionName );
}

void ChppGikTest::setMotionName ( std::istringstream& stream )
{
    if ( !stream.eof() )
        stream >> attMotionName;
    else
    {
        std::cout << "Specify a name for the result motion\n";
        return;
    }
}

void ChppGikTest::goDownChain ( const CjrlJoint* startJoint )
{
    std::cout << "joint ranked " << startJoint->rankInConfiguration() << std::endl;
    std::cout << startJoint->currentTransformation() << std::endl;

    if ( startJoint->countChildJoints() != 0 )
    {
        const CjrlJoint* childJoint = startJoint->childJoint ( 0 );
        goDownChain ( childJoint );
    }
}

void ChppGikTest::goDownTree ( const CjrlJoint* startJoint )
{
    std::cout << "joint ranked " << startJoint->rankInConfiguration() << std::endl;
    std::cout << startJoint->currentTransformation() << std::endl;

    for ( unsigned int i = 0; i<startJoint->countChildJoints(); i++ )
    {
        const CjrlJoint* childJoint = startJoint->childJoint ( i );
        goDownTree ( childJoint );
    }
}
