/*
* Copyright ( c ) 2008
*      IS/AIST-ST2I/CNRS Joint Japanese-French Robotics Laboratory ( JRL ).
* All rights reserved.
*/

#include <cstdio>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <time.h>
#include <sys/time.h>

#include <jrl/mal/matrixabstractlayer.hh>
#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"
#include "jrl/dynamics/dynamicsfactory.hh"
#include "hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h"

#include "hpp/gik/task/whole-body-task.hh"
#include "hpp/gik/constraint/position-constraint.hh"
#include "hpp/gik/constraint/parallel-constraint.hh"
#include "hpp/gik/constraint/gaze-constraint.hh"

#include "hpp/gik/task/half-sitting-task.hh"


#include "hpp/gik/motionplanner/element/step-element.hh"
#include "hpp/gik/motionplanner/element/interpolated-element.hh"


#include "hpp/gik/tools.hh"

#define M3_IJ MAL_S3x3_MATRIX_ACCESS_I_J
#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J
#define V3_I  MAL_S3_VECTOR_ACCESS

using namespace  boost::numeric::ublas;


ChppGikStandingRobot* attStandingRobot = 0;
CjrlHumanoidDynamicRobot* attRobot = 0;
dynamicsJRLJapan::ObjectFactory jrlRobotFactory;

void createHRP2 ( const std::string& inPath );
void resetRobot ();
void test1 ();
void test2 ();

void resetRobot ()
{
    attStandingRobot->staticState ( attStandingRobot->halfsittingConfiguration() );
}

void test1 ()
{
    std::cout << "Running test 1" << std::endl;
    unsigned int priority;

    ChppGikWholeBodyTask attWholeBodyTask ( attStandingRobot, 5e-3 );
    CjrlJoint* joint = attRobot->leftWrist();
    CjrlHand* hand = attRobot->leftHand();
    vector3d lpoint;
    hand->getCenter ( lpoint );
    vector3d targetPoint ( 0.8,0.2,0.8 );
    ChppGikPositionConstraint psc ( *attRobot,*joint,lpoint,targetPoint );
    priority = 4;
    attWholeBodyTask.addStateConstraint ( &psc,priority );
    ChppGikGazeConstraint gc ( *attRobot, targetPoint );
    priority = 5;
    attWholeBodyTask.addStateConstraint ( &gc,priority );
    bool solved = attWholeBodyTask.solve();
    attWholeBodyTask.solutionMotion().dumpTo ( "test1" );
    if ( solved )
    {
        attStandingRobot->staticState ( attWholeBodyTask.solutionMotion().lastSample()->configuration );
        attWholeBodyTask.solutionMotion().dumpTo ( "test1" );
        std::cout<< "Files for test 1 dumped."<< std::endl;
    }
    else
        std::cout<< "Test1 could not complete. 0 files dumped." << std::endl;
}

void test2 ()
{
    std::cout << "Running test 2" << std::endl;
    unsigned int priority;

    ChppGikWholeBodyTask attWholeBodyTask ( attStandingRobot, 5e-3 );
    CjrlJoint* joint = attRobot->leftWrist();
    CjrlHand* hand = attRobot->leftHand();
    vector3d lpoint,lvector;
    hand->getCenter ( lpoint );
    hand->getThumbAxis ( lvector );
    vector3d targetPoint ( 0.5,0.2,0.94 );
    vector3d targetAxis ( 0,0,1 );
    ChppGikPositionConstraint psc ( *attRobot,*joint,lpoint,targetPoint );
    ChppGikParallelConstraint pal ( *attRobot,*joint,lvector,targetAxis );
    priority = 10;
    attWholeBodyTask.addStateConstraint ( &psc,priority );
    priority = 20;
    attWholeBodyTask.addStateConstraint ( &pal,priority );
    ChppGikGazeConstraint gc ( *attRobot, targetPoint );
    priority = 30;
    attWholeBodyTask.addStateConstraint ( &gc,priority );
    bool solved = attWholeBodyTask.solve();
    if ( solved )
    {
        attStandingRobot->staticState ( attWholeBodyTask.solutionMotion().lastSample()->configuration );
        attWholeBodyTask.solutionMotion().dumpTo ( "test2" );
        std::cout<< "Files for test 2 dumped."<< std::endl;
    }
    else
        std::cout<< "Test2 could not complete. 0 files dumped."<< std::endl;
}

void createHRP2 ( const std::string& inPath )
{

    attRobot = new Chrp2OptHumanoidDynamicRobot ( &jrlRobotFactory );
    std::string linkJointFname,specificitiesFile;
    linkJointFname = inPath + "/HRP2LinkJointRank.xml";
    specificitiesFile = inPath + "/HRP2Specificities.xml";
    std::string modelname = inPath + "/HRP2.wrl";

    dynamicsJRLJapan::parseOpenHRPVRMLFile ( *attRobot, modelname,linkJointFname,specificitiesFile );

    std::string property,value;
    property="ComputeZMP"; value="true";attRobot->setProperty ( property,value );
    property="TimeStep"; value="0.005";attRobot->setProperty ( property,value );
    property="ComputeAccelerationCoM"; value="false";attRobot->setProperty ( property,value );
    property="ComputeBackwardDynamics"; value="false";attRobot->setProperty ( property,value );
    property="ComputeMomentum"; value="true";attRobot->setProperty ( property,value );
    property="ComputeAcceleration"; value="true";attRobot->setProperty ( property,value );
    property="ComputeVelocity"; value="true";attRobot->setProperty ( property,value );
    property="ComputeSkewCom"; value="false";attRobot->setProperty ( property,value );
    property="ComputeCoM"; value="true";attRobot->setProperty ( property,value );

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
        -10.0, 10.0, -10.0, 10.0, -10.0, // right hand parallel mechanism
        15.0,  10.0, 0.0, -30.0, 0.0, 0.0, // left arm
        10.0, // left hand clench
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
    gazeDir[0] = 0;
    gazeDir[1] = cos ( 10*M_PI/180 );
    gazeDir[2] = sin ( 10*M_PI/180 );

    gazeOrigin[0] = 0;
    gazeOrigin[1] = 0;
    gazeOrigin[2] = -0.118;

    attRobot->gaze ( gazeDir, gazeOrigin );

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

    attStandingRobot = new ChppGikStandingRobot ( *attRobot );
    attStandingRobot->staticState ( halfsittingConf );
}


int main ( int argc, char** argv )
{
    if ( argc !=2 )
    {
        std::cout << "Usage: tests [Path to folder containing files HRP2.wrl, HRP2LinkJointRank.xml and HRP2Specificities.xml]" <<std::endl;
        return  -1;
    }


    createHRP2 ( std::string ( argv[1] ) );
    test1();
    
    
    ChppGikHalfSittingTask task(attStandingRobot,0.005);
    if (!task.solve())
        std::cout << "Failed to complete ChppGikHalfSittingTask::solve" << std::endl;
    else
    {
        task.solutionMotion().dumpTo ( "test-halfsitting" );
        std::cout<< "Files for halfsitting test dumped."<< std::endl;
    }
    attStandingRobot->staticState ( task.solutionMotion().lastSample()->configuration );

    test2();

    delete attStandingRobot;

    return 0;
}
