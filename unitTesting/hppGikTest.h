#ifndef HPP_GIKTEST_H
#define HPP_GIKTEST_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "gikTask/jrlGikObjectConstructor.h"

#include "constraints/hppGikPlaneConstraint.h"
#include "constraints/hppGikPositionConstraint.h"
#include "constraints/hppGikRotationConstraint.h"
#include "constraints/hppGikTransformationConstraint.h"
#include "constraints/hppGikParallelConstraint.h"
#include "constraints/hppGikGazeConstraint.h"
#include "constraints/hppGikComConstraint.h"

#include "tasks/hppGikWholeBodyTask.h"
#include "tasks/hppGikStepBackTask.h"
#include "tasks/hppGikHandTask.h"
#include "tasks/hppGikHalfSittingTask.h"

#include "hppGikViewer.h"

/**
 \brief UnitTesting class of class ChppGik
*/
class ChppGikTest
{
public:
    /**
     \brief Constructor
    */
    ChppGikTest();

    /**
       \brief Destructor
    */
    ~ChppGikTest();

    /**
    \brief User commands interpreter
    */
    void interprete();
    
    /**
    \brief a method serving as an example on gikSolver usage
    */
    void basicExample();
    
    /**
    \brief
    */
    void locoPlannerTest();
    
    /**
    \brief
    */
    void locoPlannerTestInteractive();


private:

    /**
        \brief Create a humanoid robot
    */
    void createHumanoidRobot();

    void waist2worldPosition(vector3d& inWaistPosition, vector3d& outWorldPosition);
    
    void leftFoot2worldPosition(vector3d& inLFootPosition, vector3d& outWorldPosition);

    void printMenu();
    
    void printHelp();

    void dumpFilesAndGetLastConfig(ChppGikRobotTask* robotTask,const char* inFilename, vectorN& outLastConfig);

    bool read3coordinates(std::istringstream& stream, vector3d& coordinates);

    bool determineSide(std::istringstream& stream, bool& isRight);

    void halfsitting(const char* filename, vectorN& curConfig, vectorN& resultConfig);

    void stepback(const char* filename, vectorN& curConfig, vectorN& outLastConfig);

    void handGrasp(bool taskIsForRightHand, double valTighten, const char* filename, vectorN& curConfig, vectorN& outLastConfig);

    void handat(bool taskIsForRightHand, bool doOrientation,vector3d& coordinates, vector3d& orientation,const char* filename, vectorN& curConfig, vectorN& outLastConfig);

    void lookhandat(bool taskIsForRightHand, bool doOrientation,vector3d& coordinates, vector3d& orientation,const char* filename, vectorN& curConfig, vectorN& outLastConfig);

    void lookat(vector3d& coordinates, const char* filename, vectorN& curConfig, vectorN& outLastConfig);

    void goDownTree(const CjrlJoint* startJoint);
    
    void goDownChain(const CjrlJoint* startJoint);
    
    void setMotionName(std::istringstream& stream);
    
    void keepMotion();
    
    void dumpMotion();
    
    void clearMotion();
    
    void draw2DRobot();

    void customSave(std::istringstream& stream);

    ChppGikStandingRobot* attStandingRobot;
    
    CjrlHumanoidDynamicRobot* attRobot;
    
    CjrlGikObjectConstructor<ChppGikPlaneConstraint, ChppGikParallelConstraint, ChppGikRotationConstraint, ChppGikPositionConstraint, ChppGikTransformationConstraint, ChppGikPointingConstraint, ChppGikGazeConstraint, ChppGikComConstraint> attGikFactory;

    ChppGikRobotTask* attLastRobotTask;
    ChppGikWholeBodyTask* attWholeBodyTask;
    ChppGikHalfSittingTask* attHalfSittingTask;
    ChppGikHandTask* attHandTask;
    ChppGikStepBackTask *attStepBackTask;

    double attSamplingPeriod;

    bool attModeWaist;

    ChppRobotMotion* attMotion;
    
    char attMotionName[256];
    
    ChppGikViewer* attViewer;
    
    bool attViewerHealthy;

};

#endif
