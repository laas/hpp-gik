#include "tasks/hppGikSnoopTask.h"
#include "hppGikTools.h"
#include "constraints/hppGikPlaneConstraint.h"
#include "constraints/hppGikGazeConstraint.h"
#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J

ChppGikSnoopTask::ChppGikSnoopTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, double inHeadHeight, double inHeadAdvancement, const vector3d& inGazeTarget):ChppGikRobotTask(inStandingRobot,inSamplingPeriod,"SnoopTask")
{
    attGenericTask = new ChppGikGenericTask(inStandingRobot, inSamplingPeriod);
    attHeadHeight = inHeadHeight;
    attHeadAdvancement = inHeadAdvancement;
    attGazeTarget = inGazeTarget;
}

ChppGikSnoopTask::~ChppGikSnoopTask()
{
    delete attGenericTask;
    cleanUp();
}


void ChppGikSnoopTask::gazeTarget(const vector3d& inGazeTarget)
{
    attGazeTarget = inGazeTarget;
}

void ChppGikSnoopTask::headHeight(double inValue)
{
    attHeadHeight = inValue;
}

void ChppGikSnoopTask::headAdvancement(double inValue)
{
    attHeadAdvancement = inValue;
}

bool ChppGikSnoopTask::algorithmSolve()
{
    
    //gaze task
    double gzStart = 4.01;
    double gzDuration = 7.0;
    ChppGikGazeConstraint* gazeConstraint = new ChppGikGazeConstraint(*(attStandingRobot->robot()), attGazeTarget);
    ChppGikInterpolatedElement* gazeElement = new ChppGikInterpolatedElement(attStandingRobot->robot(), gazeConstraint, 3, gzStart, gzDuration, attSamplingPeriod);
    gazeElement->workingJoints( attStandingRobot->maskFactory()->wholeBodyMask() );
    
    double hhStartTime = 0.0;
    double hhDuration = 4.0;
    
    //Head height task
    ChppGikInterpolatedElement* headheightElement = 0;
    vector3d lposition,planePosition,planeNormal, ldirection;
    attStandingRobot->robot()->gaze(  ldirection ,lposition);
    planePosition[0] = 0;
    planePosition[1] = 0;
    planePosition[2] = attHeadHeight;
    planeNormal[0] = 0;
    planeNormal[1] = 0;
    planeNormal[2] = 1;
    ChppGikPlaneConstraint* headheightConstraint = new ChppGikPlaneConstraint(*(attStandingRobot->robot()),*(attStandingRobot->robot()->gazeJoint()),lposition,planePosition,planeNormal);
    headheightElement = new ChppGikInterpolatedElement(attStandingRobot->robot(), headheightConstraint, 2, hhStartTime, hhDuration, attSamplingPeriod);
    headheightElement->workingJoints( attStandingRobot->maskFactory()->wholeBodyMask() );

    //Head x task
    ChppGikInterpolatedElement* headXElement = 0;
    double ox,oy;
    attStandingRobot->supportPolygon()->center( ox,oy);
    planeNormal = attStandingRobot->supportPolygon()->meanOrientation();
    planePosition[0] = ox + planeNormal[0]*attHeadAdvancement;
    planePosition[1] = oy + planeNormal[1]*attHeadAdvancement;
    planePosition[2] = 0;
    ChppGikPlaneConstraint* headXConstraint = new ChppGikPlaneConstraint(*(attStandingRobot->robot()),*(attStandingRobot->robot()->gazeJoint()),lposition,planePosition,planeNormal);
    headXElement = new ChppGikInterpolatedElement(attStandingRobot->robot(), headXConstraint, 2, hhStartTime, hhDuration, attSamplingPeriod);

    attGenericTask->clearElements();
    attGenericTask->addElement( headheightElement );
    attGenericTask->addElement( headXElement );
    attGenericTask->addElement( gazeElement );

    
            
    delete gazeConstraint;
    delete headheightConstraint;
    delete headXConstraint;

    bool isSolved = attGenericTask->solve();

    cropMotion( attGenericTask );

    return isSolved;
}
