#include "tasks/hppGikStepBackTask.h"
#include "constraints/hppGikParallelConstraint.h"
#include "constraints/hppGikPlaneConstraint.h"
#define V3_I  MAL_S3_VECTOR_ACCESS

ChppGikStepBackTask::ChppGikStepBackTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod, "StepbackTask")
{
    attGenericTask = new ChppGikGenericTask(inStandingRobot, inSamplingPeriod);
    attTargetFeetDistance = inStandingRobot->halfsittingFeetDistance();
    attAverageHeight = inStandingRobot->halfsittingWaistHeight()-0.05;

    automaticFoot(true);
}

void ChppGikStepBackTask::automaticFoot(bool inAutomatic, bool inSelectedFootIsRight  )
{
    attAutomaticFoot = inAutomatic;
    attSelectedFootisRight = inSelectedFootIsRight;
}

double ChppGikStepBackTask::targetFeetDistance()
{
    return attTargetFeetDistance;
}


void ChppGikStepBackTask::targetFeetDistance(double inFeetDistance)
{
    attTargetFeetDistance = inFeetDistance;
}


bool ChppGikStepBackTask::algorithmSolve()
{
    if (!(attStandingRobot->supportPolygon()->isDoubleSupport()))
    {
        std::cout << "ChppGikStepBackTask::solve() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    CjrlJoint* displacedFoot = 0;
    ChppGikFootprint* targetFootprint;
    const ChppGikFootprint *leftFootprint, *rightFootprint;
    const ChppGikFootprint *supportFootprint, *nonsupportFootprint;
    bool FootisRight;
    double relativeTargetY,relativeTargetX =0.0;
    std::string message;

    if (attAutomaticFoot)
    {
        if (attStandingRobot->rightFootAhead())
            attSelectedFootisRight = true;
        else
            if (attStandingRobot->leftFootAhead())
                attSelectedFootisRight = false;
            else
            {
                std::cout << "No need to make a step back.\n";
                return true;
            }
    }

    if (attSelectedFootisRight)
    {
        nonsupportFootprint = attStandingRobot->supportPolygon()->rightFootprint();
        supportFootprint = attStandingRobot->supportPolygon()->leftFootprint();
        FootisRight = true;
        relativeTargetY = -attTargetFeetDistance;
        message = "Stepping back with the right foot\n";
    }
    else
    {
        supportFootprint = attStandingRobot->supportPolygon()->rightFootprint();
        nonsupportFootprint = attStandingRobot->supportPolygon()->leftFootprint();
        FootisRight = false;
        relativeTargetY = attTargetFeetDistance;
        message = "Stepping back with the left foot\n";
    }

    std::cout << message.c_str();

    targetFootprint =  new ChppGikFootprint(*supportFootprint);
    targetFootprint->localTranslate(relativeTargetX, relativeTargetY);

    /*ZMP shift element*/
    double middleX,middleY;
    double shiftStartTime = 1.7;
    double shiftDuration = 0.6;
    attStandingRobot->supportPolygon()->center( middleX, middleY );
    ChppGikZMPshiftElement* zmpshiftElement = new ChppGikZMPshiftElement(middleX, middleY, shiftStartTime, shiftDuration);

    /*waist normal height recovery (element)*/
    double whStartTime = 0.0;
    double whDuration = 3.0;
    ChppGikSingleMotionElement* waistheightElement = 0;
    vector3d lposition,planePosition,planeNormal;
    lposition[0] = 0;
    lposition[1] = 0;
    lposition[2] = 0;
    planePosition[0] = 0;
    planePosition[1] = 0;
    planePosition[2] = attAverageHeight;
    planeNormal[0] = 0;
    planeNormal[1] = 0;
    planeNormal[2] = 1;
    ChppGikPlaneConstraint* waistheightConstraint = new ChppGikPlaneConstraint(*(attStandingRobot->robot()),*(attStandingRobot->robot()->waist()),lposition,planePosition,planeNormal);
    waistheightElement = new ChppGikSingleMotionElement(waistheightConstraint, 3, whStartTime, whDuration);

    /* step back element*/
    double stepStartTime = whStartTime+whDuration;
    double zmpstartshifttime = 0.4;
    double zmpendshifttime = 0.4;
    double footflighttime = 0.9;
    vector3d& relGap= attStandingRobot->halfsittingRelativeCOM();
    ChppGikStepElement* stepElement = new ChppGikStepElement( targetFootprint,stepStartTime, FootisRight, V3_I(relGap,0), V3_I(relGap,1), zmpendshifttime, zmpstartshifttime, footflighttime);
    //ChppGikStepElement* stepElement = new ChppGikStepElement(stepStartTime, targetFootprint, FootisRight, 0.5, zmpendshifttime, zmpstartshifttime, footflighttime);

    /*waist is always vertical (element)*/
    ChppGikSingleMotionElement* waistverticalElement = 0;
    vector3d targetOrientation,laxis;
    laxis[0] = 0;
    laxis[1] = 0;
    laxis[2] = 1;
    targetOrientation = laxis;
    ChppGikParallelConstraint* waistverticalConstraint = new ChppGikParallelConstraint(*(attStandingRobot->robot()),*(attStandingRobot->robot()->waist()),laxis,targetOrientation);
    waistverticalElement = new ChppGikSingleMotionElement(waistverticalConstraint, 2, 0.0, stepStartTime+stepElement->duration()+1.0);

    /*element stack*/
    attGenericTask->clearElements();
    attGenericTask->addElement(waistverticalElement);
    attGenericTask->addElement(zmpshiftElement);
    attGenericTask->addElement(waistheightElement);
    attGenericTask->addElement(stepElement);

    delete targetFootprint;
    delete waistverticalConstraint;
    delete waistheightConstraint;

    bool isSolved = attGenericTask->solve();

    cropMotion( attGenericTask );

    return isSolved;
}

ChppGikStepBackTask::~ChppGikStepBackTask()
{
    delete attGenericTask;
    cleanUp();
}
