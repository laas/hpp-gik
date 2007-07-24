
#include "tasks/hppGikHandTask.h"
#include "hppGikTools.h"
#define V3_I MAL_S3_VECTOR_ACCESS

ChppGikHandTask::ChppGikHandTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod,"HandTask")
{
    attHand = inStandingRobot->robot()->rightHand();
    attTargetClenchValue = 0.5;
    attMotionDuration = 2.0;
}


void ChppGikHandTask::forRightHand(bool inRight)
{
    if (inRight)
        attHand = attStandingRobot->robot()->rightHand();
    else
        attHand = attStandingRobot->robot()->leftHand();
}


double ChppGikHandTask::targetClench()
{
    return attTargetClenchValue;
}


void ChppGikHandTask::targetClench(double inTargetClenchValue)
{
    if (inTargetClenchValue > 1-1e-5)
        attTargetClenchValue = 1-1e-5;
    else
        if (inTargetClenchValue < 1e-5)
            attTargetClenchValue = 1e-5;
    else
        attTargetClenchValue = inTargetClenchValue;
}


void ChppGikHandTask::motionDuration(double inMotionDuration)
{
    attMotionDuration = inMotionDuration;
}


double ChppGikHandTask::motionDuration()
{
    return attMotionDuration;
}


bool ChppGikHandTask::algorithmSolve()
{
    unsigned int nSamples = ChppGikTools::timetoRank(0.0,attMotionDuration, attSamplingPeriod)+1;

    vectorN interpolationVector(nSamples);

    bool contn = ChppGikTools::minJerkCurve( attMotionDuration,attSamplingPeriod,attStandingRobot->robot()->getHandClench(attHand) ,0.0,0.0,attTargetClenchValue,interpolationVector);
    if (!contn)
        return false;

    bool atLeastOneZMPUnsafe = false;
    vector3d ZMPwstPla,ZMPwstObs,ZMPworPla,ZMPworObs;
    double staticZMPx,staticZMPy;

    ChppGikSupportPolygon* curSupportPolygon = attStandingRobot->supportPolygon();
    curSupportPolygon->center(staticZMPx,staticZMPy);
    V3_I(ZMPworPla,0) = staticZMPx;
    V3_I(ZMPworPla,1) = staticZMPy;
    V3_I(ZMPworPla,2) = 0.0;

    for (unsigned int i=0; i<nSamples; i++)
    {
        double val;
        val = interpolationVector(i);
        if (val > 1-1e-5)
            val = 1-1e-5;
        else
            if (val < 1e-5)
                val = 1e-5;
        attStandingRobot->robot()->setHandClench( attHand, val);
        attStandingRobot->updateDynamics(attSamplingPeriod, ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla);
	attStandingRobot->robot()->SaveCurrentStateAsPastState();
        if (!curSupportPolygon->isPointInside(V3_I(ZMPworObs,0), V3_I(ZMPworObs,1)))
            atLeastOneZMPUnsafe = true;
        attSolutionMotion->appendSample(attStandingRobot->robot()->currentConfiguration(), ZMPwstPla, ZMPwstObs, ZMPworPla, ZMPworObs);
    }

    if (atLeastOneZMPUnsafe)
        std::cout << "ChppGikHandTask::solve(): ZMP goes out of support polygon\n";

    return !atLeastOneZMPUnsafe;
}


ChppGikHandTask::~ChppGikHandTask()
{}
