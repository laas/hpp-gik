
#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/task/hand-task.hh"
#include "hpp/gik/tools.hh"
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

    //bool atLeastOneZMPUnsafe = false;
    vector3d ZMPwstPla,ZMPwstObs,ZMPworPla,ZMPworObs;
    vectorN jointConfig(attStandingRobot->robot()->numberDof()-6) ;
    double staticZMPx,staticZMPy;

    ChppGikSupportPolygon* curSupportPolygon = attStandingRobot->supportPolygon();
    curSupportPolygon->center(staticZMPx,staticZMPy);
    V3_I(ZMPworPla,0) = staticZMPx;
    V3_I(ZMPworPla,1) = staticZMPy;
    V3_I(ZMPworPla,2) = 0.0;

    ChppRobotMotionSample motionsample;
    motionsample.rootpose = attStandingRobot->robot()->rootJoint()->currentTransformation();
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
        jointConfig = subrange(attStandingRobot->robot()->currentConfiguration(), 6, attStandingRobot->robot()->numberDof());
        attStandingRobot->updateRobot( attStandingRobot->robot()->rootJoint()->currentTransformation(),jointConfig, attSamplingPeriod);
        ZMPworObs = attStandingRobot->robot()->zeroMomentumPoint();
        zmpInWaist( ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla);

        /*
        if (!curSupportPolygon->isPointInsideSafeZone(V3_I(ZMPworObs,0), V3_I(ZMPworObs,1)))
            atLeastOneZMPUnsafe = true;
        */
        
        motionsample.configuration = attStandingRobot->robot()->currentConfiguration();
        motionsample.velocity = attStandingRobot->robot()->currentVelocity();
        motionsample.acceleration = attStandingRobot->robot()->currentAcceleration();
        motionsample.ZMPwstPla = ZMPwstPla;
        motionsample.ZMPwstObs = ZMPwstObs;
        motionsample.ZMPworPla = ZMPworPla;
        motionsample.ZMPworObs = ZMPworObs;
        
        attSolutionMotion->appendSample(motionsample);
    }

    /*
    if (atLeastOneZMPUnsafe)
        std::cout << "ChppGikHandTask::solve(): ZMP goes out of support polygon\n";
    */

    //return !atLeastOneZMPUnsafe;
    return true;
}


ChppGikHandTask::~ChppGikHandTask()
{
    cleanUp();
}

void ChppGikHandTask::zmpInWaist(const vector3d& inZMPworPla, const vector3d& inZMPworObs, vector3d& outZMPwstObs, vector3d& outZMPwstPla)
{
    tempM4 =attStandingRobot->robot()->waist()->currentTransformation();
    MAL_S4x4_INVERSE(tempM4,tempInv,double);
    MAL_S4x4_C_eq_A_by_B(outZMPwstObs,tempInv,inZMPworObs);
    MAL_S4x4_C_eq_A_by_B(outZMPwstPla,tempInv,inZMPworPla);
}
