#include "tasks/hppGikConfigurationTask.h"
#include "hppGikTools.h"

#define V3_I MAL_S3_VECTOR_ACCESS


ChppGikConfigurationTask::ChppGikConfigurationTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, const CjrlRobotConfiguration& inTargetConfiguration):ChppGikRobotTask(inStandingRobot,inSamplingPeriod, "ConfigurationTask")
{
    attTargetConfiguration = inTargetConfiguration;
    attMotionDuration = 4.0;
    attEpilogueDuration = 1.0;
}

CjrlRobotConfiguration& ChppGikConfigurationTask::targetConfiguration()
{
    return attTargetConfiguration;
}

void ChppGikConfigurationTask::motionDuration(double inMotionDuration)
{

    attMotionDuration = inMotionDuration;
}

void ChppGikConfigurationTask::epilogueDuration(double inDuration)
{

    attEpilogueDuration = inDuration;
}

void ChppGikConfigurationTask::targetConfiguration(const CjrlRobotConfiguration& inTargetConfiguration)
{

    attTargetConfiguration = inTargetConfiguration;
}

bool ChppGikConfigurationTask::algorithmSolve()
{
    ChppGikSupportPolygon* curSupportPolygon = attStandingRobot->supportPolygon();
    if (!curSupportPolygon->isDoubleSupport())
    {
        std::cout << "ChppGikConfigurationTask::solve() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    vectorN targetConfiguration = attStandingRobot->robot()->currentConfiguration();
    for (unsigned int i= 6; i<attStandingRobot->robot()->numberDof();i++)
    {
        if (attStandingRobot->maskFactory()->upperBodyMask()(i-6) == 1)
        {
            if (attTargetConfiguration(i) > attStandingRobot->robot()->upperBoundDof(i) || attTargetConfiguration(i) < attStandingRobot->robot()->lowerBoundDof(i))
            {
                std::cout << "ChppGikConfigurationTask::solve(): target configuration violates joint limits\n";
                return false;
            }
            targetConfiguration(i) = attTargetConfiguration(i);
        }
    }

    unsigned int extraSamples = ChppGikTools::timetoRank(0.0,attEpilogueDuration, attSamplingPeriod);
    unsigned int nSamples = ChppGikTools::timetoRank(0.0,attMotionDuration, attSamplingPeriod)+1;
    matrixNxP interpolationMatrix(attStandingRobot->robot()->numberDof(), nSamples);
    vectorN interpolationVector(nSamples);

    for (unsigned int i=0; i<attStandingRobot->robot()->numberDof(); i++)
    {
        bool contn = ChppGikTools::minJerkCurve( attMotionDuration,attSamplingPeriod,attStandingRobot->robot()->currentConfiguration()(i),0.0,0.0,targetConfiguration(i),interpolationVector);
        if (!contn)
            return false;
        row(interpolationMatrix,i) = interpolationVector;
    }

    bool atLeastOneZMPUnsafe = false;
    vector3d ZMPwstPla,ZMPwstObs,ZMPworPla,ZMPworObs;

    for (unsigned int i=0; i<nSamples; i++)
    {
        attStandingRobot->robot()->applyConfiguration(column(interpolationMatrix,i));
        ZMPworPla = attStandingRobot->robot()->positionCenterOfMass();
        attStandingRobot->updateDynamics(attSamplingPeriod, ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla);
	attStandingRobot->robot()->SaveCurrentStateAsPastState();
        if (!curSupportPolygon->isPointInside(V3_I(ZMPworObs,0), V3_I(ZMPworObs,1)))
            atLeastOneZMPUnsafe = true;
        attSolutionMotion->appendSample(column(interpolationMatrix,i), ZMPwstPla, ZMPwstObs, ZMPworPla, ZMPworObs);
    }
    
    for (unsigned int i=0; i<extraSamples; i++)
    {
        attStandingRobot->robot()->applyConfiguration(column(interpolationMatrix,nSamples-1));
        attStandingRobot->updateDynamics(attSamplingPeriod, ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla);
	attStandingRobot->robot()->SaveCurrentStateAsPastState();
        if (!curSupportPolygon->isPointInside(V3_I(ZMPworObs,0), V3_I(ZMPworObs,1)))
            atLeastOneZMPUnsafe = true;
        attSolutionMotion->appendSample(column(interpolationMatrix,nSamples-1), ZMPwstPla, ZMPwstObs, ZMPworPla, ZMPworObs);
    }
    
    if (atLeastOneZMPUnsafe)
        std::cout << "ChppGikConfigurationTask::solve(): ZMP goes out of support polygon\n";

    return !atLeastOneZMPUnsafe;
}

ChppGikConfigurationTask::~ChppGikConfigurationTask()
{
    cleanUp();
}
