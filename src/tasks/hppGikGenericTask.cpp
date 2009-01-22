#include <vector>
#include <time.h>
#include <sys/time.h>
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/io.hpp"
#include "tasks/hppGikGenericTask.h"
#include "motionplanners/elements/hppGikInterpolatedElement.h"
#include "hppGikTools.h"
#include "motionplanners/elements/hppGikZMPshiftElement.h"

#define V3_I MAL_S3_VECTOR_ACCESS

using namespace boost::numeric::ublas;

ChppGikGenericTask::ChppGikGenericTask(ChppGikStandingRobot* inStandingRobot,  double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod, "GenericTask")
{
    attRobot = inStandingRobot->robot();

    attMotionPlan = new ChppGikMotionPlan(attRobot);

    attLocomotionPlan = new ChppGikLocomotionPlan( attMotionPlan, attStandingRobot, attSamplingPeriod);

    attGikSolver = new ChppGikSolver(*attRobot);

    attUseDynamicWeights = true;
    attNeutralBodyOption = false;
    attUserDefinedMask = false;
    attExtraEndTime = 1.0;
    attBringBackZmp = false;
}

void ChppGikGenericTask::bringBackZMP(bool inChoice, double inStartTime, double inDuration)
{
    attBringBackZmp = inChoice;
    attBringStart = inStartTime;
    attBringDuration = inDuration;
}

bool ChppGikGenericTask::addElement(ChppGikPrioritizedMotion* inMotion)
{
    if (inMotion->robot() != attRobot)
    {
        std::cout<< "addElement failed -> different robot\n";
        return false;
    }

    ChppGikInterpolatedElement* interp = dynamic_cast<ChppGikInterpolatedElement*>(inMotion);
    if (interp)
    {
        ChppGikJointStateConstraint* jntc = dynamic_cast<ChppGikJointStateConstraint*>(interp->targetConstraint());
        if (jntc)
        {
            CjrlJoint* joint = jntc->joint();
            interp->workingJoints( attStandingRobot->maskFactory()->maskForJoint(joint) );
        }
    }

    attPrioritizedMotions.push_back(inMotion);
    ChppGikMotionPlanRow* motionRow = attMotionPlan->addMotion(inMotion);
    attPrioritizedMotionRows.push_back(motionRow);
    return true;
}

bool ChppGikGenericTask::addElement(ChppGikLocomotionElement* inElement)
{
    return attLocomotionPlan->addElement( inElement );
}

void ChppGikGenericTask::clearElements()
{
    //Remove readymotions from motio plan
    for( unsigned int i=0; i< attPrioritizedMotionRows.size();i++)
    {
        attPrioritizedMotionRows[i]->removeMotion(attPrioritizedMotions[i]);
        delete attPrioritizedMotions[i];
    }
    attPrioritizedMotionRows.clear();
    attPrioritizedMotions.clear();

    attLocomotionPlan->clearElements();
}

bool ChppGikGenericTask::planZMPBack()
{
    //std::cout << "ChppGikGenericTask::planZMPBack Started " << std::endl;
    ChppGikSupportPolygon *curSP = attStandingRobot->supportPolygon();
    if (!curSP)
    {
        std::cout << "ChppGikGenericTask::planZMPBack failed: support polygon not found, check your robot configuration" << std::endl;
        return false;
    }
    vector3d targetZMP = curSP->nearestCenterPointTo(attRobot->positionCenterOfMass());
    double myStartTime = attLocomotionPlan->startTime() + attBringStart;
    attBringEnd = myStartTime + attBringDuration;
    ChppGikZMPshiftElement* startshift = new ChppGikZMPshiftElement(attRobot,targetZMP,myStartTime,attBringDuration,attSamplingPeriod);
    startshift->startZMPCheck( false );
    bool inserted = addElement( startshift );
    if (!inserted)
    {
        std::cout << "ChppGikGenericTask::planZMPBack Failed: timings overlapping with the first locomotion element." << std::endl;
        return false;
    }
    //std::cout << "ChppGikGenericTask::planZMPBack OK" << std::endl;
    return true;
}
bool ChppGikGenericTask::algorithmSolve()
{
    unsigned int rank;
    bool ok,atLeastOneZMPUnsafe = false, atLeastOneZMPEccentered = false;;
    double time, motionStartTime, motionEndTime, gapTime;
    CjrlJoint* supportJoint = 0;
    vectorN gikWeights(attRobot->numberDof());
    std::vector<CjrlGikStateConstraint*> constraintStack;
    ChppGikMotionPlanColumn* columnInTime;
    vectorN uZMPworPla(3);
    ChppGikSupportPolygon* curSupportPolygon=0;
    vector3d ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla;

    attSolutionMotion->startTime(0.0);

    gapTime = attLocomotionPlan->endTime() - attMotionPlan->endTime();

    if (gapTime<0)
        attLocomotionPlan->extraEndTime(-gapTime);

    attLocomotionPlan->extraEndTime( attLocomotionPlan->extraEndTime() + attExtraEndTime );

    if (attBringBackZmp)
        planZMPBack();

    ok = attLocomotionPlan->solve();
    if (!ok)
    {
        std::cout << "Locomotion planning Failed\n";
        return false;
    }

    motionStartTime = attMotionPlan->startTime()+attSamplingPeriod;
    motionEndTime = attMotionPlan->endTime()+attEps;
    time = motionStartTime;
    rank = 1;
    
    while (time < motionEndTime)
    {
        supportJoint = attLocomotionPlan->supportFootJoint(time);
        columnInTime = attMotionPlan->columnAtTime(time);
        constraintStack = columnInTime->constraints();
        computeGikWeights(time, columnInTime->workingJoints(), gikWeights);
        
        attGikSolver->weights(gikWeights);
        attGikSolver->rootJoint(*supportJoint);
        attGikSolver->prepare( constraintStack );
        attGikSolver->solve(constraintStack);
        attGikSolver->applySolution();

        attLocomotionPlan->getZMPAtTime(time, uZMPworPla);
        ChppGikTools::UblastoVector3(uZMPworPla, ZMPworPla);
        attStandingRobot->updateDynamics(attSamplingPeriod, ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla);
        attSolutionMotion->appendSample(attStandingRobot->robot()->currentConfiguration(),ZMPwstPla,ZMPwstObs,ZMPworPla,ZMPworObs);
        
        curSupportPolygon = attStandingRobot->supportPolygon();
        if (!(curSupportPolygon->isPointInsideSafeZone(V3_I(ZMPworObs,0), V3_I(ZMPworObs,1))))
        {
            if (!(attStandingRobot->isPointInsideSupportPolygon(V3_I(ZMPworObs,0), V3_I(ZMPworObs,1),0.005)))
            {
                std::cout << "BAD ZMP: "<< V3_I(ZMPworObs,0) <<" , "<< V3_I(ZMPworObs,1) <<" at time " << time << "\n";
                curSupportPolygon->print();
                atLeastOneZMPUnsafe = true;
            }
            else
                atLeastOneZMPEccentered = true;
        }

        time += attSamplingPeriod;
        rank++;
    }

    ok = !atLeastOneZMPUnsafe;
    
    if (ok && atLeastOneZMPEccentered)
        std::cout << "Eccentered ZMP were detected during motion" << std::endl;
    
    return ok;
}

void ChppGikGenericTask::computeGikWeights(double inTime, const vectorN& inActiveJoints, vectorN& outGikWeights)
{
    unsigned int i;
    //GIK weights
    if (attUseDynamicWeights)
        attLocomotionPlan->getWeightsAtTime( inTime, outGikWeights );
    else
        outGikWeights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    if (attNeutralBodyOption)
        for (i=0; i< outGikWeights.size();i++)
            if (attStandingRobot->maskFactory()->legsMask()(i) == 0)
                outGikWeights(i) = 1;
    //Joints mask
    if (attUserDefinedMask)
        jointsMask = attUserJointsMask;
    else
        jointsMask = inActiveJoints;
    //Combine
    for (i=0;i<jointsMask.size();i++)
        outGikWeights(i) *= jointsMask(i);
    //disable freeflyer
    for (i=0;i<6;i++)
        outGikWeights(i) = 0.0;
}


void ChppGikGenericTask::dynamicWeights(bool inSwitch)
{
    attUseDynamicWeights = inSwitch;
}

void ChppGikGenericTask::neutralUpperBody(bool inSwitch)
{
    attNeutralBodyOption = inSwitch;
}

void ChppGikGenericTask::automaticJointsMask(bool inSwitch, const  vectorN* inMask)
{
    attUserDefinedMask = !inSwitch;
    if (attUserDefinedMask)
    {
        if (!inMask)
        {
            std::cout << "ChppGikGenericTask::automaticJointsMask() second argument was a null pointer \n";
            attUserDefinedMask = false;
            return;
        }
        if (inMask->size() != (attRobot->numberDof()))
        {
            std::cout << "ChppGikGenericTask::automaticJointsMask() invalid joints mask  \n";
            attUserDefinedMask = false;
            return;
        }
        attUserJointsMask = *inMask;
    }
}

void ChppGikGenericTask::extraEndTime(double inDuration)
{
    attExtraEndTime = (inDuration<0.0)?0.0:inDuration;
}

ChppGikGenericTask::~ChppGikGenericTask()
{
    clearElements();
    delete attLocomotionPlan;
    delete attMotionPlan;
    delete attGikSolver;
    cleanUp();
}
