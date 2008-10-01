#include <vector>
#include <time.h>
#include <sys/time.h>
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/io.hpp"
#include "tasks/hppGikGenericTask.h"
#include "motionplanners/elements/hppGikInterpolatedElement.h"
#include "hppGikTools.h"

#define V3_I MAL_S3_VECTOR_ACCESS

using namespace ublas;

ChppGikGenericTask::ChppGikGenericTask(ChppGikStandingRobot* inStandingRobot,  double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod, "GenericTask")
{
    attRobot = inStandingRobot->robot();

    attMotionPlan = new ChppGikMotionPlan(attRobot);

    attLocomotionPlan = new ChppGikLocomotionPlan( attMotionPlan, attStandingRobot, attSamplingPeriod);

    attGikSolver = new ChppGikSolverRobotAttached(*attRobot);

    attUseDynamicWeights = true;
    attNeutralBodyOption = false;
    attUserDefinedMask = false;
    attExtraEndTime = 1.0;
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

bool ChppGikGenericTask::algorithmSolve()
{
    unsigned int rank;
    bool ok,atLeastOneZMPUnsafe = false;
    double time, motionStartTime, motionEndTime, gapTime;
    CjrlJoint* supportJoint = 0;
    vectorN gikWeights(attRobot->numberDof()-6);
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

    //     std::cout << "motion startTime " << attMotionPlan->startTime() <<"\n";
    //     std::cout << "motion end  Time " << attMotionPlan->endTime() <<"\n";

    while (time < motionEndTime)
    {
        //std::cout << "Time now         " << time <<"\n";
        attRobot->clearFixedJoints();
        supportJoint = attLocomotionPlan->supportFootJoint(time);
        attRobot->addFixedJoint(supportJoint);

        columnInTime = attMotionPlan->columnAtTime(time);
        constraintStack = columnInTime->constraints();
        computeGikWeights(time, columnInTime->workingJoints(), gikWeights);
        attGikSolver->weights(gikWeights);

        supportJoint->computeJacobianJointWrtConfig();
        for (unsigned int i = 0; i< constraintStack.size();i++)
        {
            constraintStack[i]->computeValue();
            constraintStack[i]->computeJacobian();
        }
        ok = attGikSolver->solve(constraintStack);

        if (!ok)
        {
            std::cout <<"ChppGikGenericTask::gradientStep() Could not solve motion plan at time "<< time << "\n";
            return false;
        }
        attGikSolver->applySolution();

        attLocomotionPlan->getZMPAtTime(time, uZMPworPla);
        ChppGikTools::UblastoVector3(uZMPworPla, ZMPworPla);
        attStandingRobot->updateDynamics(attSamplingPeriod, ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla);
        attSolutionMotion->appendSample(attStandingRobot->robot()->currentConfiguration(),ZMPwstPla,ZMPwstObs,ZMPworPla,ZMPworObs);

        curSupportPolygon = attStandingRobot->supportPolygon();
        if (!curSupportPolygon->isPointInside(V3_I(ZMPworObs,0), V3_I(ZMPworObs,1)))
        {
            std::cout << "ZMP: " << V3_I(ZMPworObs,0) <<" , "<< V3_I(ZMPworObs,1) <<" out of polygon at time " << time << "\n";
            curSupportPolygon->print();
            atLeastOneZMPUnsafe = true;
        }

        time += attSamplingPeriod;
        rank++;
    }

    ok = !atLeastOneZMPUnsafe;
    return ok;
}

void ChppGikGenericTask::computeGikWeights(double inTime, const vectorN& inActiveJoints, vectorN& outGikWeights)
{
    //GIK weights
    if (attUseDynamicWeights)
        attLocomotionPlan->getWeightsAtTime( inTime, outGikWeights );
    else
        outGikWeights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    if (attNeutralBodyOption)
        for (unsigned int i=0; i< outGikWeights.size();i++)
            if (attStandingRobot->maskFactory()->legsMask()(i) == 0)
                outGikWeights(i) = 1;
    //Joints mask
    if (attUserDefinedMask)
        jointsMask = attUserJointsMask;
    else
        jointsMask = inActiveJoints;
    //Combine
    for (unsigned int i=0;i<jointsMask.size();i++)
        outGikWeights(i) *= jointsMask(i);
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
        if (inMask->size() != (attRobot->numberDof()-6))
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
