#include <vector>
#include <time.h>
#include <sys/time.h>
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/io.hpp"
#include "tasks/hppGikGenericTask.h"
#include "hppGikTools.h"

#define V3_I MAL_S3_VECTOR_ACCESS

using namespace ublas;

ChppGikGenericTask::ChppGikGenericTask(ChppGikStandingRobot* inStandingRobot,  double inSamplingPeriod):ChppGikRobotTask(inStandingRobot,inSamplingPeriod, "GenericTask")
{
    attRobot = inStandingRobot->robot();

    attMotionPlan = new ChppGikMotionPlan(attRobot);

    attLocomotionPlan = new ChppGikLocomotionPlan(attMotionPlan, attStandingRobot, attStartTime, attSamplingPeriod);

    attSingleMotionsPlan = new ChppGikSingleMotionsPlan(attMotionPlan, attStartTime, attSamplingPeriod);

    attGikSolver = new ChppGikSolver(attRobot);

    attUseDynamicWeights = true;
    attEnableReplanning = false;

}


bool ChppGikGenericTask::addElement(ChppGikGenericTaskElement* inTask)
{

    if (inTask->startTime() < attStartTime-attSamplingPeriod/2)
    {
        std::cout <<"ChppGikGenericTask::addElement(): Entered task had an invalid start time ( "<< inTask->startTime() <<" )\n";
        return false;
    }

    //record the entered task for later deletion
    attPlanningTasks.push_back(inTask);

    ChppGikLocomotionElement* locomotionTask = dynamic_cast<ChppGikLocomotionElement*>(inTask);
    if (locomotionTask != 0)
    {
        bool ok = attLocomotionPlan->addTask( locomotionTask );
        if (ok)
            if (locomotionTask->workingJoints().size() == 0)
                //assign default working joints
                locomotionTask->workingJoints(attStandingRobot->maskFactory()->legsMask());
        return ok;
    }

    ChppGikSingleMotionElement* singlemotionTask = dynamic_cast<ChppGikSingleMotionElement*>(inTask);
    if (singlemotionTask != 0)
    {
        bool ok = attSingleMotionsPlan->addTask( singlemotionTask );

        if (ok)
            if (singlemotionTask->workingJoints().size()==0)
                //assign default working joints
                singlemotionTask->workingJoints( attStandingRobot->maskFactory()->maskForJoint(singlemotionTask->targetConstraint()->joint()) );

        return ok;
    }

    std::cout <<"ChppGikGenericTask::addElement(): entered task type was not recognized\n";
    return false;
}

bool ChppGikGenericTask::addReadyMotionElement( ChppGikPrioritizedMotionConstraint* inTask)
{
    if (inTask->motionConstraint()->startTime() < attStartTime-attSamplingPeriod/2)
    {
        std::cout <<"ChppGikGenericTask::addReadyMotionElement(): Entered ready motion had an invalid start time\n";
        return false;
    }
    ChppGikMotionPlanRow* motionRow = attMotionPlan->addMotionConstraint(inTask->motionConstraint(),inTask->priority());
    attReadyMotionConstraintsRows.push_back(motionRow);
}

void ChppGikGenericTask::dynamicWeights(bool inSwitch)
{
    attUseDynamicWeights = inSwitch;
}

void ChppGikGenericTask::motionReplanning(bool inSwitch)
{
    attEnableReplanning = inSwitch;
}

void ChppGikGenericTask::clearElements()
{
    //clear references motion planning tasks
    attLocomotionPlan->clearTasks();
    attSingleMotionsPlan->clearTasks();

    //delete planning tasks
    std::vector<ChppGikGenericTaskElement*>::iterator iter;
    for( iter = attPlanningTasks.begin(); iter != attPlanningTasks.end(); iter++)
        delete *iter;
    attPlanningTasks.clear();

    //Clear ready motions
    for( unsigned int i=0; i< attReadyMotionConstraints.size();i++)
        attReadyMotionConstraintsRows[i]->removeMotionConstraint(attReadyMotionConstraints[i]);
    attReadyMotionConstraintsRows.clear();
    attReadyMotionConstraints.clear();

}

bool ChppGikGenericTask::algorithmSolve()
{
    vectorN gikWeights(attRobot->numberDof()-6);
    vectorN jointsMask(attRobot->numberDof()-6);

    CjrlJoint* supportJoint = 0;
    std::vector<CjrlGikStateConstraint*> constraintStack;
    std::vector<unsigned int> unsolvedRanks;

    double time, motionStartTime, motionEndTime, gapTime;
    unsigned int rank = 0;
    bool ok,quit=false;

    
    // Make sure the locomotion plan is not shorter the single motions plan
    double nonlocomotionEndtime = ( attSingleMotionsPlan->endTime() > attMotionPlan->endTime())? attSingleMotionsPlan->endTime() : attMotionPlan->endTime();
    gapTime = attLocomotionPlan->endTime() - nonlocomotionEndtime;

    if (gapTime<0)
        attLocomotionPlan->extendEnd(-gapTime);

    // Solve the locomotion tasks (implicitely deletes previously computed locomotion)
    attLocomotionPlan->solveOneStage();

    motionStartTime = attMotionPlan->startTime();

    motionEndTime = attMotionPlan->endTime();

    time = motionStartTime;

    // ZMP related
    ChppGikSupportPolygon* curSupportPolygon=0;
    bool atLeastOneZMPUnsafe = false;
    vector3d ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla;
    vectorN uZMPworPla(3);

    // Adjust solution motion start time according to planned locomotion
    attSolutionMotion->startTime(motionStartTime);

    while (time < motionEndTime+attSamplingPeriod/2)
    {
        //record the support joint we are interested in in the robot
        attRobot->clearFixedJoints();
        supportJoint = attLocomotionPlan->supportFootJoint(time);
        attRobot->addFixedJoint(supportJoint);

        //Update the motion plan with new motions triggered at this time
        ok = attSingleMotionsPlan->updateMotionPlan(time);

        if (!ok)
        {
            std::cout <<"ChppGikGenericTask::solve() Could not update motion plan at time "<< time << "\n";
            return false;
        }

        //get the stack of prioritized constraints to be solved at this time
        constraintStack = attMotionPlan->columnAtTime(time)->constraints();

        //Compute the weights used in solving the gik problem
        computeGikWeights(time, gikWeights, jointsMask);

        //Update config of the robot according to constraints (solve!)
        ok = attGikSolver->solve(constraintStack,gikWeights,jointsMask);

        ////Activate this to check the solution (values of constraints)
        //attGikSolver->getUnsolvedConstraints(constraintStack, unsolvedRanks, 1e-3);
        
        if (!ok || !unsolvedRanks.empty())
        {

            std::cout <<"ChppGikGenericTask::solve() Could not solve motion plan at time "<< time << "\n";
            return false;
        }

        //update dynamics
        uZMPworPla = column(attLocomotionPlan->plannedZMPmotion(),rank);
        ChppGikTools::UblastoVector3(uZMPworPla, ZMPworPla);

        attStandingRobot->updateDynamics(attSamplingPeriod, ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla);

        //append sample to solution motion
        attSolutionMotion->appendSample(attGikSolver->solutionConfiguration(),ZMPwstPla,ZMPwstObs,ZMPworPla,ZMPworObs);

        //check stability
        curSupportPolygon = attStandingRobot->supportPolygon();
        if (!curSupportPolygon->isPointInside(V3_I(ZMPworObs,0), V3_I(ZMPworObs,1)))
        {
            std::cout << "ZMP out of polygon at time " << time << "\n";
            atLeastOneZMPUnsafe = true;
        }

        time += attSamplingPeriod;
        rank++;
    }

    ok = !atLeastOneZMPUnsafe;

    return ok;
}

void ChppGikGenericTask::computeGikWeights(double inTime, vectorN& outGikWeights, vectorN& jointsMask)
{
    //GIK weights
    if (attUseDynamicWeights)
        attLocomotionPlan->weightsAtTime(inTime,outGikWeights);
    else
        outGikWeights = attStandingRobot->maskFactory()->weightsDoubleSupport();

    //Joints mask
    if (!attReadyMotionConstraintsRows.empty())
        jointsMask = attStandingRobot->maskFactory()->wholeBodyMask();
    else
    {
        const ChppGikLocomotionElement* currentLoco = attLocomotionPlan->activeTask(inTime);
        std::vector<const ChppGikSingleMotionElement*> activeTasks = attSingleMotionsPlan->activeTasks(inTime);
        
        if (currentLoco)
            jointsMask = currentLoco->workingJoints();
        else
            jointsMask = attStandingRobot->maskFactory()->legsMask();

        // Union set of all active tasks' working joints
        for (unsigned int i = 0; i<activeTasks.size();i++)
            attStandingRobot->maskFactory()->combineMasks(jointsMask,activeTasks[i]->workingJoints(), jointsMask);
    }
}


ChppGikGenericTask::~ChppGikGenericTask()
{
    clearElements();
    delete attLocomotionPlan;
    delete attSingleMotionsPlan;
    delete attMotionPlan;
    delete attGikSolver;
}
