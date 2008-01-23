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
            {
                //assign default working joints
                singlemotionTask->workingJoints( attStandingRobot->maskFactory()->maskForJoint(singlemotionTask->targetConstraint()->joint()) );
            }

        return ok;
    }

    std::cout <<"ChppGikGenericTask::addElement(): entered task type was not recognized\n";
    return false;
}

bool ChppGikGenericTask::addReadyMotionElement( ChppGikReadyMotionElement* inTask)
{
    if (inTask->motionConstraint()->startTime() < attStartTime-attSamplingPeriod/2)
    {
        std::cout <<"ChppGikGenericTask::addReadyMotionElement(): Entered ready motion had an invalid start time\n";
        return false;
    }

    attReadyMotionElements.push_back(inTask);

    ChppGikMotionPlanRow* motionRow = attMotionPlan->addMotionConstraint(inTask->motionConstraint(),inTask->priority());

    attReadyMotionElementsRows.push_back(motionRow);
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
    for( unsigned int i=0; i< attReadyMotionElementsRows.size();i++)
        attReadyMotionElementsRows[i]->removeMotionConstraint(attReadyMotionElements[i]->motionConstraint());
    attReadyMotionElementsRows.clear();

    //delete ready motions
    std::vector<ChppGikReadyMotionElement*>::iterator iterM;
    for( iterM = attReadyMotionElements.begin(); iterM != attReadyMotionElements.end(); iterM++)
        delete *iterM;
    attReadyMotionElements.clear();

}

unsigned int ChppGikGenericTask::numberElements()
{
    return attPlanningTasks.size();
}

ChppGikGenericTaskElement* ChppGikGenericTask::elementAtRank(unsigned int inRank)
{
    if (inRank < attPlanningTasks.size())
        return attPlanningTasks[inRank];
    else
        return NULL;
}

bool ChppGikGenericTask::algorithmSolve()
{
    vectorN gikWeights(attRobot->numberDof()-6);
    CjrlJoint* supportJoint = 0;
    std::vector<CjrlGikStateConstraint*> constraintStack;

    double time, motionStartTime, motionEndTime, gapTime;
    unsigned int rank = 0;
    bool ok,quit=false;

    // ZMP related
    ChppGikSupportPolygon* curSupportPolygon=0;
    bool atLeastOneZMPUnsafe = false;
    vector3d ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla;
    vectorN uZMPworPla(3);

    // Make sure the locomotion plan is not shorter the single motions plan
    double nonlocomotionEndtime = ( attSingleMotionsPlan->endTime() > attMotionPlan->endTime())? attSingleMotionsPlan->endTime() : attMotionPlan->endTime();
    gapTime = attLocomotionPlan->endTime() - nonlocomotionEndtime;

    if (gapTime<0)
        attLocomotionPlan->extendEnd(-gapTime);

    // Solve the locomotion tasks (implicitely deletes previously computed locomotion)
    bool okloco = attLocomotionPlan->solveOneStage();
    if (!okloco)
    {
        std::cout << "An error occured while planning.Aborting.\n";
        return false;
    }

    motionStartTime = attMotionPlan->startTime();

    motionEndTime = attMotionPlan->endTime() + attSamplingPeriod/2;

    // Adjust solution motion start time according to planned locomotion
    attSolutionMotion->startTime(motionStartTime);

    time = motionStartTime;
    rank = 0;

    //Plan start motion
    ok = attSingleMotionsPlan->updateMotionPlan(time);
    if (!ok)
    {
        std::cout <<"ChppGikGenericTask::solve() Could not update motion plan at time "<< time << "\n";
        return false;
    }

    time += attSamplingPeriod;
    rank++;

    while (time < motionEndTime)
    {
        //record the support joint we are interested in in the robot
        attRobot->clearFixedJoints();
        supportJoint = attLocomotionPlan->supportFootJoint(time);
        attRobot->addFixedJoint(supportJoint);

        //get the stack of prioritized constraints to be solved at this time
        constraintStack = attMotionPlan->columnAtTime(time)->constraints();

        //Compute the weights used in solving the gik problem
        computeGikWeights(time, gikWeights);

        //set the weights in the gik solver
        attGikSolver->weights(gikWeights);

        //compute foot(fixed joint) jacobian
        supportJoint->computeJacobianJointWrtConfig();

        //compute hpp constaints values and jacobians (these need the previous support joint jacobian computation)
        for (unsigned int i = 0; i< constraintStack.size();i++)
        {
            constraintStack[i]->computeValue();
            constraintStack[i]->computeJacobian();
        }

        attGikSolver->accountForJointLimits();
        //Update config of the robot according to constraints (solve!)
        ok = attGikSolver->gradientStep(constraintStack);

        if (!ok)
        {

            std::cout <<"ChppGikGenericTask::gradientStep() Could not solve motion plan at time "<< time << "\n";
            return false;
        }

        //update dynamics
        uZMPworPla = column(attLocomotionPlan->plannedZMPmotion(),rank);
        ChppGikTools::UblastoVector3(uZMPworPla, ZMPworPla);

        attStandingRobot->updateDynamics(attSamplingPeriod, ZMPworPla, ZMPworObs, ZMPwstObs, ZMPwstPla);

        attStandingRobot->robot()->SaveCurrentStateAsPastState();

        //append sample to solution motion
        attSolutionMotion->appendSample(attStandingRobot->robot()->currentConfiguration(),ZMPwstPla,ZMPwstObs,ZMPworPla,ZMPworObs);

        //check stability
        curSupportPolygon = attStandingRobot->supportPolygon();
        if (!curSupportPolygon->isPointInside(V3_I(ZMPworObs,0), V3_I(ZMPworObs,1)))
        {
            std::cout << "ZMP: " << V3_I(ZMPworObs,0) <<" , "<< V3_I(ZMPworObs,1) <<" out of polygon at time " << time << "\n";
            curSupportPolygon->print();
            atLeastOneZMPUnsafe = true;
        }

        //Update the motion plan
        ok = attSingleMotionsPlan->updateMotionPlan(time);

        if (!ok)
        {
            std::cout <<"ChppGikGenericTask::solve() Could not update motion plan at time "<< time << "\n";
            return false;
        }

        time += attSamplingPeriod;
        rank++;
    }


    ok = !atLeastOneZMPUnsafe;

    return ok;
}

void ChppGikGenericTask::computeGikWeights(double inTime, vectorN& outGikWeights)
{
    //GIK weights
    if (attUseDynamicWeights)
        attLocomotionPlan->weightsAtTime(inTime,outGikWeights);
    else
        outGikWeights = attStandingRobot->maskFactory()->weightsDoubleSupport();

    //Joints mask

    const ChppGikLocomotionElement* currentLoco = attLocomotionPlan->activeTask(inTime);
    std::vector<const ChppGikSingleMotionElement*> activeTasks = attSingleMotionsPlan->activeTasks(inTime);

    if (currentLoco)
        jointsMask = currentLoco->workingJoints();
    else
        jointsMask = attStandingRobot->maskFactory()->legsMask();

    // Union set of all active tasks' working joints
    for (unsigned int i = 0; i<activeTasks.size();i++)
        attStandingRobot->maskFactory()->combineMasks(jointsMask,activeTasks[i]->workingJoints(), jointsMask);
    
    // working joints from ready motion elements
    for (unsigned int i = 0; i<attReadyMotionElements.size();i++)
    {
        CjrlGikMotionConstraint *mc = attReadyMotionElements[i]->motionConstraint();
        if ( ( mc->startTime() < inTime - attEps ) && ( mc->endTime() >= inTime - attEps ) )
            attStandingRobot->maskFactory()->combineMasks(jointsMask,attReadyMotionElements[i]->workingJoints(), jointsMask);
    }

    for (unsigned int i=0;i<jointsMask.size();i++)
        outGikWeights(i) *= jointsMask(i);
}


ChppGikGenericTask::~ChppGikGenericTask()
{
    clearElements();
    delete attLocomotionPlan;
    delete attSingleMotionsPlan;
    delete attMotionPlan;
    delete attGikSolver;
    cleanUp();
}
