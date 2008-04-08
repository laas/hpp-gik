#include <vector>
#include <string>
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"

#include "tasks/hppGikWholeBodyTask.h"
#include "motionplanners/elements/hppGikInterpolatedElement.h"
#include "motionplanners/elements/hppGikStepElement.h"
#include "constraints/hppGikPositionConstraint.h"
#include "constraints/hppGikParallelConstraint.h"
#include "constraints/hppGikTransformationConstraint.h"
#include "hppGikTools.h"


ChppGikWholeBodyTask::ChppGikWholeBodyTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, unsigned int inMaxIterations, ChppGikGenericTask *inGenericTask):ChppGikRobotTask(inStandingRobot,inSamplingPeriod,"WholebodyTask")
{
    attMaxIterations = inMaxIterations;

    attEnableStep = true;

    attHadToStep = false;

    if (inGenericTask)
    {
        attGenericTask = inGenericTask;
    }
    else
    {
        attGenericTask = new ChppGikGenericTask(inStandingRobot, inSamplingPeriod);
    }
}

CjrlHumanoidDynamicRobot& ChppGikWholeBodyTask::robot()const
{
    return *(attStandingRobot->robot());
}


void ChppGikWholeBodyTask::reset()
{
    //delete user state constraint stack
    std::vector<ChppGikPrioritizedStateConstraint*>::iterator iter;
    for( iter = attUserStateTasks.begin(); iter != attUserStateTasks.end(); iter++)
        delete *iter;
    attUserStateTasks.clear();
    attUserMotionTasks.clear();
}

void ChppGikWholeBodyTask::clear()
{
    attGenericTask->clearElements();
}


void ChppGikWholeBodyTask::addStateConstraint(CjrlGikStateConstraint* inStateConstraint, unsigned int inPriority)
{
    ChppGikPrioritizedStateConstraint* stateTask = new ChppGikPrioritizedStateConstraint(inStateConstraint, inPriority);
    attUserStateTasks.push_back(stateTask);
}


void ChppGikWholeBodyTask::addMotionConstraint(CjrlGikMotionConstraint* inMotionConstraint, unsigned int inPriority)
{
    ChppGikReadyElement* readyMo = new ChppGikReadyElement( attStandingRobot->robot(), inMotionConstraint, inPriority, attStandingRobot->maskFactory()->wholeBodyMask());
    attUserMotionTasks.push_back(readyMo);
}

void ChppGikWholeBodyTask::enableStepping(bool inEnabled)
{
    attEnableStep = inEnabled;
}

bool ChppGikWholeBodyTask::hadToStep()
{
    return attHadToStep;
}

bool ChppGikWholeBodyTask::algorithmSolve()
{

    if (attUserStateTasks.empty() && attUserMotionTasks.empty())
    {
        std::cout << "ChppGikWholeBodyTask::solve() no constraints entered by the user\n";
        return false;
    }

    attHadToStep = false;

    bool tryBasic = true;
    bool isSolved = false;

    if (attEnableStep)
    {
        double targetX, targetY, distance,centerX,centerY;

        attStandingRobot->supportPolygon()->center ( centerX,centerY );

        furthestTargetProjection ( centerX,centerY,targetX, targetY, distance );

        if ( distance > 0.7 )
            tryBasic =  false;
    }

    if (tryBasic)
    {
        //try to solve the problem without stepping
        isSolved = basicSolve();
    }

    if (!isSolved && attEnableStep)
    {
        attHadToStep = true;
        restoreRobot();
        isSolved = onestepSolve();
    }

    cropMotion( attGenericTask );

    return isSolved;
}


bool ChppGikWholeBodyTask::basicSolve()
{
    //(stupid) ALGORITHM
    //assign a start time and an end time to every user-entered prioritized state constraint

    //Clean task plan
    clear();

    std::cout << "Trying to solve without making a step.\n";

    //get initial support polygon
    if (!attStandingRobot->supportPolygon()->isDoubleSupport())
    {
        std::cout << "ChppGikWholeBodyTask::basicSolve() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    double defaultStartTime = attStartTime;
    double defaultTaskDuration = 4.0; //in seconds

    defaultPlannerTaskMaker(defaultStartTime,defaultTaskDuration);

    return executeResolutionPlan();
}

void ChppGikWholeBodyTask::defaultPlannerTaskMaker(double defaultStartTime,double defaultTaskDuration)
{
    std::vector<ChppGikPrioritizedStateConstraint*>::iterator iter;
    for( iter = attUserStateTasks.begin(); iter != attUserStateTasks.end(); iter++)
    {
        ChppGikVectorizableConstraint* taskconstraint= dynamic_cast<ChppGikVectorizableConstraint*>((*iter)->stateConstraint());

        if (taskconstraint != 0)
        {
            ChppGikInterpolatedElement* task = new ChppGikInterpolatedElement( attStandingRobot->robot(), taskconstraint, (*iter)->priority(), defaultStartTime, defaultTaskDuration, attSamplingPeriod);
            attGenericTask->addElement(task);
        }
        else
            std::cout << "An entered state constraint is ignored for not being of class CjrlGikJointStateConstraint\n";
    }
}

bool ChppGikWholeBodyTask::onestepSolve()
{
    //(stupid) ALGORITHM
    // look for position constraint tasks in the user-entered (either transformation or position for now)
    // determin the furthest position from the vertical axis between the footprints
    // determin which foot to move
    // compute footprint candidates
    // go through footprint candidates until a solution motion is met or all footprints unsuccessful

    std::cout << "Trying to solve with a step.\n";

    //get initial support polygon
    if (!attStandingRobot->supportPolygon()->isDoubleSupport())
    {
        std::cout << "ChppGikWholeBodyTask::onestepSolve() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    double centerX, centerY, targetX, targetY, distance;

    attStandingRobot->supportPolygon()->center(centerX,centerY);

    furthestTargetProjection(centerX,centerY,targetX, targetY, distance);

    //Determin moving foot
    bool whichFoot = true;
    const ChppGikFootprint* movingFootprint = 0;
    const ChppGikFootprint* rFootprint = attStandingRobot->supportPolygon()->rightFootprint();



    double iprod = ChppGikTools::inner_prod_3points(centerX,centerY,rFootprint->x(),rFootprint->y(),targetX,targetY);
    if (iprod >=0)
    {
        std::cout << "Note from task planner: I had to step right\n";
        movingFootprint = rFootprint;
    }
    else
    {
        std::cout << "Note from task planner: I had to step left\n";
        movingFootprint = attStandingRobot->supportPolygon()->leftFootprint();
        whichFoot = false;
    }

    //Create footprint candidates
    std::vector<ChppGikFootprint*> vectorFootprints;
    createFootprintCandidates(movingFootprint, targetX, targetY, vectorFootprints);

    //Write solving loop
    double stepStartTime = attStartTime+1.7;

    double zmpstartshifttime = 0.02;//0.1;
    double zmpendshifttime = 0.1;//0.1;
    double footflighttime = 0.8;//0.7;
    double zmpendcoef = 0.70;

    attGenericTask->dynamicWeights(false);

    double otherTasksStartTime = stepStartTime + 1.8;
    double otherTasksDuration = 4.0;

    bool isSolved =false;
    ChppGikStepElement* stepTask = 0;

    ChppGikInterpolatedElement* waistTask = 0;
    vector3d targetOrientation,laxis;
    laxis[0] = 0;
    laxis[1] = 0;
    laxis[2] = 1;
    targetOrientation = laxis;
    ChppGikParallelConstraint* waistConstraint = new ChppGikParallelConstraint(*(attStandingRobot->robot()),*(attStandingRobot->robot()->waist()),laxis,targetOrientation);

    for (unsigned int i=0;i<attMaxIterations;i++)
    {
        //Clean task plan
        clear();

        //Create step task (default timings)
        stepTask = new ChppGikStepElement( attStandingRobot->robot(), stepStartTime, vectorFootprints[i], whichFoot, attSamplingPeriod , zmpendcoef,  zmpendshifttime, zmpstartshifttime, footflighttime);

        attGenericTask->addElement(stepTask);

        waistTask = new ChppGikInterpolatedElement(attStandingRobot->robot(),waistConstraint, 4, 0.0, otherTasksStartTime+otherTasksDuration, attSamplingPeriod);
        attGenericTask->addElement(waistTask);

        //wrap & time user-entered state constraints, and add them to attPlannerTasks vector
        defaultPlannerTaskMaker(otherTasksStartTime,otherTasksDuration);

        //solve
        isSolved = executeResolutionPlan();

        if (isSolved)
            break;
    }

    //Delete footprint candidates
    deleteFootprintCandidates(vectorFootprints);
    delete waistConstraint;
    return isSolved;
}

void ChppGikWholeBodyTask::furthestTargetProjection(double centerX, double centerY,double& outX, double& outY, double& outDistance)
{
    outDistance = 0.0;
    outX = centerX;
    outY = centerY;

    double maxDist = 0.0;
    vectorN target(3), vectorizedTarget(6);
    bool check = false;
    double temp;


    std::vector<ChppGikPrioritizedStateConstraint*>::iterator iter;
    for( iter = attUserStateTasks.begin(); iter != attUserStateTasks.end(); iter++)
    {
        ChppGikPositionConstraint* posConstr= dynamic_cast<ChppGikPositionConstraint*>((*iter)->stateConstraint());
        check = (posConstr != 0);
        if (check)
        {
            posConstr->computeVectorizedTarget();
            target = ((ChppGikVectorizableConstraint*)posConstr)->vectorizedTarget();
        }
        else
        {
            ChppGikTransformationConstraint* transConstr= dynamic_cast<ChppGikTransformationConstraint*>((*iter)->stateConstraint());
            check = (transConstr != 0);
            if (check)
            {
                transConstr->computeVectorizedTarget();
                vectorizedTarget = ((ChppGikVectorizableConstraint*)transConstr)->vectorizedTarget();
                target = subrange(vectorizedTarget,0,3);
            }
        }
        if (check)
        {
            temp = sqrt(pow((target(0)-centerX),2)+pow((target(1)-centerY),2));
            if ( temp > maxDist)
            {
                maxDist = temp;
                outDistance = temp;
                outX = target(0);
                outY = target(1);
            }
        }
        check = false;
    }
}

void ChppGikWholeBodyTask::createFootprintCandidates(const ChppGikFootprint* startFootprint,double targetX, double targetY, std::vector<ChppGikFootprint*>& inVectorFootprints )
{
    double travelLimit = 0.20; //should be safe enough
    double rotateLimit = M_PI / 4; //safe enough ? too safe ?

    double gapX = targetX - startFootprint->x();
    double gapY = targetY - startFootprint->y();
    double cosVal =   cos(startFootprint->th()) * gapX + sin( startFootprint->th()) * gapY;
    double sinVal = cos(startFootprint->th()) * gapY - sin( startFootprint->th())*gapX;
    double gapTheta = atan2(sinVal,cosVal);

    //Thresholding target footprint
    double gap = sqrt(gapX*gapX+gapY*gapY);
    if  ( gap > travelLimit)
    {
        double frac = travelLimit/gap;
        gapX *= frac;
        gapY *= frac;
    }
    if  (gapTheta > rotateLimit)
        gapTheta = rotateLimit;

    if  (gapTheta < -rotateLimit)
        gapTheta = -rotateLimit;


    for (unsigned int i=0;i<attMaxIterations;i++)
    {
        double f = (i+1)/attMaxIterations;
        ChppGikFootprint* newFp = new ChppGikFootprint(startFootprint->x()+f*gapX,startFootprint->y()+f*gapY,startFootprint->th()+f*gapTheta);
        inVectorFootprints.push_back(newFp);
    }
}


void ChppGikWholeBodyTask::deleteFootprintCandidates(std::vector<ChppGikFootprint*>& inVectorFootprints)
{
    std::vector<ChppGikFootprint*>::iterator iter;
    for( iter = inVectorFootprints.begin(); iter != inVectorFootprints.end(); iter++)
        delete *iter;
    inVectorFootprints.clear();
}


bool ChppGikWholeBodyTask::executeResolutionPlan()
{
    //Add ready motions
    std::vector<ChppGikReadyElement*>::iterator iterM;
    for( iterM = attUserMotionTasks.begin(); iterM != attUserMotionTasks.end(); iterM++)
        attGenericTask->addElement(*iterM);

    return attGenericTask->solve();
}


ChppGikWholeBodyTask::~ChppGikWholeBodyTask()
{
    reset();
    delete attGenericTask;
    cleanUp();
}
