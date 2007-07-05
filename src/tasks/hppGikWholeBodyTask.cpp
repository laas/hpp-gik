#include <vector>
#include <string>
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"

#include "tasks/hppGikWholeBodyTask.h"
#include "constraints/hppGikPositionConstraint.h"
#include "constraints/hppGikParallelConstraint.h" 
#include "constraints/hppGikTransformationConstraint.h"
#include "hppGikTools.h"


ChppGikWholeBodyTask::ChppGikWholeBodyTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, unsigned int inMaxIterations):ChppGikRobotTask(inStandingRobot,inSamplingPeriod,"WholebodyTask")
{
    attMaxIterations = inMaxIterations;
    
    attEnableStep = true;
    
    attGenericTask = new ChppGikGenericTask(inStandingRobot, inSamplingPeriod);
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
    //delete user motion constraint stack
    std::vector<ChppGikPrioritizedMotionConstraint*>::iterator iterM;
    for( iterM = attUserMotionTasks.begin(); iterM != attUserMotionTasks.end(); iterM++)
        delete *iterM;
    attUserMotionTasks.clear();
}

void ChppGikWholeBodyTask::clear()
{
    attGenericTask->clearElements();
    attSolutionMotion->clear();
}


void ChppGikWholeBodyTask::addStateConstraint(CjrlGikStateConstraint* inStateConstraint, unsigned int inPriority)
{
    ChppGikPrioritizedStateConstraint* stateTask = new ChppGikPrioritizedStateConstraint(inStateConstraint, inPriority);
    attUserStateTasks.push_back(stateTask);
}


void ChppGikWholeBodyTask::addMotionConstraint(CjrlGikMotionConstraint* inMotionConstraint, unsigned int inPriority)
{
    ChppGikPrioritizedMotionConstraint* readyMo = new ChppGikPrioritizedMotionConstraint(inMotionConstraint, inPriority);
    attUserMotionTasks.push_back(readyMo);
}

void ChppGikWholeBodyTask::enableStepping(bool inEnabled)
{
    attEnableStep = inEnabled;
}

bool ChppGikWholeBodyTask::algorithmSolve()
{

    if (attUserStateTasks.empty() && attUserMotionTasks.empty())
    {
        std::cout << "ChppGikWholeBodyTask::solve() no constraints entered by the user\n";
        return false;
    }


    //try to solve the problem without stepping
    bool isSolved = basicSolve();
    if (!isSolved && attEnableStep)
    {
        restoreRobot();
        isSolved = onestepSolve();//(auto resets planners and motion plan)
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

    //get initial support polygon
    if (!attStandingRobot->supportPolygon()->isDoubleSupport())
    {
        std::cout << "ChppGikWholeBodyTask::basicSolve() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    double targetX, targetY, distance,centerX,centerY;

    attStandingRobot->supportPolygon()->center(centerX,centerY);

    furthestTargetProjection(centerX,centerY,targetX, targetY, distance);

    if (distance > 0.7)
        return false;

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
        ChppGikSingleMotionElementConstraint* taskconstraint= dynamic_cast<ChppGikSingleMotionElementConstraint*>((*iter)->stateConstraint());

        if (taskconstraint != 0)
        {
            ChppGikSingleMotionElement* task = new ChppGikSingleMotionElement( taskconstraint,(*iter)->priority(),defaultStartTime,defaultTaskDuration);
            attGenericTask->addElement(task);
        }
        else
            std::cout << "An entered state constraint is ignored for not being of class ChppGikSingleMotionElementConstraint\n";
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

    double zmpstartshifttime = 0.2;//0.1;
    double zmpendshifttime = 0.4;//0.1;
    double footflighttime = 0.9;//0.7;
    double zmpendcoef = 0.70;

    double otherTasksStartTime = stepStartTime + 2.2;
    double otherTasksDuration = 4.0;

    bool isSolved =false;
    ChppGikStepElement* stepTask = 0;
    
    ChppGikSingleMotionElement* waistTask = 0;
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
        stepTask = new ChppGikStepElement(stepStartTime, vectorFootprints[i], whichFoot, zmpendcoef, zmpendshifttime, zmpstartshifttime, footflighttime);

        attGenericTask->addElement(stepTask);

        //waistTask = new ChppGikSingleMotionElement(waistConstraint, 4, 0.0, otherTasksStartTime+otherTasksDuration);
        //attGenericTask->addElement(waistTask);

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
    double maxDist = 0;
    vectorN target(3);
    bool check = false;
    double temp;


    std::vector<ChppGikPrioritizedStateConstraint*>::iterator iter;
    for( iter = attUserStateTasks.begin(); iter != attUserStateTasks.end(); iter++)
    {
        ChppGikPositionConstraint* posConstr= dynamic_cast<ChppGikPositionConstraint*>((*iter)->stateConstraint());
        check = (posConstr != 0);
        if (check)
        {
            target = posConstr->worldTargetU();
        }
        else
        {
            ChppGikTransformationConstraint* transConstr= dynamic_cast<ChppGikTransformationConstraint*>((*iter)->stateConstraint());
            check = (transConstr != 0);
            if (check)
            {
                target = transConstr->worldTargetU();
            }
        }
        if (check)
        {
            outDistance = sqrt(pow((target(0)-centerX),2)+pow((target(1)-centerY),2));
            if ( outDistance > maxDist)
            {
                maxDist = temp;
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
    std::vector<ChppGikPrioritizedMotionConstraint*>::iterator iterM;
    for( iterM = attUserMotionTasks.begin(); iterM != attUserMotionTasks.end(); iterM++)
        attGenericTask->addReadyMotionElement(*iterM);

    return attGenericTask->solve();
}


ChppGikWholeBodyTask::~ChppGikWholeBodyTask()
{
    reset();
    delete attGenericTask;
}
