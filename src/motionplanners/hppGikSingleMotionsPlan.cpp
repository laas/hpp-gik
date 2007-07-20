#include "motionplanners/hppGikSingleMotionsPlan.h"
#include "hppGikTools.h"


ChppGikSingleMotionsPlan::ChppGikSingleMotionsPlan(ChppGikMotionPlan* inAssociatedMotionPlan, double inStartTime, double inSamplingPeriod)
{
    attStartTime = inStartTime;
    attEndTime = inStartTime;
    attSamplingPeriod = inSamplingPeriod;
    attAssociatedMotionPlan =  inAssociatedMotionPlan;
}


bool ChppGikSingleMotionsPlan::addTask(ChppGikSingleMotionElement* inTask)
{
    CjrlJoint* taskJoint = inTask->targetConstraint()->joint();
    CjrlDynamicRobot& robot = inTask->targetConstraint()->robot();
    CjrlHumanoidDynamicRobot* humanoid;
    humanoid = dynamic_cast<CjrlHumanoidDynamicRobot *>(&robot);
    if (humanoid
	&& (taskJoint == humanoid->leftFoot() 
	    || taskJoint == humanoid->rightFoot()))
    {
        std::cout << "ChppGikSingleMotionsPlan::addTask() Entered task should not be on the feet joints\n";
        return false;
    }
    attTasks.push_back(inTask);
    if (inTask->endTime()>attEndTime)
        attEndTime = inTask->endTime();
    return true;
}

std::vector<const ChppGikSingleMotionElement*> ChppGikSingleMotionsPlan::activeTasks(double inTime) const
{
    std::vector<const ChppGikSingleMotionElement*> retVec;
    double epsilon = attSamplingPeriod/2;

    std::vector<ChppGikSingleMotionElement*>::const_iterator ppTask;
    for( ppTask = attTasks.begin(); ppTask != attTasks.end(); ppTask++)
    {
        if ( ( (*ppTask)->startTime() < inTime +epsilon ) && ( (*ppTask)->endTime() >= inTime - epsilon ) )
            retVec.push_back(*ppTask);
    }
    return retVec;
}

double ChppGikSingleMotionsPlan::endTime()
{
    return attEndTime;
}

void ChppGikSingleMotionsPlan::clearTasks()
{
    attTasks.clear();
    attEndTime = attStartTime;
}

bool ChppGikSingleMotionsPlan::updateMotionPlan(double inTime)
{
    std::vector<ChppGikSingleMotionElement*>::iterator ppTask;
    double epsilon = attSamplingPeriod/2;
    bool ok;
    for( ppTask = attTasks.begin(); ppTask != attTasks.end(); ppTask++)
        //detect that a task has to be planned now
        if (((*ppTask)->startTime() >= inTime-epsilon) && ((*ppTask)->startTime() < inTime + epsilon))
        {
            ok = (*ppTask)->planMotion(attSamplingPeriod);
            if (!ok)
            {
                std::cout << "ChppGikSingleMotionsPlan::updateMotionPlan() failed to solve a motion planning task\n";
                return false;
            }

            ok = (*ppTask)->referMotion(attAssociatedMotionPlan);
            if (!ok)
            {
                std::cout << "ChppGikSingleMotionsPlan::updateMotionPlan() failed to add motion to motion plan\n";
                return false;
            }
        }
    return true;
}

bool ChppGikSingleMotionsPlan::replan(std::vector<unsigned int>& inRowsToReplan, double inCurrentTime)
{
    std::vector<ChppGikSingleMotionElement*>::iterator ppTask;
    unsigned int counter = 0;
    unsigned int rownumber;
    for( ppTask = attTasks.begin(); ppTask != attTasks.end(); ppTask++)
    {
        bool priorityexists = attAssociatedMotionPlan->rankPriority((*ppTask)->priority(), rownumber);
        if (!priorityexists)
            continue;

        bool replan = false;
        for(unsigned int i=0; i< inRowsToReplan.size(); i++)
            if (rownumber == inRowsToReplan[i])
            {
                replan = true;
                break;
            }

        if (replan)
        {
            std::cout <<"replanning task "<< counter <<" at time "<< inCurrentTime << "\n";
            (*ppTask)->dereferMotion();
            double startTime = inCurrentTime + attSamplingPeriod;
            (*ppTask)->startTime(startTime);
        }
        counter ++;
    }
}

ChppGikSingleMotionsPlan::~ChppGikSingleMotionsPlan()
{}
