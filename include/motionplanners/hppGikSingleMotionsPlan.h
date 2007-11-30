#ifndef HPP_GIK_SINGLE_MOTIONS_PLAN_H
#define HPP_GIK_SINGLE_MOTIONS_PLAN_H

#include <vector>
#include "tasks/hppGikGenericTaskElements.h"
#include "core/hppGikMotionPlan.h"



/**
\brief This is a task plan for tasks that produce a single motion constraint (ChppGikSingleMotionElement).
SolveAtTime(inTime) find tasks whose resolution are scheduled to start at inTime.For each of these tasks, the motion planning method is called, and the reference to the created motion is added in the global motion plan (ChppGikMotionPlan). On reset() or destruction, this object cleans all references it put in the global motion plan.
 */
class ChppGikSingleMotionsPlan
{

public:
    /**
    \brief constructor
     */
    ChppGikSingleMotionsPlan(ChppGikMotionPlan* inAssociatedMotionPlan, double inStartTime, double inSamplingPeriod);

    /**
    \brief add a single motion task
    \return false if task concerns a foot joint
     */
    bool addTask(ChppGikSingleMotionElement* inTask);

    /**
    \brief  Get the highest end time in the task stack
     */
    double endTime();

    /**
    \brief Clear task vector
     */
    void clearTasks();
    
    /**
    \brief Plan motion constraints fot tasks trtiggered at time inTime.
    The triggering happens when a task's start time is equal(modulus sampling period) to inTime.
    The motion constraint is built by calling an interpolation method embedded in the state constraint that defines a task.
    \return false if the planning of a motion fails
    */
    bool updateMotionPlan(double inTime);
    
    /**
    \brief Replan motions stored in the rows indicated by the given ranks
    \return false if the planning of a motion fails 
    */
    bool replan(std::vector<unsigned int>& inRowsToReplan, double inCurrentTime);

    /**
    \brief Get a vector of pointers to active tasks for the given time
     */
    std::vector<const ChppGikSingleMotionElement*> activeTasks(double inTime) const;
    
    /**
    \brief Destructor
     */
    ~ChppGikSingleMotionsPlan();

private:

    /**
    \brief Associated motion plan
     */
    ChppGikMotionPlan* attAssociatedMotionPlan;

    std::vector<ChppGikSingleMotionElement*> attTasks;

    double attStartTime;
    
    double attEndTime;

    double attSamplingPeriod, attEps;

};
#endif
