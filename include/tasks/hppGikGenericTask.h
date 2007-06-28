#ifndef HPP_GIK_GENERICTASK_H
#define HPP_GIK_GENERICTASK_H

#include "motionplanners/hppGikLocomotionPlan.h"
#include "motionplanners/hppGikSingleMotionsPlan.h"
#include "tasks/hppGikRobotTask.h"
#include "core/hppGikSolver.h"

/**
\brief The ChppGikGenericTask is a robot task composed of generic task elements and ready motions.
*/
class ChppGikGenericTask: public ChppGikRobotTask
{
public :
    /**
    \brief constructor
    */
    ChppGikGenericTask(ChppGikStandingRobot* inStandingRobot,  double inSamplingPeriod);

    /**
    \brief add a motion planning task: single motion task or locomotion task expected, and the start time of the task must be greater than the start time passed to the constructor of this task plan.
     */
    bool addElement(ChppGikGenericTaskElement* inTask);

    /**
    \brief add a ready motion constraint
     */
    bool addReadyMotionElement(ChppGikPrioritizedMotionConstraint* inTask);

    /**
    \brief Activate/disactivate different inverse kinematics weights according to support polygon type (single or double). Default is Enabled.
     */
    void dynamicWeights(bool inSwitch);
    
    /**
    \brief Activate/disactivate motion replanning when a time sample cannot be solved. Default is Disabled.
     */
    void motionReplanning(bool inSwitch);
    
    /**
    \brief Clear all the entered elements
     */
    void clearElements();
  
    /**
    \brief Destructor
     */
    ~ChppGikGenericTask();

private:

    virtual bool algorithmSolve();
    
    void computeGikWeights(double inTime, vectorN& outGikWeights, vectorN& jointsMask);
    
    CjrlHumanoidDynamicRobot* attRobot;

    ChppGikMotionPlan* attMotionPlan;

    ChppGikLocomotionPlan* attLocomotionPlan;

    ChppGikSingleMotionsPlan* attSingleMotionsPlan;
    
    ChppGikSolver* attGikSolver;

    /**
    \brief Remember the pointers to the entered ready motion constraints
    */
    std::vector<ChppGikMotionConstraint*> attReadyMotionConstraints;
    /**
    \brief Remember the pointers to the motions plan rows holding the references to the entered ready motion constraints
    */
    std::vector<ChppGikMotionPlanRow*> attReadyMotionConstraintsRows;
    
    /**
    \brief Vector of pointers to the generic task elements.
     */
    std::vector<ChppGikGenericTaskElement*> attPlanningTasks;
    
    /**
    \brief Dynamic weights option
    */
    bool attUseDynamicWeights;
    
    /**
    \brief Motion replanning option
     */
    bool attEnableReplanning;
};
#endif
