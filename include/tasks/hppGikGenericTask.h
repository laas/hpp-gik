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
    bool addElement(ChppGikGenericTaskElement* inElement);

    /**
    \brief add a ready motion constraint
     */
    bool addReadyMotionElement(ChppGikReadyMotionElement* inElement);

    /**
    \brief Activate/disactivate different inverse kinematics weights according to support polygon type (single or double). Default is Enabled.
     */
    void dynamicWeights(bool inSwitch);
    
    /**
    \brief True = privelege to lighter joints in upper body. False = no privelege. Default is False
     */
    void neutralUpperBody(bool inSwitch);
    
    
    /**
    \brief Activate/disactivate motion replanning when a time sample cannot be solved. Default is Disabled.
     */
    void motionReplanning(bool inSwitch);
    
    
    /**
    \brief Return the number of stored elements
     */
    unsigned int numberElements();
    
    /**
    \brief Return a pointer to the element at the requested rank. Null pointer is returned in case rank is out of bounds
     */
    ChppGikGenericTaskElement* elementAtRank(unsigned int inRank);
    
    /**
    \brief Clear all the entered elements
     */
    void clearElements();
  
    /**
    \brief Destructor
     */
    ~ChppGikGenericTask();

protected:
    CjrlHumanoidDynamicRobot* attRobot;

    ChppGikMotionPlan* attMotionPlan;

    ChppGikLocomotionPlan* attLocomotionPlan;

    ChppGikSingleMotionsPlan* attSingleMotionsPlan;
    
    ChppGikSolver* attGikSolver;

    void computeGikWeights(double inTime, vectorN& outGikWeights);
    
private:

    virtual bool algorithmSolve();
    
    /**
    \brief Remember the pointers to the entered ready motion constraints
    */
    std::vector<ChppGikReadyMotionElement*> attReadyMotionElements;
    /**
    \brief Remember the pointers to the motions plan rows holding the references to the entered ready motion constraints
    */
    std::vector<ChppGikMotionPlanRow*> attReadyMotionElementsRows;
    
    /**
    \brief Vector of pointers to the generic task elements.
     */
    std::vector<ChppGikGenericTaskElement*> attPlanningTasks;
    
    /**
    \brief Dynamic weights option
    */
    bool attUseDynamicWeights;
    
    /**
    \brief Neutral body option
     */
    bool attNeutralBodyOption;
    
    /**
    \brief Motion replanning option
     */
    bool attEnableReplanning;
    
    /**
    \brief computation vector
    */
    vectorN jointsMask;
};
#endif
