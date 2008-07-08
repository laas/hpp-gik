#ifndef HPP_GIK_GENERICTASK_H
#define HPP_GIK_GENERICTASK_H

#include "motionplanners/hppGikLocomotionPlan.h"
#include "tasks/hppGikRobotTask.h"
#include "core/hppGikSolverRobotAttached.h"
#include "core/hppGikPrioritizedMotion.h"

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
    \brief add a prioritized motion (like ChppGikInterpolatedElement or ChppGikReadyElement)
     */
    bool addElement(ChppGikPrioritizedMotion* inElement);
    
    /**
    \brief add a locomotion
     */
    bool addElement(ChppGikLocomotionElement* inElement);

    /**
    \brief Clear all the entered elements
     */
    void clearElements();


    /**
    \brief modify extra end time (use with caution, make sure there is at least one second of planned motion after the endtime of the last planned stepElement)
     */
    void extraEndTime(double inDuration);

    /**
    \brief Activate/disactivate different inverse kinematics weights according to support polygon type (single or double). Default is Enabled.
     */
    void dynamicWeights(bool inSwitch);

    /**
    \brief True = privelege to lighter joints in upper body. False = no privelege.
    \note Default value is False
     */
    void neutralUpperBody(bool inSwitch);

    /**
    \brief True = use the given joints mask. False = automatically computed joints mask.
    \note Default is False
     */
    void automaticJointsMask(bool inSwitch, const  vectorN* inMask = 0);


    /**
    \brief Destructor
     */
    ~ChppGikGenericTask();

protected:
    CjrlHumanoidDynamicRobot* attRobot;
    ChppGikMotionPlan* attMotionPlan;
    ChppGikLocomotionPlan* attLocomotionPlan;
    ChppGikSolverRobotAttached* attGikSolver;
    void computeGikWeights(double inTime, const vectorN& inActiveJoints, vectorN& outGikWeights);

private:
    virtual bool algorithmSolve();
    std::vector<ChppGikMotionPlanRow*> attPrioritizedMotionRows;
    std::vector<ChppGikPrioritizedMotion*> attPrioritizedMotions;
    bool attUseDynamicWeights;
    bool attNeutralBodyOption;
    vectorN jointsMask;
    bool attUserDefinedMask;
    vectorN attUserJointsMask;
    double attExtraEndTime;
};
#endif
