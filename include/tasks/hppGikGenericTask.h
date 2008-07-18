#ifndef HPP_GIK_GENERICTASK_H
#define HPP_GIK_GENERICTASK_H

#include "motionplanners/hppGikLocomotionPlan.h"
#include "tasks/hppGikRobotTask.h"
#include "core/hppGikSolverRobotAttached.h"
#include "core/hppGikPrioritizedMotion.h"

/**
\brief This object produces whole body joint motion based on entered "elements".These elements are cartesian motion planners of class ChppGikPrioritizedMotion or class ChppGikLocomotionElement.
Special attention must be paid when adding ChppGikLocomotionElement objects:
<br>- Their time frames should not be overlapping, otherwise the overlapping Element is discarded
<br>- The first ChppGikLocomotionElement will generate 1.6 seconds of motion before its actual start time, the last ChppGikLocomotionElement will generate 1.0 seconds of motion after its end. This is due to the preview controller. For example: the user enters a ChppGikStepElement that starts at time 0.0s and finishes at time 2.0s. Then the motion time scale will begin at time -1.6s to end at time 3.0s.
<br>The first 1.6 second duration cannot be changed, but the latter 1.0s can be modified (with caution) through method ChppGikGenericTask::extraEndTime(). This extra 1.0 second is used to allow the center of mass of the robot to finish its motion.
<br>
<br>
Example of situation where forgetting these time extensions lead to unsolved problems (due to ZMP out of Support Polygon notifications):
<br>Initial configuration is HalfSitting, the waist of the robot is horizontal, parallel to the flat ground. The user adds a ChppGikPrioritizedMotion to maintain the waist horizontality between time 0.0 and time 2.0. he also adds a ChppGikStepElement to make a step between time 0.0 and time 2.0.
<br>
Calling method ChppGikGenericTask::solve() will likely provoke "ZMP out" at time 0.0. 
This is because the center of mass starts moving due to the Preview Controller between times -1.6s and 0.0 such that at time 0.0, a discontinous constraint on the waist horizontality is suddenly added in the stack of tasks, and provokes large accelerations, and "ZMP out". Moreover, after time 2.0s, the constraint on the waist is no longer included in the stack of tasks, and while the center of mass continues to move during the extra end time of 1.0second, the constraint to be maintained for the waist does not hold anymore at the end the motion (time 3.0s).<br>The correct way to program such a scenario would be to set a waist ChppGikPrioritizedMotion between time 0.0 and time 4.6s, and to set the ChppGikStepElement to begin at time 1.6s and end at time 1.6+2.0=3.6s.

\ingroup motionsolver
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
