#ifndef HPP_GIK_REACH_TASK_H
#define HPP_GIK_REACH_TASK_H

#include "tasks/hppGikWholeBodyTask.h"

/**
\brief Use this task to make the robot reach for a target
 
The following options are available:
- enable stepping if needed to achieve  the reach (default: enabled)
- apply a gaze constraint (default: disabled)
- apply a parallelConstraint on the okayAxis (thumb up) of the hand (default: disabled)
- go back to half sitting configuration after reaching (default: disabaled)

the reach target has to be set through method target() or the solve() algorithm will return false.

The options are not reset by method solve()
*/

class ChppGikReachTask : public ChppGikRobotTask
{
public:
    /**
        \brief constructor
     */
    ChppGikReachTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod);

    /**
        \brief set reach target (a point in world frame) and effector (a boolean indicating the hand: 'true' for 'right hand')
    */
    void target(const vector3d& inReachTarget, bool isForRightHand);
    
    /**
        \brief enable the gaze constraint and set the gaze target (a point in world frame)
     */
    void gazeTarget(const vector3d& inGazeTarget);
    
    /**
        \brief disable the gaze constraint
     */
    void disableGaze();
    
    /**
        \brief enable the hand okayAxis constraint and set the target orientation (a vector in world frame) .
     */
    void handOrientation(const vector3d& inTargetOrientation);
    
    /**
        \brief disable the hand okayAxis orientation constraint
     */
    void disableOrientation();

    /**
        \brief enable or disable stepping 
     */
    void steppingEnabled(bool inOption);

    /**
        \brief Destructor
     */
    ~ChppGikReachTask();

protected:

    virtual bool algorithmSolve();

private:

    ChppGikWholeBodyTask* attWholeBodyTask;

    vector3d attHandTargetAxis;
    
    vector3d attGazeTarget;
    
    vector3d attReachTarget;
    
    bool attEnableGaze;
    
    bool attEnableOrientation;
    
    bool attEnableStepping;
    
    bool attForRightHand;
    
    bool attTargetSet;
    
};
#endif
