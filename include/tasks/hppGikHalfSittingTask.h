#ifndef HPP_GIK_HALFSITTING_TASK_H
#define HPP_GIK_HALFSITTING_TASK_H

#include "tasks/hppGikConfigurationTask.h"
#include "tasks/hppGikStepBackTask.h"

/**
\brief This robot task consists in going back to half-sitting stance.
 */
class ChppGikHalfSittingTask : public ChppGikRobotTask
{

public:

    /**
    \brief constructor
     */
    ChppGikHalfSittingTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod);
    
    /**
    \brief Manual mode
     */
    void automaticFoot(bool mode = true, bool footIsRight = true );

    /**
    \brief Destructor
     */
    virtual ~ChppGikHalfSittingTask();

protected :
    /**
    \brief Plan joint motion to bring the robot to half sitting stance
    Underlying Algorithm:
    Determin if there is a need to make a step back (in which case the front foot is set next to the other)
    Place the waist
    Do a slow minimum jerk interpolation on the upper body
    Prequisite: Double support polygon
         */
    virtual bool algorithmSolve();

    /**
    \brief Convenience function which automates the creation of transformation tasks from transformations
     */
    ChppGikSingleMotionElement* extractTransformationTask(const matrix4d& refBase, const matrix4d& refTarget, const matrix4d& nowBase, CjrlJoint* joint, double startTime,double  duration, unsigned int priority );

    /**
    \brief Half sitting configuration
     */
    CjrlRobotConfiguration attHalfSitting;

    /**
    \brief StepBack task
     */
    ChppGikStepBackTask* attStepBackTask;

    /**
    \brief The solution task plan
     */
    ChppGikGenericTask* attGenericTask;

    /**
    \brief Upper body interpolation task
     */
    ChppGikConfigurationTask* attUpperBodyTask;

};
#endif

