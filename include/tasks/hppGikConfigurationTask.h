#ifndef HPP_GIK_CONFIGURATION_TASK_H
#define HPP_GIK_CONFIGURATION_TASK_H

#include "tasks/hppGikRobotTask.h"

/**
\brief This is a robot task. It can be used to plan joint motion to bring the robotat a desired joint configuration. For safety, only upper body joints can be moved through this task, although the constructor expects a full configuration specification.
Joint motion is a slow minimum jerk motion from current configuration to target.
 */
class ChppGikConfigurationTask : public ChppGikRobotTask
{
public:
    /**
    \brief constructor
    */
    ChppGikConfigurationTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, const CjrlRobotConfiguration& inTargetConfiguration);

    /**
    \brief Set the motion duration
    */
    void motionDuration(double inMotionDuration);
    
    /**
    \brief Set the duration of motion extension (after the interpolation is finished, the motion duration is extended by this value)
    */
    void epilogueDuration(double inDuration);

    /**
    \brief Get the motion duration
     */
    double motionDuration();

    /**
    \brief get the target configuration
     */
    CjrlRobotConfiguration& targetConfiguration();

    /**
    \brief Set the target configuration
     */
    void targetConfiguration(const CjrlRobotConfiguration& inTargetConfiguration);

    /**
    \brief Destructor
     */
    ~ChppGikConfigurationTask();

protected:
    /**
    \brief Solve for joint motion from the current configuration to the specified target configuration. Only upper body joints are enabled.
    */
    virtual bool algorithmSolve();
    
private :
    /**
    \brief Target configuration
    */
    CjrlRobotConfiguration attTargetConfiguration;

    /**
    \brief Motion duration and epilogue
    */
    double attMotionDuration, attEpilogueDuration;



};
#endif
