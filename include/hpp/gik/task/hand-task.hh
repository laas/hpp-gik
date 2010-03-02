#ifndef HPP_GIK_HAND_TASK_H
#define HPP_GIK_HAND_TASK_H

#include "hpp/gik/task/robot-task.hh"

/**
\brief This is a robot hand task. It can be used to plan joint motion to change the clench of a hand. The clench is represented by a single scalar value ranging from 0 (open hand) to 1 (closed hand).
Joint motion is a slow minimum jerk motion from current to target clench.
\ingroup tasks
 */
class ChppGikHandTask : public ChppGikRobotTask
{
public:
    /**
        \brief constructor
     */
    ChppGikHandTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod);

    /**
        \brief Set the hand. true = right hand, false = left
     */
    void forRightHand(bool inSwitch);

    /**
        \brief get the target clench value
     */
    double targetClench();

    /**
        \brief Set the target clench value
     */
    void targetClench(double inTargetClenchValue);

    /**
        \brief Set the motion duration
     */
    void motionDuration(double inMotionDuration);

    /**
        \brief Get the motion duration
     */
    double motionDuration();

    void bringBackZMP(bool inChoice, double inStartTime, double inDuration) {}
    
    /**
        \brief Destructor
     */
    ~ChppGikHandTask();

protected:
    
    virtual bool algorithmSolve();

private:

    double attTargetClenchValue;
    double attMotionDuration;
    CjrlHand* attHand;
    void zmpInWaist(const vector3d& inZMPworPla, const vector3d& inZMPworObs, vector3d& outZMPwstObs, vector3d& outZMPwstPla);
    matrix4d  tempM4,tempInv; 

};
#endif
