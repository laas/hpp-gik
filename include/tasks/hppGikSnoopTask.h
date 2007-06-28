#ifndef HPP_GIK_SNOOP_TASK_H
#define HPP_GIK_SNOOP_TASK_H

#include "tasks/hppGikGenericTask.h"

/**
\brief This is a task to make the robot look somewhere whith its head being at a certain height
 */

class ChppGikSnoopTask : public ChppGikRobotTask
{
public:
    /**
        \brief constructor
     */
    ChppGikSnoopTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, double inHeadHeight, double inHeadAdvancement, const vector3d& inGazeTarget);
    
    /**
        \brief set the gaze target (a point in world frame)
    */
    void gazeTarget(const vector3d& inGazeTarget);
    
    /**
        \brief Set the desired x coordinate of the gaze origin in the support polygon frame. The support polygon frame has its origin on the middle of the ankles projections on the ground. The x axis points ahead.
     */
    void headAdvancement(double inValue);
    
    /**
        \brief set the desired height for the head in world frame
    */
    void headHeight(double inValue);

    /**
        \brief Destructor
     */
    ~ChppGikSnoopTask();

protected:

    virtual bool algorithmSolve();
    
private:

    /**
        \brief The solution task plan
     */
    ChppGikGenericTask* attGenericTask;

    /**
        \brief The target height for the head
    */
    double attHeadHeight;
    
    /**
        \brief This is the desired x coordinate of the origin of gaze defined in the current support polygon frame
    */
    double attHeadAdvancement;

    /**
        \brief The target point for the gaze vector
    */
    vector3d attGazeTarget;

};
#endif
