#ifndef HPP_GIK_ROBOT_TASK_H
#define HPP_GIK_ROBOT_TASK_H

#include <time.h>
#include <sys/time.h>
#include <string>
#include "robot/hppGikStandingRobot.h"
#include "robot/hppRobotMotion.h"

class ChppGikRobotTask;
class ChppGikRobotTask
{
public:

    /**
    \brief Constructor
    */
    ChppGikRobotTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, const char* inTaskName);

    /**
    \brief Solve for joint motion. Note that this method should return true in the case the task is already solved. It is up to the user to verify that the retrieved solutionMotion() is not an empty motion.
    */
    bool solve();

    /**
    \brief show the resolution time
    */
    void showResolutionTime(bool inSwitch);
    
    /**
    \brief Get the computed solution motion. Can be empty.
    */
    const ChppRobotMotion& solutionMotion() const;

    /**
    \brief destructor
    */
    ~ChppGikRobotTask();


protected:
    /**
    \brief The algorithm implemented by solve()
    */
    virtual bool algorithmSolve() =0;
            
    /**
    \brief Backup current robot configuration (q, dot q, ddot q)
    */
    void backupRobot();

    /**
    \brief Restore saved robot configuration (q, dot q, ddot q) and compute forward kinematics
     */
    void restoreRobot();
    
    /**
    \brief concatanate a task solution motion to the current attSolutionMotion and apply the last solution configuration to the robot.
    */
    void cropMotion(ChppGikRobotTask* inRobotTask);
    
    /**
    \brief Associated robot
    */
    ChppGikStandingRobot* attStandingRobot;

    /**
    \brief Motion sampling period
    */
    double attSamplingPeriod;

    /**
    \brief Start time
     */
    double attStartTime;
    
    /**
    \brief option to show resolution time
    */
    bool attShowTime;

    /**
    \brief The solution joint motion
    */
    ChppRobotMotion* attSolutionMotion;
    
    /**
    \brief This task's name
    */
    std::string attTaskName;

    /**
    \brief Backup of robot configuration before task solving
    {@
     */
    CjrlRobotConfiguration attInitialConfiguration;

    /**
    \brief timimg
    {@
    */
    struct timeval *Tps, *Tpf;
    struct timezone *Tzp;
    /**
    @}
     */

};

#endif
