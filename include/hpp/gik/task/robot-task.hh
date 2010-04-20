#ifndef HPP_GIK_ROBOT_TASK_H
#define HPP_GIK_ROBOT_TASK_H

#include <time.h>
#include <sys/time.h>
#include <string>
#include "hpp/gik/robot/standing-robot.hh"
#include "hpp/gik/robot/robot-motion.hh"

class ChppGikRobotTask;
/**
\brief Abstract class of an object that produces a ChppRobotMotion through a method ChppGikRobotTask::solve()
\ingroup tasks
*/
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
    ChppRobotMotion& solutionMotion() const;
    
    /**
    \brief Option to bring back the ZMP before any other motion (Disabled by default).
    A motion is planned to bring back the ZMP to the closest point on the line segment defining the center of the safe zone. The start time and the duration of this ZMP motion is left up to the user. The start time is defined relatively to the start of the locomotion plan. Example: bringBackZmp(-0.5, 2.0 ) will make the zmp planning start 0.5 seconds before the first locomotion element (if none, absolute time 0.0s), and take 2.0 seconds to finish.
    USE AT YOUR OWN RISK
     */
    virtual void bringBackZMP(bool inChoice, double inStartTime, double inDuration) = 0;

    /**
    \brief destructor
    */
    virtual ~ChppGikRobotTask() {}


protected:
    /**
    \brief Delete objects created in this instance. Meant to be called by derived classes'destructors
    */
    void cleanUp();
            
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
    \brief Motion sampling period and its half
    */
    double attSamplingPeriod, attEps;

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
