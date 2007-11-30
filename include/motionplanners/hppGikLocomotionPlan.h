#ifndef HPP_GIK_LOCOMOTION_PLAN_H
#define HPP_GIK_LOCOMOTION_PLAN_H

#include "tasks/hppGikGenericTaskElements.h"
#include "robot/hppGikStandingRobot.h"
#include "core/hppGikMotionPlan.h"
#include "motionplanners/hppGikPreviewController.h"

/**
\brief This is a task plan for locomotion tasks (ChppGikLocomotionElement).
At anytime, there is a foot which is considered attached to the ground (in other terms non slippery contact with infinite grip). We define hereby a nonsupport foot as a foot that is not considered attached to the ground.
This locomotion task plan is transformed into two motion constaints: the COM and the the nonsupport foot motion constraints which are inseted in the global motion plan (ChppGikMotionPlan)
Each locomotion task knows how to plan for a stable Zero momentum Point motion and a foot motion for itself. The produced chunks of motions are stored by this object.
*/
class ChppGikLocomotionPlan
{
public:

    /**
    \name Construction
    {@
     */
    /**
    \brief Constructor.
     */
    ChppGikLocomotionPlan(ChppGikMotionPlan* inAssociatedMotionPlan, ChppGikStandingRobot* inStandingRobot, double inStartTime, double inSamplingPeriod);
    
    /**
    \brief Add a locomotion task.
    The task addition fails if the time frame taken by the added task overlaps a previously added task
     */
    bool addTask(ChppGikLocomotionElement* inStep);
    
    /**
    \brief Get the end time
     */
    double endTime();
    
    /**
    \brief Clear task stack
    */
    void clearTasks();
    
    /**
    \brief Add a static duration at the end of the planned motion
    */
    void extendEnd(double inDuration);
    
    /**
    @}
    */

    /**
    \name Computations
    {@
     */
    
    /**
    \brief Solve for support polygon motion, non support foot motion and COM motion. Returned motions start at attStartTime - PreviewControl->previewTime().
     */
    bool solveOneStage();
    
    /**
    \brief Get a pointer to the computed support polygon motion
     */
    ChppGikSupportPolygonMotion* supportPolygonMotion();
    
    /**
    \brief Get the default support foot joint at the given time
     */
    CjrlJoint* supportFootJoint(double inTime);
    
    /**
    \brief Get an appropriate joints weighting vecotr according to current task.
    */
    bool weightsAtTime(double inTime, vectorN& outWeights);
    
    /**
    \brief Get the currently active task if any.
    \return null pointer if no locomotion task is active at the current time
    */
    const ChppGikLocomotionElement* activeTask(double inTime) const;
    
    /**
    \brief Get planned ZMP motion. The returned matrix has 3 rows
     */
    const matrixNxP& plannedZMPmotion();

    /**
    @}
     */

    /**
    \brief Destructor
     */
    ~ChppGikLocomotionPlan();

private:
    /**
    \brief Return a pointer to a new instance of footprint based on the given transformation. Doesn't check whether the rotation is a yaw transformation of world frame.
     */
    ChppGikFootprint*  createFootprintFromTransformation(const matrix4d& inTransformation, bool& isAnkleHeightOK);

    /**
    \brief Clear previously-computed motions
    */
    bool reset(double inStartTime);
    
    /**
    \brief add COM and foot motions references in the associated motion plan 
    */
    void addtoMotionPlan();

    /**
    \brief Build motions by calling all tasks' planning method
    */
    bool buildMotions();

    /**
    \brief Prolongate last planned motion values
     */
    bool prolongate(double inDuration);
    /**
    \brief Prolongate last planned ZMP
     */
    void prolongateZMP(double inDuration);

    /**
    \brief Associated robot
     */
    ChppGikStandingRobot* attStandingRobot;
    
    /**
    \brief Associated motion plan
    */
    ChppGikMotionPlan* attAssociatedMotionPlan;

    /**
    \brief The vector of pointers to tasks (not stored in this object)
    */
    std::vector<ChppGikLocomotionElement*> attTasks;

    /**
    \brief Start time
     */
    double attStartTime;
    
    /**
    \brief End time
     */
    double attTasksEndTime;
    
    /**
    \brief Extra end time
     */
    double attExtraEndTime;
    
    /**
    \brief End time of zmp trajectory
     */
    double attExtraZMPEndTime;
    
    /**
    \brief Sampling period
     */
    double attSamplingPeriod;
    
    /**
    \brief Support polygon motion
     */
    ChppGikSupportPolygonMotion* attSupportPolygonMotion;
    
    /**
    \brief The planned ZMP motion
     */
    matrixNxP attPlannedZMP;
    
    /**
    \brief The corrected ZMP motion
     */
    matrixNxP attCorrectedZMP;
    
    /**
    \brief Motion constraint on the CoM.
     */
    ChppGikMotionConstraint* attComMotion;
    

    ChppGikMotionPlanRow* attComMotionPlanRow;
    
    ChppGikMotionPlanRow* attFootMotionPlanRow;
    

    /**
    \brief Motion constraint on nonsupport foot
     */
    ChppGikMotionConstraint* attFootMotion;
    
    /**
    \brief The Preview Controller that comuptes com motion
     */
    ChppGikPreviewController* attPreviewController;
};

#endif
