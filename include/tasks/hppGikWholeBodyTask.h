#ifndef HPP_GIK_WHOLEBODY_TASK_H
#define HPP_GIK_WHOLEBODY_TASK_H

#include  <vector>
#include "tasks/hppGikGenericTask.h"
#include "constraints/hppGikPrioritizedStateConstraint.h"
#include "motionplanners/elements/hppGikReadyElement.h"

/**
\brief Implementation of the Humanoid2006 papers by E.Yoshida.
\ingroup tasks
 */
class ChppGikWholeBodyTask: public ChppGikRobotTask
{
public:

    /**
    \brief Constructor
     */
    ChppGikWholeBodyTask(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod, unsigned int inMaxIterations =1, ChppGikGenericTask *inGenericTask=NULL);

    /**
    \brief Get associated robot
    */
    CjrlHumanoidDynamicRobot& robot()const;

    /**
    \brief Add a prioritized state constraint. The state constraint is transformed by the implemented algorithms into motion subtasks which are added to the whole body motion task.
     */
    void addStateConstraint(CjrlGikStateConstraint* inStateConstraint, unsigned int inPriority);

    /**
    \brief Add a prioritized motion constraint. The motion constraint is inserted as-is to the whole body motion task. Simultaneous motion subtasks might be generated by the implemented algorithms.
     */
    void addMotionConstraint(CjrlGikMotionConstraint* inMotionConstraint, unsigned int inPriority);

    /**
    \brief Tell if last solution computation needed making a step
    */
    bool hadToStep();
    
    /**
    \brief enable stepping if a solution could not be found with the current support polygon.Default value upon construction is True.
    */
    void enableStepping(bool inEnabled);

    /**
    \brief Reset the planner by clearing entered constraints (state and motion)
     */
    void reset();

    /**
    \brief Destructor
     */
    ~ChppGikWholeBodyTask();

protected:
    /**
    \brief Compute a motion complying with the constraints contained in whole body motion task.
     */
    virtual bool algorithmSolve();

    /**
    \brief Clear data related to previous resolution
     */
    void clear();

    /**
    \brief Execute the task resolution plan
     */
    bool executeResolutionPlan();


    /**
    \brief Try to compute a RobotMotion complying with the entered constraints without stepping.
    */
    bool basicSolve();

    /**
    \brief Convert user-entered prioritized constraints to single motion planning tasks with the given start time and duration
    */
    void defaultPlannerTaskMaker(double defaultStartTime, double defaultTaskDuration);

    /**
    \brief Try to compute a RobotMotion complying with the entered constraints by doing a step at the same time.
     */
    bool onestepSolve();

    /**
    \brief Used by oneStep solve to find the projection of the furthest position or tranformation target from the vertical axis between the footprints
    */
    void furthestTargetProjection(double centerX, double centerY, double& outX, double& outY, double& outDistance);

    /**
    \brief Used by onestepSolve().
    Compute footprint candidates for a stepping motion that can help realize the constraints entered by the user. attMaxIterations footprints are computed.
    */
    void createFootprintCandidates(const ChppGikFootprint* startFootprint,double targetX, double targetY, std::vector<ChppGikFootprint*>& inVectorFootprints );

    /**
    \brief Used by onestepSolve().
    Delete footprints created by createFootprintCandidates()
     */
    void deleteFootprintCandidates(std::vector<ChppGikFootprint*>& inVectorFootprints );

    bool attEnableStep;
    bool attHadToStep;
    ChppGikGenericTask* attGenericTask;
    std::vector<ChppGikPrioritizedStateConstraint*> attUserStateTasks;
    std::vector<ChppGikReadyElement*> attUserMotionTasks;
    unsigned int attMaxIterations;

};

#endif
