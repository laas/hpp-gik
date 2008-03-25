#ifndef HPP_GIK_LOCOMOTION_PLAN_H
#define HPP_GIK_LOCOMOTION_PLAN_H

#include "robot/hppGikStandingRobot.h"
#include "motionplanners/hppGikPreviewController.h"
#include "motionplanners/elements/hppGikLocomotionElement.h"
#include "motionplanners/elements/hppGikNoLocomotion.h"
#include "motionplanners/elements/hppGikComMotion.h"
#include "core/hppGikMotionPlan.h"


/**
\brief This is a locomotion plan for objects of type ChppGikLocomotionElement.
its purpose is to transform ChppGikLocomotionElement objects into ChppGikPrioritizedMotion objects which can be inserted in a ChppGikMotionPlan object
 
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
    \note the start time is fixed to 0.0
     */
    ChppGikLocomotionPlan(ChppGikMotionPlan* inAssociatedMotionPlan, ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod);

    /**
    \brief Add a locomotion element (the object is not copied).
    \return false if the time frame taken by the added element overlaps with a previously added element
     */
    bool addElement(ChppGikLocomotionElement* inElement);

    /**
    \brief Delete entered elements (the object deleted)
    */
    void clearElements();

    /**
    \brief Get the end time according to entered elements
     */
    double endTime();
    
   
    /**
    \brief Get an extra end time
     */
    double extraEndTime();

    /**
    \brief Set an extra end time
    */
    void extraEndTime(double inDuration);
    /**
    @}
    */

    /**
    \name Computations
    {@
     */

    /**
    \brief Solve = based on the entered elements do:
        - plan ZMP
        - plan COM
        - append feet motion to MotionPlan
     \return false if any of the above steps fail
     */
    bool solve();

    /**
    \brief Get an appropriate joints weighting vecotr according to current task.
    \return False if given time is out of bounds
    */
    bool getWeightsAtTime(double inTime, vectorN& outWeights);

    /**
    \brief Get planned ZMP at time inTime
    \return False if given time is out of bounds
     */
    bool getZMPAtTime(double inTime, vectorN& outZMP);

    /**
    \brief Get the currently active task if any.
    \return null pointer if no locomotion element is active at the given time
    */
    const ChppGikLocomotionElement* activeElement(double inTime) const;

    /**
    \brief Get the default support foot joint at the given time
    \return null pointer if no locomotion element is active at the given time
     */
    CjrlJoint* supportFootJoint(double inTime);



    /**
    @}
     */

    /**
    \brief Destructor
     */
    ~ChppGikLocomotionPlan();

private:
    void clearSolverMess();
    bool planElementsZMP();
    void prolongateZMP(double inDuration);

    CjrlHumanoidDynamicRobot* attRobot;
    ChppGikStandingRobot* attStandingRobot;
    ChppGikMotionPlan* attAssociatedMotionPlan;
    std::vector<ChppGikLocomotionElement*> attElements;
    ChppGikComMotion* attComMotion;
    ChppGikMotionPlanRow* attComMotionPlanRow, *attFootMotionPlanRow, *attNoElementRow;
    ChppGikPreviewController* attPreviewController;
    ChppGikNoLocomotion* attNoElementCase;
    double attStartTime, attModifiedStartTime;
    double attEndTime, attModifiedEndTime;
    double attExtraEndTime;
    double attExtraZMPEndTime;
    double attSamplingPeriod, attEps;
    bool attPlanSuccess;
    matrixNxP attPlannedZMP;

};

#endif
