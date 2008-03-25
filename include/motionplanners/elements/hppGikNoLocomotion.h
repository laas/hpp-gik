#ifndef HPP_GIK_NOLOCOMOTION_ELEMENT_H
#define HPP_GIK_NOLOCOMOTION_ELEMENT_H

#include "core/hppGikPrioritizedMotion.h"
#include "core/hppGikMotionPlanElement.h"
#include "constraints/hppGikComConstraint.h"
#include "constraints/hppGikTransformationConstraint.h"

/**
\brief A motion that return a constraint composed of a Com and a Foot transformation. The constraint is built upon construction and is returned between startTime and endTime.
 */
class ChppGikNoLocomotion:public ChppGikPrioritizedMotion, public CjrlGikMotionConstraint
{
public:

    /**
        \brief Constructor
        \note given times should be superior to 0.0
     */
    ChppGikNoLocomotion(CjrlHumanoidDynamicRobot* inRobot, CjrlJoint* inConstrainedFoot, double inStartTime, double inEndTime, const vectorN& inWorkingJoints, unsigned int inPriority);

    /**
        \brief Get a pointer to associated robot
     */
    CjrlDynamicRobot* robot();

    /**
        \brief Get a pointer to the motion constraint
     */
    CjrlGikMotionConstraint* motionConstraint();

    /**
        \brief Clone constructor
     */
    CjrlGikMotionConstraint* clone() const;

    /**
        \brief Get state constraint at a given time.
     */
    CjrlGikStateConstraint* stateConstraintAtTime(double inTime);

    /**
        \brief Set lower bound of definition interval.
        \note inStartTime must be between 0.0 and endTime()
     */
    void startTime(double inStartTime);

    /**
        \brief Set upper bound of definition interval.
        \note inEndTime must be superior to startTIme()
     */
    void endTime(double inEndTime);

    /**
        \brief Get lower bound of definition interval.
     */
    double startTime();

    /**
        \brief Get upper bound of definition interval.
     */
    double endTime();

    /**
        \brief Get the support foot
    */
    CjrlJoint* supportFoot();
    
    /**
    \brief Get the support foot
     */
    const vectorN& ZMP();

    /**
        \brief Destructor
     */
    ~ChppGikNoLocomotion();


private:
    ChppGikComConstraint* attComConstraint;
    ChppGikTransformationConstraint* attFootConstraint;
    ChppGikMotionPlanElement* attConstraint;
    CjrlJoint* attSupportFoot;
    CjrlHumanoidDynamicRobot* attRobot;
    double attStartTime, attEndTime;
    vectorN attZMP;
};

#endif
