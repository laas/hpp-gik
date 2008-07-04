#ifndef HPP_GIK_CUSTOMFOOT_ELEMENT_H
#define HPP_GIK_CUSTOMFOOT_ELEMENT_H

#include "motionplanners/elements/hppGikLocomotionElement.h"

/**
\brief A locomotion element for moving a foot according to a custom foot motion (i.e. does not have to be a stepping).
 */
class ChppGikCustomFootMotionElement:public ChppGikLocomotionElement
{
public:

    /**
        \brief constructor
        \param inStartTime time of motion start
        \param inDuration time of motion start
        \param inFootTargetConstraint target constraint for the foot
        \note that the target constraint's joint has to be a foot that is not supporting the robot at the moment of motion start
     */
    ChppGikCustomFootElement(ChppPlannableConstraint * inFootTargetConstraint, bool inForRightFoot double inStartTime, double inDuration);

    /**
       \brief Clone constructor
     */
    virtual CjrlGikMotionConstraint* clone() const ;

    /**
        \brief Get state constraint at a given time.
     */
    virtual CjrlGikStateConstraint* stateConstraintAtTime(double inTime);

    /**
        \brief Set lower bound of definition interval.
     */
    virtual void startTime(double inStartTime);

    /**
        \brief Get lower bound of definition interval.
     */
    virtual double startTime() ;

    /**
        \brief Get the time available to solve the task
     */
    virtual double duration()  ;

    /**
        \brief Get upper bound of definition interval.
        \note Returned value is affected by method prolongate()
     */
    virtual double endTime() ;

    /**
        \brief Get the support foot joint at time inTime
        \return 0 if the time is out of definition bounds
     */
    virtual CjrlJoint* supportFootAtTime(double inTime);

    /**
    \brief Plan stability-consistent ZMP and update given arguments in case of success.
    \param supportPolygon used and modified by this element according to purpose
    \param ZMP used and modified by this element according to purpose
    \return false conditions required by implemented planning algorithms not met
     */
    virtual bool plan(ChppGikSupportPolygon& supportPolygon, vector3d& ZMP);

    /**
    \brief Get a pointer to the motion constraint planned by this element
     */
    virtual CjrlGikMotionConstraint* motionConstraint();


    /**
        \brief Destructor
     */
    ~ChppGikCustomFootMotionElement()
    {}
}
;

#endif

