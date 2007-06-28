#ifndef HPP_GIK_SINGLEMOTION_ELEMENT_CONSTRAINT_H
#define HPP_GIK_SINGLEMOTION_ELEMENT_CONSTRAINT_H

#include "hppGikMotionConstraint.h"
#include "gikTask/jrlGikJointStateConstraint.h"

/**
\brief Interface for state constraints interpolation. This abstract class is meant as an abstract parent for the inplementations of CjrlGikStateConstraint.
 */
class ChppGikInterpolable
{
public:
    /**
    \brief This method computes a minimum jerk motion constraint for the entered state constraint, motion time and sampling rate.
    First the current position velocity and acceleration of the constraint are computed using outputs from CjrlHumanoidDynamicRobot.
    inStateConstraint must have a target position, velocity and acceleration.
    \return true if interpolation succeeded
     */
    virtual bool minimumJerkInterpolation(ChppGikMotionConstraint* outMotionConstraint, double inSamplingPeriod, double inTime) =0;

    virtual ~ChppGikInterpolable(){}

};


class ChppGikSingleMotionElementConstraint: public ChppGikInterpolable, public CjrlGikJointStateConstraint
{
};
#endif
