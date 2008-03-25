#ifndef HPP_GIK_COMMOTION_ELEMENT_H
#define HPP_GIK_COMMOTION_ELEMENT_H

#include "core/hppGikPrioritizedMotion.h"
#include "constraints/hppGikComConstraint.h"

/**
\brief A motion that return a constraint composed of a Com and a Foot transformation. The constraint is built upon construction and is returned between startTime and endTime.
 */
class ChppGikComMotion:public ChppGikPrioritizedMotion, public CjrlGikMotionConstraint
{
    public:

    /**
        \brief Constructor
        \note given times should be superior to 0.0
     */
        ChppGikComMotion(CjrlHumanoidDynamicRobot* inRobot, double inStartTime, double inSamplingPeriod, const vectorN& inWorkingJoints, unsigned int inPriority);

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
        \note inStartTime must be >= 0
     */
        void startTime(double inStartTime);

    /**
        \brief Get lower bound of definition interval.
     */
        double startTime();

    /**
        \brief Get upper bound of definition interval.
     */
        double endTime();

    /**
        \brief Set COM trajectory as a 2-by-(N+1) matrix. We have N*SamplingPeriod = motion_duration, the first sample being ignored by stateConstraintAtTime()
        \return False when entered matrix does not have exactly 2 rows
     */
        bool setSamples(const matrixNxP& inSamples);

    /**
        \brief Destructor
     */
        ~ChppGikComMotion();


    private:
        ChppGikComConstraint* attComConstraint;
        double attStartTime, attEndTime, attSamplingPeriod, attEps;
        matrixNxP attSamples;
};

#endif
