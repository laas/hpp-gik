#ifndef HPP_GIK_LOCOMOTION_ELEMENT_H
#define HPP_GIK_LOCOMOTION_ELEMENT_H

#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"
#include "hpp/gik/robot/standing-robot.hh"
#include "hpp/gik/core/prioritized-motion.hh"
#include "hpp/gik/constraint/transformation-constraint.hh"

/**
\brief A locomotion element is a motion planning task that produces threes motion chunks: a 2D Zero Momentum Point motion, a foot motion constraint and a support polygon motion.
 */
class ChppGikLocomotionElement: public CjrlGikMotionConstraint, public ChppGikPrioritizedMotion
{
    public:

        /**
        \brief constructor
        */
        ChppGikLocomotionElement ( ChppGikStandingRobot* inSRobot, double inStartTime, double inDuration, double inSamplingPeriod ) :ChppGikPrioritizedMotion ( inSRobot->robot(), 0, this, 0.0 )
        {
            attHumanoidRobot = inSRobot->robot();
            attStandingRobot = inSRobot;
            attStartTime = attModifiedStart = inStartTime;
            attDuration = ( inDuration>=0.0 ) ?inDuration:0.0;
            attEndTime = attModifiedEnd = attStartTime + attDuration;
            attSamplingPeriod = inSamplingPeriod;
            attEps = attSamplingPeriod/2;
            attPostProlongation = 0.0;
            attPreProlongation = 0.0;
            attConstrainedFoot = attSupportFoot = 0;
            attZMPmotion.resize ( 3,0 );
            attPlanSuccess = false;
        }

        /**
        \brief Get a pointer to associated robot
         */
        CjrlDynamicRobot* robot()
        {
            return attRobot;
        }

        /**
        \brief Get the foot transformation constraint at given time
         */
        virtual ChppGikTransformationConstraint* footConstraintAtTime ( double inTime ) = 0;
        

        /**
        \brief Mmake this motion constraint return the last planned constraint, from time endTime() to time endTime()+inProlongation.
         */
        virtual void preProlongate ( double inPreProlongation )
        {
            attPreProlongation = ( inPreProlongation>0.0 ) ?inPreProlongation:0.0;
            attModifiedStart = attStartTime - attPreProlongation;
            attPlanSuccess = false;
        }

        /**
        \brief Make this motion constraint return the first planned constraint from time current startTime()-inProlongation to current startTime().
         */
        virtual void postProlongate ( double inPostProlongation )
        {
            attPostProlongation = ( inPostProlongation>0.0 ) ?inPostProlongation:0.0;
            attModifiedEnd = attEndTime + attPostProlongation;
            attPlanSuccess = false;
        }

        /**
        \brief Disactivated
         */
        virtual void startTime ( double inStartTime )
        {}

        /**
        \brief Get the start time for this jrlGikMotionConstraint
        \return the start time given upon construction minus the preprolongation time
         */
        virtual double startTime()
        {
            return attModifiedStart;
        }

        /**
        \brief Get the end time for this jrlGikMotionConstraint
        \return the end time given upon construction plus the postprolongation time
         */
        virtual double endTime()
        {
            return attModifiedEnd;
        }

        /**
        \brief Get the duration of the original motion
        \note duration of the original motion i.e with prolongation being null
         */
        virtual double duration()
        {
            return attDuration;
        }

        /**
        \brief Get a pointer to the motion constraint
        */
        virtual CjrlGikMotionConstraint* motionConstraint()
        {
            return this;
        }

        /**
        \brief Get the support foot joint at time inTime
        \return 0 if the time is out of definition bounds
         */
        virtual matrixNxP& ZMPmotion()
        {
            return attZMPmotion;
        }

        /**
        \brief Get the support foot joint at time inTime
        \return 0 if the time is out of definition bounds
         */
        virtual CjrlJoint* supportFootAtTime ( double inTime ) = 0;

        /**
        \brief Plan stability-consistent ZMP and update given arguments in case of success.
        \param supportPolygon used and modified by this element according to purpose
        \param ZMP used and modified by this element according to purpose
        \return false conditions required by implemented planning algorithms not met
         */
        virtual bool plan ( ChppGikSupportPolygon& supportPolygon, vector3d& ZMP ) = 0;

        /**
            \brief Destructor
         */
        virtual ~ChppGikLocomotionElement()
        {}

    protected:

        double attPreProlongation;
        double attPostProlongation;
        double attStartTime, attEndTime, attSamplingPeriod, attEps, attDuration, attModifiedStart, attModifiedEnd;
        CjrlJoint* attSupportFoot, *attConstrainedFoot;
        bool attPlanSuccess;
        matrixNxP attZMPmotion;
        CjrlHumanoidDynamicRobot* attHumanoidRobot;
        ChppGikStandingRobot* attStandingRobot;
};


#endif


