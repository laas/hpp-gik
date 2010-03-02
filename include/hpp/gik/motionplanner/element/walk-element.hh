#ifndef HPP_GIK_WALK_ELEMENT_H
#define HPP_GIK_WALK_ELEMENT_H

#include "hpp/gik/motionplanner/element/step-element.hh"
#include "hpp/gik/robot/foot-print-related.hh"

/**
\brief This is motion planner for a walk (multiple steps)
\ingroup motions
 */
class ChppGikWalkElement: public ChppGikLocomotionElement
{
    public:

        /**
            \brief Constructor. The minimum time for the step motion (i.e parameter inFlightTime) is 0.5. So if the user constructs a step with anything below, it gets automatically thresholded.
         */
        ChppGikWalkElement ( CjrlHumanoidDynamicRobot* inRobot, double inSamplingPeriod, double inStartTime, const std::vector<ChppGikStepTarget*>& inAbsoluteSteps, double inZMPstart = 0.1, double inFoot = 1.2, double inZMPEnd = 0.1);
        
        /**
        \brief Constructor. The minimum time for the step motion (i.e parameter inFlightTime) is 0.5. So if the user constructs a step with anything below, it gets automatically thresholded.
         */
        ChppGikWalkElement ( double rightfoot2TargetZMPX, double rightfoot2TargetZMPY ,double zmplasttime, CjrlHumanoidDynamicRobot* inRobot, double inSamplingPeriod, double inStartTime, const std::vector<ChppGikStepTarget*>& inAbsoluteSteps, double inZMPstart = 0.1 , double inFoot = 1.2, double inZMPEnd = 0.1);

        /**
            \brief Clone constructor
         */
        virtual CjrlGikMotionConstraint* clone() const;

        /**
            \brief Get state constraint at a given time.
         */
        virtual CjrlGikStateConstraint* stateConstraintAtTime ( double inTime );

        /**
        \brief Get foot transformation constraint at given time
         */
        ChppGikTransformationConstraint* footConstraintAtTime ( double inTime );

        /**
            \brief Get the support foot joint at time inTime
            \return 0 if the time is out of definition bounds
         */
        virtual CjrlJoint* supportFootAtTime ( double inTime );

        /**
            \brief Plan stability-consistent ZMP and update given arguments in case of success.
            \param supportPolygon used and modified by this element according to purpose
            \param ZMP used and modified by this element according to purpose
            \return false conditions required by implemented planning algorithms not met
         */
        virtual bool plan ( ChppGikSupportPolygon& supportPolygon, vector3d& ZMP );

        /**
            \brief Mmake this motion constraint return the last planned constraint, from time endTime() to time endTime()+inProlongation.
         */
        virtual void preProlongate ( double inPreProlongation );

        /**
            \brief Make this motion constraint return the first planned constraint from time current startTime()-inProlongation to current startTime().
         */
        virtual void postProlongate ( double inPostProlongation );


        /**
            \brief Destructor
         */
        ~ChppGikWalkElement();

    private:
        double attZMPstart,attZMPend,attFootFlight,attStepDuration,attZMPlasttime;
        std::vector<ChppGikStepElement*> attSteps;
        std::vector<ChppGikStepTarget*> attStepTargets;
        bool attUseZMPcoefficient;
        double attRightfoot2TargetZMPX;
        double attRightfoot2TargetZMPY;
};


#endif
