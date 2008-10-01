#ifndef HPP_GIK_WALK_ELEMENT_H
#define HPP_GIK_WALK_ELEMENT_H

#include "hppGik/motionplanners/elements/hppGikStepElement.h"
#include "hppGik/robot/hppGikFootprintRelated.h"

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
        ChppGikWalkElement(CjrlHumanoidDynamicRobot* inRobot, double inSamplingPeriod, double inStartTime, const std::vector<ChppGikStepTarget*>& inAbsoluteSteps);

    /**
        \brief Clone constructor
     */
        virtual CjrlGikMotionConstraint* clone() const;

    /**
        \brief Get state constraint at a given time.
     */
        virtual CjrlGikStateConstraint* stateConstraintAtTime(double inTime);

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
        \brief Mmake this motion constraint return the last planned constraint, from time endTime() to time endTime()+inProlongation.
     */
        virtual void preProlongate(double inPreProlongation);

    /**
        \brief Make this motion constraint return the first planned constraint from time current startTime()-inProlongation to current startTime().
     */
        virtual void postProlongate(double inPostProlongation);


    /**
        \brief Destructor
     */
        ~ChppGikWalkElement();

    private:
        double attZMPstart,attZMPend,attFootFlight,attStepDuration;
        std::vector<ChppGikStepElement*> attSteps;
        std::vector<ChppGikStepTarget*> attStepTargets;
};


#endif
