#ifndef HPP_GIK_STEP_ELEMENT_H
#define HPP_GIK_STEP_ELEMENT_H

#include "motionplanners/elements/hppGikZMPshiftElement.h"
#include "motionplanners/elements/hppGikFootDisplaceElement.h"

/**
\brief This is motion planner for a step
This is a sequence ChppGikZMPshiftElement-ChppGikFootDisplaceElement-ChppGIkZMPshiftElement that plans motion for the feet and for the ZMP as follows:
<br>
First, ZMP is moved under the support foot.
<br>
Second, the non-support foot is displaced
<br>
Third, the ZMP is moved at a given point between the two feet.
\ingroup motions
 */
class ChppGikStepElement: public ChppGikLocomotionElement
{
public:

    /**
        \brief Constructor. The minimum time for the step motion (i.e parameter inFlightTime) is 0.5. So if the user constructs a step with anything below, it gets automatically thresholded.
     */
    ChppGikStepElement(CjrlHumanoidDynamicRobot* inRobot,double inStartTime, const ChppGikFootprint* inFootprint, bool isRightFoot, double inSamplingPeriod, double inFinalZMPCoefficient=0.75, double inEndShiftTime=0.8, double inStartZMPShiftTime=0.6, double inFootMotionDuration=0.8, double inStepHeight=0.05);

    /**
        \brief Constructor. The minimum time for the step motion (i.e parameter inFlightTime) is 0.5. So if the user constructs a step with anything below, it gets automatically thresholded.
     */
    ChppGikStepElement(CjrlHumanoidDynamicRobot* inRobot, const ChppGikFootprint* inFootprint, double inStartTime, bool isRightFoot, double rightfoot2TargetZMPX, double rightfoot2TargetZMPY, double inSamplingPeriod, double inEndShiftTime=0.8, double inStartZMPShiftTime=0.6, double inFootMotionDuration=0.8, double inStepHeight=0.05);

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
    \return true if this step is for right foot, false if for left
    */
    bool isRight();


    /**
        \brief Destructor
     */
    ~ChppGikStepElement();

private:
    void init(double inFootMotionDuration, bool isRight, double inHeight, const ChppGikFootprint* inFootprint, double inStartShiftTime, double inEndShiftTime);
    double attRfoot2TargetZMPX, attRfoot2TargetZMPY;
    double attFinalZMPCoef;
    bool attUseZMPcoefficient;
    ChppGikZMPshiftElement* attShift1, * attShift2;
    ChppGikFootDisplaceElement* attFootDisplace;

};


#endif
