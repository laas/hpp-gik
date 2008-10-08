#ifndef HPP_GIK_ZMPSHIFT_ELEMENT_H
#define HPP_GIK_ZMPSHIFT_ELEMENT_H

#include "motionplanners/elements/hppGikLocomotionElement.h"
#include "constraints/hppGikTransformationConstraint.h"

/**
\brief A locomotion element for shifting the body mass by planning a linear zmp motion inside the support polygon.
\ingroup motions
 */
class ChppGikZMPshiftElement:public ChppGikLocomotionElement
{
public:

    /**
        \brief Constructor
     */
    ChppGikZMPshiftElement(CjrlHumanoidDynamicRobot* inRobot, const vector3d& targetZMP, double inStartTime, double inDuration, double inSamplingPeriod);
    
    /**
        \brief Set target ZMP
     */
    void targetZMP(const vector3d& targetZMP)
    {
        attTargetZMP = targetZMP;
    }
    
    /**
        \brief Get target ZMP
     */
    const vector3d& targetZMP()
    {
        return attTargetZMP;
    }

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
    \brief enable/disable the checking of the initial ZMP inside supportPolygon
    */
    void startZMPCheck(bool inChoice);

    /**
        \brief Destructor
     */
    ~ChppGikZMPshiftElement();


private:

    vector3d attTargetZMP;
    ChppGikTransformationConstraint* attConstraint;
    bool attStartZMPCheck;
};

#endif
