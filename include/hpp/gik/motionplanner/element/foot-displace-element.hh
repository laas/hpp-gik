#ifndef HPP_GIK_FOOTDISPLACE_ELEMENT_H
#define HPP_GIK_FOOTDISPLACE_ELEMENT_H

#include "hpp/gik/motionplanner/element/locomotion-element.hh"
#include "hpp/gik/constraint/transformation-constraint.hh"

/**
\brief A locomotion element for displacing a foot to the given footprint
\ingroup motions
 */
class ChppGikFootDisplaceElement:public ChppGikLocomotionElement
{
public:

    /**
        \brief Constructor
     */
    ChppGikFootDisplaceElement(ChppGikStandingRobot* inSRobot, double inStartTime, const ChppGikFootprint * inTargetFootprint, bool isRight, double inDuration, double inSamplingPeriod, double inHeight=0.05);

    /**
        \brief Tell if this step is to be done by the right foot
     */
    bool isRight() const
    {
        return attIsRight;
    }
    
    /**
    \brief Get the via point's height during step
     */
    bool height() const
    {
        return attHeight;
    }

    /**
        \brief Get the target footprint
     */
    const ChppGikFootprint* targetFootprint() const
    {
        return attTargetFootprint;
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
    \brief Get foot transformation constraint at given time
     */
    virtual ChppGikTransformationConstraint* footConstraintAtTime ( double inTime );
    
    /**
        \brief Get the support foot joint at time inTime
        \return 0 if the time is out of definition bounds
     */
    virtual CjrlFoot* supportFootAtTime(double inTime);

    /**
        \brief Plan stability-consistent ZMP and update given arguments in case of success.
        \param supportPolygon used and modified by this element according to purpose
        \param ZMP used and modified by this element according to purpose
        \return false conditions required by implemented planning algorithms not met
     */
    virtual bool plan(ChppGikSupportPolygon& supportPolygon, vector3d& ZMP);

    /**
        \brief Destructor
     */
    ~ChppGikFootDisplaceElement();

private:
    bool planFeet();
    double attHeight;
    bool attIsRight;
    ChppGikFootprint* attTargetFootprint;
    ChppGikTransformationConstraint* attConstraint;
    matrixNxP attFootMotion;
    vectorN attVectorizedTarget;
};


#endif
