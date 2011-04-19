#ifndef HPP_GIK_GAZE_CONSTRAINT_H
#define HPP_GIK_GAZE_CONSTRAINT_H

#include <vector>
#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"
#include "hpp/gik/constraint/pointing-constraint.hh"
#include "gikTask/jrlGikGazeConstraint.h"

/**
\brief Constraint on a line segment attached to a body to be aligned with a given point in the world frame. The line segment is defined by an origin point and a vector both given in the body's local frame.
\ingroup constraints
 */
class ChppGikGazeConstraint: public ChppGikPointingConstraint, public CjrlGikGazeConstraint
{
public:
    /**
    \name Definition of the constraint
    @{
     */
    /**
    brief Constructor
    */
    ChppGikGazeConstraint(CjrlHumanoidDynamicRobot& inRobot, const vector3d& inTargetWorldPoint);
 
    /**
    \brief Set the joint associated to the constraint.
     */
    void  joint(CjrlJoint* inJoint) {};
    
    /**
    \brief Set the origin of the pointing vector.
     */
    void  localOrigin(const vector3d& inPoint) {};
    
    /**
    \brief Set the pointing vector in joint's local frame
     */
    void  localVector(const vector3d& inPoint) {};
    
    /**
    \brief Destructor
     */
    ~ChppGikGazeConstraint();

};

#endif
