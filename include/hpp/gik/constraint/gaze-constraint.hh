#ifndef HPP_GIK_GAZE_CONSTRAINT_H
#define HPP_GIK_GAZE_CONSTRAINT_H

#include <vector>
#include "abstract-robot-dynamics/traits/default-pointer.hh"
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
 
  /// \brief Set the joint associated to the constraint.
  /// \note Does nothing since not applicable for this class.
  void  joint(CjrlJoint*) {}
    
  /// \brief Set the origin of the pointing vector.
  /// \note Does nothing since not applicable for this class.
  void  localOrigin(const vector3d&) {}
    
  /// \brief Set the pointing vector in joint's local frame
  /// \note Does nothing since not applicable for this class.
  void  localVector(const vector3d&) {}
    
    /**
    \brief Destructor
     */
    ~ChppGikGazeConstraint();

};

#endif
