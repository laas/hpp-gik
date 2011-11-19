#ifndef HPP_GIK_RELATIVE_COM_CONSTRAINT_H
#define HPP_GIK_RELATIVE_COM_CONSTRAINT_H

#include <hpp/gik/constraint/com-constraint.hh>

/**
   \brief Specify a relative position constraint on the center of mass.
   \ingroup constraints
*/

class ChppGikRelativeComConstraint: public ChppGikComConstraint
{
public:
  /**
     \name Definition of the constraint
     @{
  */

  /**
     \brief Constructor
     @param inRobot Robot
     @param inJoint Joint in which the CoM position is computed
     @param inX CoM target position
     @param inY CoM target position
  */
  ChppGikRelativeComConstraint(CjrlDynamicRobot& inRobot, CjrlJoint* inJoint, double inX, double inY);
  
  /**
     \brief Copy the object
  */
  CjrlGikStateConstraint* clone() const;

     /**
    \brief Compute the value of the constraint.
     */
    virtual void computeValue();

   /**
    \brief Get the constraint Jacobian wrt internal configuration variables.
   */
    virtual void computeJacobian();

   /**
    \brief Destructor
     */
  virtual ~ChppGikRelativeComConstraint();


  /**
    \brief Get the constraint value.
  */
  virtual const vectorN& value();


  /**
    \brief Get the constraint jacobian
  */
  virtual const matrixNxP& jacobian();



private:
  CjrlDynamicRobot* robot_;

  CjrlJoint *rootJoint_;

  matrixNxP comJacobian_;
  matrixNxP posJacobian_;
  matrixNxP jacobian_;

  matrixNxP jointRot_;

  vectorN comPos_;
  vectorN jointTr_;
  vectorN worldTarget_;
  vectorN value_;

  vectorN localTarget_;
  vector3d localTargetVector3d_;
};

#endif
