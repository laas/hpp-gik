#ifndef HPP_GIK_RELATIVE_TRANSFORMATION_CONSTRAINT_H
#define HPP_GIK_RELATIVE_TRANSFORMATION_CONSTRAINT_H

#include <hpp/gik/constraint/transformation-constraint.hh>

/**
\brief Specify a transformation constraint on a joint of the robot, relatively to another joint.
\ingroup constraints
 */

class ChppGikRelativeTransformationConstraint: public ChppGikTransformationConstraint
{
public:
    /**
    \brief Constructor
     */
  
  ChppGikRelativeTransformationConstraint(CjrlDynamicRobot& inRobot, 
					  CjrlJoint& inJoint, 
					  CjrlJoint * inRootJoint,
					  const matrix4d& inRelativeTransformation);

    /**
    \brief Copy the object
    */
    virtual CjrlGikStateConstraint* clone() const;

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
  virtual ~ChppGikRelativeTransformationConstraint()
  {}

private:
  CjrlJoint * rootJoint_;

  matrix4d targetTransformation4_;
  vectorN targetT_;
  matrixNxP targetR_;

  matrixNxP jointR_;
  vectorN jointT_;

  matrixNxP gapR_;
  vectorN gapREuler_;

  matrix4d globalTargetTransformation_;
};

#endif //HPP_GIK_RELATIVE_TRANSFORMATION_CONSTRAINT_H
