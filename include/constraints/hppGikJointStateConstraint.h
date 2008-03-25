#ifndef HPP_GIK_JOINT_STATE_CONSTRAINT_H
#define HPP_GIK_JOINT_STATE_CONSTRAINT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikJointStateConstraint.h"

/**
\brief Implementation of some commom functions in CjrlGikJointStateConstraint
 */
class ChppGikJointStateConstraint: virtual public CjrlGikJointStateConstraint
{
public:

    /**
        \brief Constructor
    */
    ChppGikJointStateConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint);

    /**
    \brief Get the dimension of the constraint.
     */
    virtual unsigned int dimension() const;

    /**
        \brief Get robot associated to the constraint.
     */
    virtual CjrlDynamicRobot& robot();

    /**
        \brief Set the joint associated to the constraint. This also calls computeInfluencingDofs().
     */
    virtual void  joint(CjrlJoint* inJoint);
    /**
        \brief Get the joint associated to the constraint.
     */
    virtual  CjrlJoint* joint();

    /**
        \brief Compute a binary vector whose size matches the robot cnfiguration's, where an element with value 1 indicates that the corresponding degree of freedom can modify the value of this constraint, and an element with value 0 cannot.
     */
    virtual void computeInfluencingDofs();

    /**
        \brief Get the constraint value.
     */
    virtual const vectorN& value();

    /**
        \brief Get the constraint jacobian.
     */
    virtual const matrixNxP& jacobian();

    /**
        \brief Get the influencing dofs
     */
    virtual vectorN& influencingDofs();
    

protected:

    /**
        \brief The robot on which this constraint is applied
     */
    CjrlDynamicRobot* attRobot;

    /**
        \brief The joint on which this constraint is applied
     */
    CjrlJoint* attJoint;
    
    /**
    \brief The dimension of this constraint
    */
    unsigned int attDimension;

    /**
        \brief The value of this constraint
     */
    vectorN attValue;

    /**
        \brief The Jacobian of the constraint
     */
    matrixNxP attJacobian;

    /**
        \brief The degrees of freedom influencing this constraint from (and including) the root
     */
    vectorN attInfluencingDofs;
    
    /**
        \brief The number of actuated degrees of freedom 
    */
    unsigned int attNumberActuatedDofs;

};

#endif

