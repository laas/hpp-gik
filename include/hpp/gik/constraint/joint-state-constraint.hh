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

    \param inRobot Reference to the robot the constraint is associated to.
    \param inJoint Reference to the joint the constraint is associated to.
    \param inDimension of the constraint.
    */
    ChppGikJointStateConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint,
                                unsigned int inDimension);

    /**
    \brief Get the dimension of the constraint.
     */
    virtual unsigned int dimension() const;

    /**
    \brief Get robot associated to the constraint.
     */
    virtual CjrlDynamicRobot& robot()
    {
        return *attRobot;
    };

    /**
    \brief Set the joint associated to the constraint. Will trigger computeInfluencingDofs() when the joint changes
     */
    virtual void  joint(CjrlJoint* inJoint);
    /**
    \brief Get the joint associated to the constraint.
     */
    virtual  CjrlJoint* joint()
    {
        return attJoint;
    };

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

    /**
    \brief inherited. Will trigger computeInfluencingDofs() when the joint changes
    */
    virtual void jacobianRoot(CjrlJoint& inJoint);

protected:

    vectorN attValue;
    matrixNxP attJacobian;
    vectorN attInfluencingDofs;
    CjrlJoint *attRootJoint;

private:

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

};

#endif

