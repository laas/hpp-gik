#ifndef HPP_GIK_ROTATION_CONSTRAINT_H
#define HPP_GIK_ROTATION_CONSTRAINT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikRotationConstraint.h"
#include "constraints/hppGikJointStateConstraint.h"

/**
\brief Specify a 3D orientation constraint on a body of the robot.
 */

class ChppGikRotationConstraint:public CjrlGikRotationConstraint, public ChppGikJointStateConstraint
{
public:
    /**
    \brief Constructor
     */
    ChppGikRotationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const matrix3d& inTargetOrientation);

    /**
    \brief Copy the object
     */
    virtual CjrlGikStateConstraint* clone() const;

    /**
    \brief Set the target orientation for this constraint.
     */
    virtual void  targetOrientation(const matrix3d& inRot);
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const matrix3d& targetOrientation();

    /**
    \brief Same as targetorientation but take/return ublas matrix
     */
    virtual void  targetOrientationU(const matrixNxP& inRot);
    virtual const matrixNxP& targetOrientationU();

    /**
    \brief Get the full state of the constraint (constraint plus it's 2 first derivatives) expressed as a vectorN. Dimenstion of returned vector is 3xdimension of the implementing constraint
     */
    virtual const vectorN& vectorizedState();

    /**
    \brief Set the target of the constraint expressed as a vectorN.
    \return false if the vectorizedTarget is not of the correct dimension
     */
    virtual bool vectorizedTarget ( const vectorN& inTarget );
    
    /**
    \brief Get the target of the constraint expressed as a vectorN. Each constraint knows how to compute its own vectorizedTarget
     */
    virtual const vectorN& vectorizedTarget();

    /**
    \brief Compute the value of the constraint.
     */
    virtual void computeValue();

    /**
    \brief Get the constraint Jacobian wrt internal configuration variables.
    This method supposes that:
     * the robot has at least one fixed joint.
     * the jacobian for this fixed joint has been computed for the current configuration

    Only the first fixed joint of the robot affects the computation of the jacobian. (closed kinematic chains are not handeled)
     */
    virtual void computeJacobian();

    /**
    \brief Destructor
     */
    virtual ~ChppGikRotationConstraint()
    {}

private:

    matrix3d attTargetOrientationMatrix3;


    matrixNxP attTargetOrientation;


    /** \name Computation temporary variables (to avoid dynamic allocation)
    {@
     */
    matrixNxP tempRot;
    matrixNxP tempGapRot;
    vectorN temp3DVec;
    CjrlJoint* tempFixedJoint;
    const matrixNxP* tempFixedJointJacobian;
    const matrixNxP* tempEffectorJointJacobian;
    /**
    @}
     */
};

#endif
