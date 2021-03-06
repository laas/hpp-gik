#ifndef HPP_GIK_ROTATION_CONSTRAINT_H
#define HPP_GIK_ROTATION_CONSTRAINT_H

#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"
#include "gikTask/jrlGikRotationConstraint.h"
#include "hpp/gik/constraint/joint-state-constraint.hh"
#include "hpp/gik/constraint/vectorizable-constraint.hh"

/**
\brief Specify a 3D orientation constraint on a body of the robot.
\ingroup constraints
 */

class ChppGikRotationConstraint:public CjrlGikRotationConstraint, public ChppGikJointStateConstraint, public ChppGikVectorizableConstraint
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
    \brief Get the full state (this constraint and its first two derivatives) of the constraint expressed as a vectorN.
     */
    virtual void computeVectorizedState();


    /**
    \brief Compute the target of the constraint as a vectorN.
     */
    virtual void computeVectorizedTarget();
    
    /**
    \brief A unified method to change the target of the constraint with a vectorN.
    \return false if the argument's size does not match the one expected for this constraint type
     */
    virtual bool vectorizedTarget( const vectorN& inTarget );

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

    /**
    @}
     */
};

#endif
