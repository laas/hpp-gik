#ifndef HPP_GIK_TRANSFORMATION_CONSTRAINT_H
#define HPP_GIK_TRANSFORMATION_CONSTRAINT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikTransformationConstraint.h"
#include "constraints/hppGikJointStateConstraint.h"

/**
\brief Specify a 3D position constraint on a point of the robot.
 */

class ChppGikTransformationConstraint: public CjrlGikTransformationConstraint, public ChppGikJointStateConstraint
{
public:
    /**
    \brief Constructor
     */
    ChppGikTransformationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inPointInWorldFrame, const matrix3d& inOrientation);

    ChppGikTransformationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame,  const matrix4d& inTransformation);


    /**
    \brief Copy the object
    */
    virtual CjrlGikStateConstraint* clone() const;

    /**
    \brief Set the point (in joint's local frame) associated to the constraint.
     */
    virtual void  localPoint(const vector3d& inPoint);
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const vector3d& localPoint();
    /**
    \brief Set the target point associated to the constraint (in world's frame).
     */
    virtual void  worldTarget(const vector3d& inPoint);
    /**
    \brief Get the target point associated to the constraint (in world's frame).
     */
    virtual const vector3d& worldTarget();
    /**
    \brief Set the target orientation for this constraint.
     */
    virtual void  targetOrientation(const matrix3d& inRot);
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const matrix3d& targetOrientation();

    /**
    \brief Set the target transformation for this constraint.
     */
    virtual void  targetTransformation(const matrix4d& inTransform);
    /**
    \brief Get the target transformation
     */
    virtual const matrix4d& targetTransformation();

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
    virtual ~ChppGikTransformationConstraint()
    {}

private:

    vector3d attLocalPointVector3;

    vector3d attWorldTargetVector3;

    matrix3d attTargetOrientationMatrix3;
    
    matrix4d attTargetTransformation4;


    vectorN attLocalPoint;

    vectorN attWorldTarget;

    matrixNxP attTargetOrientation;


    /** \name Computation temporary variables (to avoid dynamic allocation)
    {@
     */

    matrixNxP tempJacobian;
    vectorN temp3DVec;
    vectorN temp3DVec1;
    matrixNxP tempRot;
    matrixNxP tempGapRot;
    CjrlJoint* tempFixedJoint;
    const matrixNxP* tempFixedJointJacobian;
    const matrixNxP* tempEffectorJointJacobian;
    /**
    @}
     */
};

#endif
