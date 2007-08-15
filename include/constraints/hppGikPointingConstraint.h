#ifndef HPP_GIK_POINTING_CONSTRAINT_H
#define HPP_GIK_POINTING_CONSTRAINT_H

#include "gikTask/jrlGikPointingConstraint.h"
#include "constraints/hppGikSingleMotionElementConstraint.h"

/**
\brief Constraint on a line segment attached to a body to be aligned with a given point in the world frame. The line segment is defined by an origin point and a vector both given in the body's local frame.
 */
class ChppGikPointingConstraint: virtual public CjrlGikPointingConstraint, public ChppGikSingleMotionElementConstraint
{
public:
    /**
        \name Definition of the constraint
        @{
     */

    ChppGikPointingConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalOrigin, const vector3d& inLocalVector, const vector3d& inTargetWorldPoint);
    /**
        \brief Copy
     */
    virtual CjrlGikStateConstraint* clone() const;

    /**
        \brief Get the dimension of the constraint.
     */
    unsigned int dimension() const;

    /**
        \brief Get robot associated to the constraint.
     */
    CjrlDynamicRobot& robot();

    /**
        \brief Set the joint associated to the constraint.
     */
    virtual void  joint(CjrlJoint* inJoint);
    /**
        \brief Get the joint associated to the constraint.
     */
    CjrlJoint* joint();
    /**
        \brief Set the origin of the pointing vector in joint's local frame (illelgal operation for gaze constraint).
     */
    virtual void  localOrigin(const vector3d& inPoint);
    /**
        \brief Get the origin of the pointing vector.
     */
    const vector3d& localOrigin();
    /**
        \brief Set the pointing vector in joint's local frame (illelgal operation for gaze constraint).
     */
    virtual void  localVector(const vector3d& inPoint);
    /**
        \brief Set the pointing vector in joint's local frame
     */
    const vector3d& localVector();
    /**
        \brief Set the target point associated to the constraint (in world's frame).
     */
    void  worldTarget(const vector3d& inPoint);
    /**
        \brief Get the target point associated to the constraint (in world's frame).
     */
    const vector3d& worldTarget();
    /**
        @}
     */

    /**
    \brief Same as worldTarget but take/return ublas vector
     */
    void  worldTargetU(const vectorN& inPoint);
    const vectorN& worldTargetU();
    /**
        \name Computations
        @{
     */
    /**
    \brief Get a binary vector which size matches the robot cnfiguration's, where an element with value 1 indicates that the corresponding degree of freedom can modify the value of this constraint, and an element with value 0 cannot.
    */
    vectorN& influencingDofs();
    /**
    \brief The interpolation is carried out in this way:
    First take the world target point (final point) and compute the distance L to current local origin
    Then compute the position of a point at the distance  L from the current local origin in the direction given by the local vector.
    This gives an arbitrary initial point. From there compute a minimum jerk motion to the final state.
    False is returned if:
    -the local origin is found between the initial and final points (because it is likely to produce some weired behaviour)
    -the local origin coincides with target or second point
    */
    bool minimumJerkInterpolation(ChppGikMotionConstraint* outMotionConstraint, double inSamplingPeriod, double inTime);
    
    /**
        \brief Compute the value of the constraint. Stored in a static variable.
     */
    void computeValue();

    /**
    \brief Get the constraint Jacobian wrt internal configuration variables.
    This method supposes that:
     * the robot has at least one fixed joint.
     * the jacobian for this fixed joint has been computed for the current configuration
    
    Only the first fixed joint of the robot affects the computation of the jacobian. (closed kinematic chains are not handeled)
     */
    void computeJacobian();

    /**
        @}
     */

    /**
        \name Getting result of computations
        @{
     */

    /**
        \brief Get the constraint value.
     */
    const vectorN& value();

    /**
    \brief Get the constraint jacobian.
     */
    const matrixNxP& jacobian();

    /**
        @}
     */
    /**
        \brief Destructor
     */
    virtual ~ChppGikPointingConstraint();

protected:

    CjrlDynamicRobot* attRobot;

    CjrlJoint* attJoint;


    vector3d attLocalOriginVector3;

    vector3d attLocalVectorVector3;

    vector3d attWorldTargetVector3;


    vectorN attLocalOrigin;

    vectorN attLocalVector;

    vectorN attWorldTarget;

    vectorN attInfluencingDofs;


    /**
    \name Computation temporary variables (to avoid dynamic allocation)
    {@
     */

    vectorN attValue;
    matrixNxP attJacobian;

    unsigned int tempNumJoints;
    matrixNxP tempRot;
    matrixNxP matOP;
    matrixNxP matOT;
    matrixNxP matFO;
    matrixNxP rotF;
    matrixNxP jointrot;
    vectorN jointpos;
    vectorN posO;
    vectorN posF;
    vectorN posP;
    vectorN vecFO;
    vectorN vecOP;
    vectorN vecOT;
    CjrlJoint* tempFixedJoint;
    const matrixNxP* tempFixedJointJacobian;
    const matrixNxP* tempEffectorJointJacobian;
    matrixNxP tempJacobian;
    matrixNxP tempJacobian0;
    /**
    @}
     */

};

#endif
