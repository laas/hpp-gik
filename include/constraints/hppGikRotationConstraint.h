#ifndef HPP_GIK_ROTATION_CONSTRAINT_H
#define HPP_GIK_ROTATION_CONSTRAINT_H

#include "gikTask/jrlGikRotationConstraint.h"
#include "constraints/hppGikSingleMotionElementConstraint.h"

/**
\brief Specify a 3D orientation constraint on a body of the robot.
 */

class ChppGikRotationConstraint:public CjrlGikRotationConstraint, public ChppGikSingleMotionElementConstraint
{
public:
    /**
    \name Definition of the constraint
    @{
     */

    /**
    \brief Constructor
     */
    ChppGikRotationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const matrix3d& inTargetOrientation);

    /**
    \brief Copy the object
     */
    virtual CjrlGikStateConstraint* clone() const;


    /**
    \brief Get the dimension of the constraint.
     */
    virtual unsigned int dimension() const;

    /**
    \brief Get robot associated to the constraint.
     */
    virtual CjrlDynamicRobot& robot();

    /**
    \brief Set the joint associated to the constraint.
     */
    virtual void  joint(CjrlJoint* inJoint);
    /**
    \brief Get the joint associated to the constraint.
     */
    virtual  CjrlJoint* joint();
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
    @}
     */

    /**
    \name Computations
    @{
     */
    /**
    \brief Get a binary vector which size matches the robot cnfiguration's, where an element with value 1 indicates that the corresponding degree of freedom can modify the value of this constraint, and an element with value 0 cannot.
    */
    virtual vectorN& influencingDofs();
    /**
    \brief This method computes a minimum jerk motion constraint for the given motion time and sampling rate.
    First the current position velocity and acceleration of the constraint are computed using outputs from the CjrlDynamicRobot.
    inStateConstraint must have a target position, velocity and acceleration.
    This implementation assumes the target velocity and accelerations to be constantly 0.
     */
    virtual bool minimumJerkInterpolation(ChppGikMotionConstraint* outMotionConstraint, double inSamplingPeriod, double inTime);

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
    @}
     */

    /**
    \name Getting result of computations
    @{
     */

    /**
    \brief Get the constraint value.
     */
    virtual const vectorN& value();

    /**
    \brief Get the constraint jacobian.
     */
    virtual const matrixNxP& jacobian();

    /**
    @}
     */

    /**
    \brief Destructor
     */
    virtual ~ChppGikRotationConstraint()
    {}

private:

    CjrlDynamicRobot* attRobot;

    CjrlJoint* attJoint;


    matrix3d attTargetOrientationMatrix3;


    matrixNxP attTargetOrientation;


    /** \name Computation temporary variables (to avoid dynamic allocation)
    {@
     */
    vectorN attInfluencingDofs;
    vectorN attValue;
    matrixNxP attJacobian;
    
    unsigned int tempNumJoints;
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
