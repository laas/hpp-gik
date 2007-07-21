#ifndef HPP_GIK_TRANSFORMATION_CONSTRAINT_H
#define HPP_GIK_TRANSFORMATION_CONSTRAINT_H

#include "gikTask/jrlGikTransformationConstraint.h"
#include "constraints/hppGikSingleMotionElementConstraint.h"


/**
\brief Specify a 3D position constraint on a point of the robot.
 */

class ChppGikTransformationConstraint:public CjrlGikTransformationConstraint, public ChppGikSingleMotionElementConstraint
{
public:
    /**
    \name Definition of the constraint
        @{
     */

    /**
    \brief Constructor
     */
    ChppGikTransformationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inPointInWorldFrame, const matrix3d& inOrientation);
    
    ChppGikTransformationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame,  const matrix4d& inTransformation);


    /**
    \brief Copy the object
    */
    CjrlGikStateConstraint* clone() const;


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
    \brief Same as above methos but take/return ublas vectors/matrices.
     */
    virtual void  worldTargetU(const vectorN& inPoint);
    virtual const vectorN& worldTargetU();
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
    virtual ~ChppGikTransformationConstraint()
    {}

private:

    CjrlDynamicRobot* attRobot;

    CjrlJoint* attJoint;


    vector3d attLocalPointVector3;

    vector3d attWorldTargetVector3;

    matrix3d attTargetOrientationMatrix3;


    vectorN attLocalPoint;

    vectorN attWorldTarget;

    matrixNxP attTargetOrientation;


    /** \name Computation temporary variables (to avoid dynamic allocation)
    {@
     */
    vectorN attValue;
    matrixNxP attJacobian;
    matrixNxP tempJacobian;
    vectorN attInfluencingDofs;

    unsigned int tempNumJoints;
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
