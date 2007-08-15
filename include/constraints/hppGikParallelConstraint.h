#ifndef HPP_GIK_PARALLEL_CONSTRAINT_H
#define HPP_GIK_PARALLEL_CONSTRAINT_H

#include "gikTask/jrlGikParallelConstraint.h"
#include "constraints/hppGikSingleMotionElementConstraint.h"



/**
\brief Specify a 2D orientation constraint on a body of the robot.
 */

class ChppGikParallelConstraint:public CjrlGikParallelConstraint, public ChppGikSingleMotionElementConstraint
{
public:
    /**
    \name Definition of the constraint
    @{
     */

    /**
    \brief Constructor
     */
    ChppGikParallelConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint,const vector3d& inLocalVector,  const vector3d& inTargetVector);

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
    \brief Set the vector (in joint's local frame) associated to the constraint.
     */
    virtual void  localVector(const vector3d& inVector);
    /**
    \brief Get the vector associated to the constraint (in joint's local frame).
     */
    virtual const vector3d& localVector();
    /**
    \brief Set the target vector associated to the constraint (in world's frame).
     */
    virtual void  targetVector(const vector3d& inVector);
    /**
    \brief Get the target vector associated to the constraint (in world's frame).
     */
    virtual const vector3d& targetVector();

    /**
    \brief Same as targetvector but take / return ublas matrix
     */
    virtual void  targetVectorU(const vectorN& inVector);
    virtual const vectorN& targetVectorU();


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
    \brief Get the constraint jacobian
     */
    virtual const matrixNxP& jacobian();

    /**
    @}
     */

    /**
    \brief Destructor
     */
    virtual ~ChppGikParallelConstraint()
    {}

private:

    CjrlDynamicRobot* attRobot;

    CjrlJoint* attJoint;


    vector3d attLocalVectorVector3;

    vector3d attTargetVectorVector3;


    vectorN attLocalVector;

    vectorN attTargetVector;


    /** \name Computation temporary variables (to avoid dynamic allocation)
    {@
     */
    vectorN attInfluencingDofs;
    vectorN attValue;
    matrixNxP attJacobian;
    
    unsigned int tempNumJoints;
    matrixNxP tempRot;
    matrixNxP tempRot1;
    matrixNxP tempRot2;
    matrixNxP tempRot3;
    vectorN temp3DVec;
    vectorN temp3DVec1;
    CjrlJoint* tempFixedJoint;
    const matrixNxP* tempFixedJointJacobian;
    const matrixNxP* tempEffectorJointJacobian;
    matrixNxP tempJointOrientJacobian;
    /**
    @}
     */
};

#endif
