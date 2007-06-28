/*
        Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
        Authors: Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef HPP_GIK_PLANE_CONSTRAINT_H
#define HPP_GIK_PLANE_CONSTRAINT_H

#include "gikTask/jrlGikPlaneConstraint.h"
#include "constraints/hppGikSingleMotionElementConstraint.h"


/**
\brief Description of a constraint that limits the position of a point of the robot to a given plan.
*/

class ChppGikPlaneConstraint:public CjrlGikPlaneConstraint, public ChppGikSingleMotionElementConstraint
{
public:
    /**
    \name Definition of the constraint
    @{
     */

    /**
    \brief Constructor
     */
    ChppGikPlaneConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inWorldPlanePoint, const vector3d& inWorldPlaneNormal);

    /**
    \brief Copy the object
    */
    virtual CjrlGikStateConstraint * clone() const;

    /**
    \brief Get the dimension of the constraint.
     */
    virtual unsigned int dimension() const;

    /**
    \brief Get robot associated to the constraint.
     */
    virtual CjrlHumanoidDynamicRobot& robot();

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
    \brief Set a point of the target plane (in world's frame).
     */
    virtual void  worldPlanePoint(const vector3d& inPoint);
    /**
    \brief Get the point of the defined plane (in world's frame).
     */
    virtual const vector3d& worldPlanePoint();
    /**
    \brief Set a point of the target plane (in world's frame).
     */
    virtual void  worldPlaneNormal(const vector3d& inPoint);
    /**
    \brief Get the point of the defined plane (in world's frame).
     */
    virtual const vector3d& worldPlaneNormal();


    /**
    \brief Same as above methods but take /return ublas vectors
     */
    virtual void  worldPlanePointU(const vectorN& inPoint);
    virtual const vectorN& worldPlanePointU();
    virtual void  worldPlaneNormalU(const vectorN& inPoint);
    virtual const vectorN& worldPlaneNormalU();

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
    First the current position velocity and acceleration of the constraint are computed using outputs from the CjrlHumanoidDynamicRobot.
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
    virtual ~ChppGikPlaneConstraint()
    {}

private:

    CjrlHumanoidDynamicRobot* attRobot;

    CjrlJoint* attJoint;


    vector3d attLocalPointVector3;

    vector3d attWorldPlanePointVector3;

    vector3d attWorldPlaneNormalVector3;


    vectorN attLocalPoint;

    vectorN attWorldPlanePoint;

    vectorN attWorldPlaneNormal;


    /** \name Temporary variables (to avoid dynamic allocation)
    {@
     */
    vectorN attInfluencingDofs;
    vectorN attValue;
    matrixNxP attJacobian;

    unsigned int tempNumJoints;
    vectorN temp3DVec;
    vectorN temp3DVec1;
    matrixNxP tempRot;
    CjrlJoint* tempFixedJoint;
    const matrixNxP* tempFixedJointJacobian;
    const matrixNxP* tempEffectorJointJacobian;
    matrixNxP tempJointPositionJacobian;
    /**
    @}
     */
};

#endif
