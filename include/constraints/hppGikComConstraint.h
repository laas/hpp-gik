#ifndef HPP_GIK_COM_CONSTRAINT_H
#define HPP_GIK_COM_CONSTRAINT_H

#include "gikTask/jrlGikComConstraint.h"



/**
\brief Specify a full or partial position constraint on the center of mass.
 */

class ChppGikComConstraint:public CjrlGikComConstraint
{
public:
    /**
    \name Definition of the constraint
    @{
     */

    /**
    \brief Constructor
     */
    ChppGikComConstraint(CjrlDynamicRobot& inRobot, double inX, double inY);
    
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
    \brief Set a constraint on Xcom and Ycom (in world's frame).
     */
    virtual void  targetXY(double inX, double inY);

    /**
    \brief Set constraint on x,y,z of com (in world's frame).
     */
    virtual void  targetXYZ(const vectorN& inTarget);

    /**
    \brief Get the target point associated to the constraint (in world's frame).
     */
    virtual const vectorN& worldTarget();

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
    virtual ~ChppGikComConstraint()
    {}

private:
    
    void dimension(unsigned int inDim);

    CjrlDynamicRobot* attRobot;

    CjrlJoint* attJoint;
    
    unsigned int attDimension;

    vectorN attWorldTarget;

    /** 
    \name Computation temporary variables (to avoid dynamic allocation)
    {@
     */
    vectorN attValue;
    vectorN attInfluencingDofs;
    matrixNxP attJacobian;
 
    
    unsigned int tempNumJoints;
    vectorN temp3DVec;
    vectorN temp3DVec1;
    matrixNxP tempRot;
    CjrlJoint* tempFixedJoint;
    const matrixNxP* tempFixedJointJacobian;
    const matrixNxP* tempEffectorJointJacobian;
    matrixNxP tempJacobian;
    /**
    @}
     */
};

#endif
