#ifndef HPP_GIK_PARALLEL_CONSTRAINT_H
#define HPP_GIK_PARALLEL_CONSTRAINT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "constraints/hppGikJointStateConstraint.h"
#include "gikTask/jrlGikParallelConstraint.h"

/**
\brief Specify a 2D orientation constraint on a body of the robot.
 */

class ChppGikParallelConstraint: public CjrlGikParallelConstraint, public ChppGikJointStateConstraint
{
public:
    /**
    \brief Constructor
     */
    ChppGikParallelConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint,const vector3d& inLocalVector,  const vector3d& inTargetVector);

    /**
    \brief Copy the object
     */
    virtual CjrlGikStateConstraint* clone() const;
   
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
    @}
     */

    /**
    \brief Destructor
     */
    virtual ~ChppGikParallelConstraint()
    {}

private:

    vector3d attLocalVectorVector3;

    vector3d attTargetVectorVector3;


    vectorN attLocalVector;

    vectorN attTargetVector;


    /** \name Computation temporary variables (to avoid dynamic allocation)
    {@
     */
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
