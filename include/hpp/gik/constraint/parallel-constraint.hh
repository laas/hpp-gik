#ifndef HPP_GIK_PARALLEL_CONSTRAINT_H
#define HPP_GIK_PARALLEL_CONSTRAINT_H

#include "jrl/mal/matrixabstractlayer.hh"
#include "hpp/gik/constraint/joint-state-constraint.hh"
#include "gikTask/jrlGikParallelConstraint.h"
#include "hpp/gik/constraint/vectorizable-constraint.hh"

/**
\brief Specify a 2D orientation constraint on a body of the robot.
\ingroup constraints
 */

class ChppGikParallelConstraint: public CjrlGikParallelConstraint, public ChppGikJointStateConstraint, public ChppGikVectorizableConstraint
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

    matrixNxP tempJointOrientJacobian;
    /**
    @}
     */
};

#endif
