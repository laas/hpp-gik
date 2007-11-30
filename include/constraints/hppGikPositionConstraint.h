#ifndef HPP_GIK_POSITION_CONSTRAINT_H
#define HPP_GIK_POSITION_CONSTRAINT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikPositionConstraint.h"
#include "constraints/hppGikJointStateConstraint.h"

/**
        \brief Specify a 3D position constraint on a point of the robot.
 */

class ChppGikPositionConstraint: public CjrlGikPositionConstraint, public ChppGikJointStateConstraint
{
public:

    /**
    \brief Constructor
     */
    ChppGikPositionConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inPointInWorldFrame);


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
    \brief Same as worldTarget but take/return ublas vector
     */
    virtual void  worldTargetU(const vectorN& inPoint);
    virtual const vectorN& worldTargetU();

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
        \brief Destructor
     */
    virtual ~ChppGikPositionConstraint()
    {}

private:

    vector3d attLocalPointVector3;

    vector3d attWorldTargetVector3;


    vectorN attLocalPoint;

    vectorN attWorldTarget;


    /** \name Computation temporary variables (to avoid dynamic allocation)
    {@
     */
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
