#ifndef HPP_GIK_POINTING_CONSTRAINT_H
#define HPP_GIK_POINTING_CONSTRAINT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikPointingConstraint.h"
#include "constraints/hppGikJointStateConstraint.h"


/**
\brief Constraint on a line segment attached to a body to be aligned with a given point in the world frame. The line segment is defined by an origin point and a vector both given in the body's local frame.
 */
class ChppGikPointingConstraint: virtual public CjrlGikPointingConstraint, public ChppGikJointStateConstraint
{
public:
    /**
        \brief Constructor
     */

    ChppGikPointingConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalOrigin, const vector3d& inLocalVector, const vector3d& inTargetWorldPoint);
    /**
        \brief Copy
     */
    virtual CjrlGikStateConstraint* clone() const;

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
    \brief Same as worldTarget but take/return ublas vector
     */
    void  worldTargetU(const vectorN& inPoint);
    const vectorN& worldTargetU();
    
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
        \brief Destructor
     */
    virtual ~ChppGikPointingConstraint();

protected:

    vector3d attLocalOriginVector3;

    vector3d attLocalVectorVector3;

    vector3d attWorldTargetVector3;


    vectorN attLocalOrigin;

    vectorN attLocalVector;

    vectorN attWorldTarget;


    /**
    \name Computation temporary variables (to avoid dynamic allocation)
    {@
     */
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
