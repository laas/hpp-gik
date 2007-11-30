/*
        Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
        Authors: Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef HPP_GIK_PLANE_CONSTRAINT_H
#define HPP_GIK_PLANE_CONSTRAINT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikPlaneConstraint.h"
#include "constraints/hppGikJointStateConstraint.h"


/**
\brief Description of a constraint that limits the position of a point of the robot to a given plan.
*/

class ChppGikPlaneConstraint:  public CjrlGikPlaneConstraint, public ChppGikJointStateConstraint
{
public:
    /**
    \brief Constructor
     */
    ChppGikPlaneConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inWorldPlanePoint, const vector3d& inWorldPlaneNormal);

    /**
    \brief Copy the object
    */
    virtual CjrlGikStateConstraint * clone() const;
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
    virtual ~ChppGikPlaneConstraint()
    {}

private:

    vector3d attLocalPointVector3;

    vector3d attWorldPlanePointVector3;

    vector3d attWorldPlaneNormalVector3;

    matrixNxP attWorldPlaneNormalTrans;

    vectorN attLocalPoint;

    vectorN attWorldPlanePoint;

    vectorN attWorldPlaneNormal;


    /** \name Temporary variables (to avoid dynamic allocation)
    {@
     */
    vectorN temp3DVec;
    vectorN temp3DVec1;
    matrixNxP tempRot;
    CjrlJoint* tempFixedJoint;
    const matrixNxP* tempFixedJointJacobian;
    matrixNxP tempEffectorJointJacobian;
    matrixNxP tempJointPositionJacobian;
    /**
    @}
     */
};

#endif
