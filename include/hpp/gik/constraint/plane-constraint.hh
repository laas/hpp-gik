#ifndef HPP_GIK_PLANE_CONSTRAINT_H
#define HPP_GIK_PLANE_CONSTRAINT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikPlaneConstraint.h"
#include "hpp/gik/constraint/joint-state-constraint.hh"
#include "hpp/gik/constraint/vectorizable-constraint.hh"

/**
\brief Description of a constraint that limits the position of a point of the robot to a given plan.
\ingroup constraints
*/

class ChppGikPlaneConstraint:  public CjrlGikPlaneConstraint, public ChppGikJointStateConstraint, public ChppGikVectorizableConstraint
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
    \brief Get the full state (this constraint and its first two derivatives) of the constraint expressed as a vectorN.
     */
    virtual void computeVectorizedState();


    /**
    \brief Compute the target of the constraint as a vectorN.
     */
    virtual void computeVectorizedTarget();

    /**
    \brief A unified method to change the target of the constraint with a vectorN.
    \return false if the argument's size does not match the one expected for this constraint type.
    \param inTarget must be of size 6. First three coordinates are for the target plane's point and the last three are for the target plane's normal
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

    matrixNxP tempJacobian;
    matrixNxP tempJointPositionJacobian;
    /**
    @}
     */
};

#endif
