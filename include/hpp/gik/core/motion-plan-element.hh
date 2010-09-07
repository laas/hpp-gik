#ifndef HPP_GIK_MOTION_PLAN_ELEMENT_H
#define HPP_GIK_MOTION_PLAN_ELEMENT_H

#include <stdlib.h>
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikStateConstraint.h"


/**
\brief Use this to concatenate several CjrlGikStateConstraint objects.
\ingroup constraints
\ingroup motionsplan
 */
class ChppGikMotionPlanElement: public CjrlGikStateConstraint
{

public:

    /**
    \brief Constructor.
     */
    ChppGikMotionPlanElement(CjrlDynamicRobot* inRobot, unsigned int inPriority, double inDampingFactor = 0.0);

    /**
    \brief Copy
     */
    CjrlGikStateConstraint* clone() const;

    /**
    \brief Get associated robot
     */
    CjrlDynamicRobot& robot();

    /**
    \brief Get the priority
    */
    unsigned int priority() const;

    /**
    \brief Add a constraint.
     */
    void addConstraint(CjrlGikStateConstraint* inJrlStateConstraint);

    /**
    \brief Remove a constraint. Recomputing new value and jacobian is NOT done automatically.
    \return false if constraint not found
     */
    bool removeConstraint(const CjrlGikStateConstraint* inJrlStateConstraint);

    /**
    \brief Set the damping factor
     */
    void dampingFactor(double inDF);
    
    /**
    \brief Get the damping factor
     */
    double dampingFactor() const;
    
    /**
    \brief Get the dimension of this motion plan element
     */
    unsigned int dimension() const;

    /**
    \brief Get a mask on the configuration vector denoting the working degrees of freedom
     */
    const vectorN& workingJoints() const;

    /**
    \brief Set a mask on the configuration vector denoting the working degrees of freedom
     */
    void workingJoints(const vectorN& inVec);

    /**
    \brief Clear stored references to constraints
    */
    void clear();
    
    void jacobianRoot( CjrlJoint& inJoint);

    /**
    \name Computations
     @{
     */

    /**
    \brief Compute a binary vector whose size matches the robot cnfiguration's, where an element with value 1 indicates that the corresponding degree of freedom can modify the value of this constraint, and an element with value 0 cannot.
    */
    void computeInfluencingDofs();

    /**
    \brief Compute the value of this motion plan element (task).
     */
    void computeValue();
    /**
    \brief Compute the Jacobian of this motion plan element (task) from robot's root taking into account contacts with the environement
    */
    void computeJacobian();
    /**
    @}
     */

    /**
    \name Getting result of computations
    @{
     */
    /**
    \brief Get the influencing dofs
     */
    vectorN& influencingDofs();
    /**
    \brief Get the current value of this motion plan element (task).
     */
    const vectorN& value();
    /**
    \brief Get the current Jacobian
     */
    const matrixNxP& jacobian();

    /**
    @}
     */

    /**
       \brief Get constraints included in this element
     */
    const std::vector<CjrlGikStateConstraint*>& constraints()
    {
        return attConstraints;
    }

    ~ChppGikMotionPlanElement();


private:

    CjrlDynamicRobot* attRobot;
    unsigned int attPriority;
    std::vector<CjrlGikStateConstraint*> attConstraints;
    unsigned int attDimension;
    vectorN attValue;
    matrixNxP attJacobian;
    vectorN attInfluencingDofs;
    vectorN attWorkingJoints;
    double attDampingFactor;
};

#endif
