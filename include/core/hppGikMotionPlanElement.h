#ifndef HPP_GIK_MOTION_PLAN_ELEMENT_H
#define HPP_GIK_MOTION_PLAN_ELEMENT_H

#include <stdlib.h>
#include <string>
#include "gikTask/jrlGikStateConstraint.h"


/**
\brief A "motion element" is a container for state constraints that share the same priority. A motion element is equivalent to a generelized inverse kinematics task in the literature.
This is a low level object.
 */
class ChppGikMotionPlanElement: public CjrlGikStateConstraint
{

public:

    /**
    \brief Constructor.
     */
    ChppGikMotionPlanElement(CjrlDynamicRobot* inRobot, unsigned int inPriority);
   
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
    \brief Get the dimension of this motion plan element
     */
    unsigned int dimension() const;

    /**
    \brief Clear stored references to constraints
    */
    void clear();
    /**
    \name Computations
     @{
     */
    
    /**
    \brief Get a binary vector which size matches the robot cnfiguration's, where an element with value 1 indicates that the corresponding degree of freedom can modify the value of this constraint, and an element with value 0 cannot.
     */
    vectorN& influencingDofs();
    
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
    const std::vector<CjrlGikStateConstraint*>& constraints(){
      return attConstraints;
    }

    ~ChppGikMotionPlanElement();


private:

    /**
    \brief robot
    */
    CjrlDynamicRobot* attRobot;
    /**
    \brief  Priority
    */
    unsigned int attPriority;
    /**
    \brief Vector of state constraints.
     */
    std::vector<CjrlGikStateConstraint*> attConstraints;
    /**
    \brief Dimension
     */
    unsigned int attDimension;
    /**
    \brief Value
     */
    vectorN attValue;
    /**
    \brief Jacobian wrt to internal degrees of freedom (takes into account one non slippery contact with a fixed part of the world)
     */
    matrixNxP attJacobian;

    vectorN attInfluencingDofs;
};

#endif
