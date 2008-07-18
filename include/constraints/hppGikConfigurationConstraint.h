#ifndef HPP_GIK_CONF_CONSTRAINT_H
#define HPP_GIK_CONF_CONSTRAINT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "constraints/hppGikVectorizableConstraint.h"


/**
\brief Specify a full or partial configuration constraint
\ingroup constraints
 */

class ChppGikConfigurationConstraint:public ChppGikVectorizableConstraint
{
public:
    /**
    \brief Constructor. inTargetConfig is of size  inRobot.numberDof() and inMask of size inRobot.numberDof()-6 (i.e. excluding free flyer's dofs)
     */
    ChppGikConfigurationConstraint(CjrlDynamicRobot& inRobot, const vectorN& inTargetConfig,  const vectorN& inMask);

    /**
        \brief Destructor
     */
    ~ChppGikConfigurationConstraint();

    /**
        \brief Copy
     */
    virtual CjrlGikStateConstraint* clone() const;

    /**
    \brief set target
    \return false if size of vector incorrect
    */
    bool target(const vectorN& inTargetConfig);

    /**
        \brief Get associated robot
     */
    virtual CjrlDynamicRobot& robot();

    /**
        \brief Get the dimension of the constraint.
     */
    virtual unsigned int dimension() const;



    /**
        \brief Compute a binary vector whose size matches the robot cnfiguration's, where an element with value 1 indicates that the corresponding degree of freedom can modify the value of this constraint, and an element with value 0 cannot.
     */
    virtual void computeInfluencingDofs();

    /**
        \brief Get the influencing dofs
     */
    virtual vectorN& influencingDofs();

    /**
        \brief Compute the value of the constraint.
     */
    virtual void computeValue();

    /**
        \brief Compute the Jacobian matrix of the constraint value wrt internal configuration variables.
        The interaction with the environment is taken into account (for instance a foot on the ground).
     */
    virtual void computeJacobian();

    /**
        \brief Get the constraint value.
     */
    virtual const vectorN& value();

    /**
        \brief Get the constraint Jacobian wrt internal configuration variables.
        The interaction with the environment is taken into account (for instance a foot on the ground)
     */
    virtual const matrixNxP& jacobian();
    
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


private:

    CjrlDynamicRobot* attRobot;
    unsigned int attNumberActuatedDofs, attDimension;
    vectorN attTargetConfiguration, attTarget;
    vectorN attValue;
    vectorN attInfluencingDofs;
    matrixNxP attJacobian;

};

#endif
