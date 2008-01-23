#ifndef HPP_GIK_LP_CONSTRAINT_H
#define HPP_GIK_LP_CONSTRAINT_H

#include <vector>
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikStateConstraint.h"


/**
\brief A constraint on the waist to maintain its projection on the ground at the middle of the ankles'
 */
class ChppGikLPConstraint: public CjrlGikStateConstraint
{
public:

    /**
        brief Constructor
     */
    ChppGikLPConstraint(CjrlHumanoidDynamicRobot& inRobot);

    void movingFoot(CjrlJoint* inJoint);
    
    CjrlGikStateConstraint* clone() const;

    CjrlDynamicRobot& robot();

    unsigned int dimension() const;

    void computeInfluencingDofs();

    vectorN& influencingDofs();

    void computeValue();

    void computeJacobian();

    const vectorN& value();

    const matrixNxP& jacobian();

    /**
    \brief Destructor
     */
    ~ChppGikLPConstraint();

private:

    /**
    \brief The robot on which this constraint is applied
     */
    CjrlHumanoidDynamicRobot* attRobot;

    unsigned int attDimension;
    
    CjrlJoint* attMovingFoot, *attFixedFoot;

    /**
    \brief The value of this constraint
     */
    vectorN attValue;

    /**
    \brief The Jacobian of the constraint
     */
    matrixNxP attJacobian, attTempJacobian;

    /**
    \brief The degrees of freedom influencing this constraint from (and including) the root
     */
    vectorN attInfluencingDofs;

    /**
    \brief The number of actuated degrees of freedom 
     */
    unsigned int attNumberActuatedDofs;


};

#endif
