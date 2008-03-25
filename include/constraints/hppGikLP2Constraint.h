#ifndef HPP_GIK_LP2_CONSTRAINT_H
#define HPP_GIK_LP2_CONSTRAINT_H

#include <vector>
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikStateConstraint.h"


/**
\brief A constraint on the waist to maintain its x axis as the mean axis of feet's OR to maintaint the waist's x axis equal non support foot's
 */
class ChppGikLP2Constraint: public CjrlGikStateConstraint
{
    public:

    /**
        brief Constructor
     */
        ChppGikLP2Constraint(CjrlHumanoidDynamicRobot& inRobot);

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
        ~ChppGikLP2Constraint();

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
    
        double attWaistHeight;
    

};

#endif
