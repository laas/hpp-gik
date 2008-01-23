#ifndef HPP_GIK_PLANNABLE_CONSTRAINT_H
#define HPP_GIK_PLANNABLE_CONSTRAINT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikJointStateConstraint.h"

class ChppGikVectorizable
{
public:


    /**
    \brief Get the full state (this constraint and its first two derivatives) of the constraint expressed as a vectorN.
     */
    virtual void computeVectorizedState() = 0;


    /**
    \brief Compute the target of the constraint as a vectorN.
     */
    virtual void computeVectorizedTarget() = 0;


    /**
        \brief Get the full state (this constraint and its first two derivatives) of the constraint expressed as a vectorN.
     */
    virtual const vectorN& vectorizedState() const = 0;

    /**
    \brief A unified method to change the target of the constraint with a vectorN.
    \return false if the argument's size does not match the one expected for this constraint type
     */
    virtual bool vectorizedTarget( const vectorN& inTarget ) = 0;


    /**
    \brief A method to get the target of the constraint expressed as a vectorN.
     */
    virtual const vectorN& vectorizedTarget() const = 0;

    /**
        \brief Compute a linearly interpolated target between current state (excluding derivatives) and target as follows:
        returnedTarget = (1 -lambda) * currentState + lambda *  target.
        \parameter inLambda is automatically clamped between 0 and 1
        It is left the the user discretion to assess the need of calling computeVectorizedState and computeVectorizedTarget before calling this method.
     */
    virtual void computeInterpolatedTarget(double inLambda) = 0;

    /**
    \brief return the interpolation result.
    */
    virtual const vectorN& interpolatedTarget() = 0;
    
    /**
    \brief Destructor
    */
    virtual ~ChppGikVectorizable()
    {}

protected:

    /**
    \brief The vectorial expression of the constraint's target at the moment of computation
     */
    vectorN attVectorizedTarget;

    /**
    \brief The vectorial expression of the constraint's full sdtate at the moment of computation
     */
    vectorN attVectorizedState;

    /**
    \brief Interpolated target
     */
    vectorN attInterpolatedTarget;
    
};

class ChppGikPlannableConstraint: public ChppGikVectorizable, virtual public CjrlGikJointStateConstraint
    {}
;

#endif

