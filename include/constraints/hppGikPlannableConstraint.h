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
    virtual const vectorN& vectorizedState() = 0;

    /**
    \brief A unified method to change the target of the constraint with a vectorN.
    \return false if the argument's size does not match the one expected for this constraint type
     */
    virtual bool vectorizedTarget( const vectorN& inTarget ) = 0;

    /**
    \brief A method to get the target of the constraint expressed as a vectorN.
     */
    virtual const vectorN& vectorizedTarget() = 0;

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
};

class ChppGikPlannableConstraint: public ChppGikVectorizable, virtual public CjrlGikJointStateConstraint
    {}
;

#endif

