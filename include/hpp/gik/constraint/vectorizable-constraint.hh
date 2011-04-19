#ifndef HPP_GIK_VECTORIZABLE_CONSTRAINT_H
#define HPP_GIK_VECTORIZABLE_CONSTRAINT_H

#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"
#include "gikTask/jrlGikStateConstraint.h"


class ChppGikVectorizableConstraint: virtual public CjrlGikStateConstraint
{
public:


    /**
    \brief Get the full state (this constraint and its first two derivatives) of the constraint expressed as a vectorN.
     */
    virtual void computeVectorizedState() = 0;

    /**
        \brief Get the full state (this constraint and its first two derivatives) of the constraint expressed as a vectorN.
     */
    virtual const vectorN& vectorizedState() const
    {
        return attVectorizedState;
    }


    /**
    \brief Compute the target of the constraint as a vectorN.
     */
    virtual void computeVectorizedTarget() = 0;

    /**
    \brief Set the target of the constraint with a vectorN.
    \return false if the argument's size does not match the one expected for this constraint type
     */
    virtual bool vectorizedTarget( const vectorN& inTarget ) = 0;

    /**
    \brief Get the target of the constraint expressed as a vectorN.
     */
    virtual const vectorN& vectorizedTarget() const
    {
        return attVectorizedTarget;
    }


    /**
    \brief Destructor
    */
    virtual ~ChppGikVectorizableConstraint()
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


#endif

