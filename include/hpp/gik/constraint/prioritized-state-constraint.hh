#ifndef HPP_GIK_PRIO_CONSTRAINT_H
#define HPP_GIK_PRIO_CONSTRAINT_H

#include "gikTask/jrlGikStateConstraint.h"

/**
\brief 
 */
class ChppGikPrioritizedStateConstraint
{
public:

    ChppGikPrioritizedStateConstraint(CjrlGikStateConstraint* inStateConstraint, unsigned int inPriority)
    {
        attStateConstraint = inStateConstraint;
        attPriority = inPriority;
    }

    CjrlGikStateConstraint* stateConstraint()
    {
        return attStateConstraint;
    }
    
    unsigned int priority()
    {
        return attPriority;
    }

    ~ChppGikPrioritizedStateConstraint(){}

private:
    CjrlGikStateConstraint* attStateConstraint;
    unsigned int attPriority;

};

#endif
