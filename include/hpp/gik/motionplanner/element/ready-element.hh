#ifndef HPP_GIK_READY_ELEMENT_H
#define HPP_GIK_READY_ELEMENT_H

#include "hpp/gik/core/prioritized-motion.hh"

/**
\brief CjrlMotionConstraint + priority + working joints
\ingroup motions
 */
class ChppGikReadyElement:public ChppGikPrioritizedMotion
{
    public:

        ChppGikReadyElement(CjrlHumanoidDynamicRobot* inRobot, CjrlGikMotionConstraint* inMotionConstraint, unsigned int inPriority, const vectorN& inWorkingJoints):ChppGikPrioritizedMotion( inRobot, inPriority, NULL)
        {
            attMotionConstraint =  inMotionConstraint->clone();
            if(inWorkingJoints.size() == attWorkingJoints.size())
                attWorkingJoints = inWorkingJoints;
        }

        ~ChppGikReadyElement()
        {
            delete attMotionConstraint;
        }

};


#endif
