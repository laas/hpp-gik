#ifndef HPP_GIK_HALFSITTING_TASK_BETA_H
#define HPP_GIK_HALFSITTING_TASK_BETA_H

#include "tasks/hppGikHalfSittingTask.h"

/**
\brief This robot task consists in going back to half-sitting stance using gik only (no interpolation).
 */
class ChppGikHalfSittingTaskBeta : public ChppGikHalfSittingTask
{
    public:

        ChppGikHalfSittingTaskBeta(ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod):ChppGikHalfSittingTask(inStandingRobot,inSamplingPeriod){}
        
    protected:
        
        virtual bool algorithmSolve();

};
#endif

