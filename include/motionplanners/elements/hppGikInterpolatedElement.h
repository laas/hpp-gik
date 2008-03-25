#ifndef HPP_GIK_INTERPOLATED_ELEMENT_H
#define HPP_GIK_INTERPOLATED_ELEMENT_H

#include "constraints/hppGikVectorizableConstraint.h"
#include "core/hppGikPrioritizedMotion.h"
#include "core/hppGikMotionPlanRow.h"
#include "gikTask/jrlGikMotionConstraint.h"

/**
\brief An object that plans a motion a constraint based on a given target constraint
 */
class ChppGikInterpolatedElement: public ChppGikPrioritizedMotion, public CjrlGikMotionConstraint
{
public:
    /**
        \brief constructor
     */
    ChppGikInterpolatedElement(CjrlHumanoidDynamicRobot* inRobot, ChppGikVectorizableConstraint* inTargetConstraint, unsigned int inPriority, double inStartTime, double inDuration, double inSamplingPeriod);

    /**
    \brief Get a pointer to associated robot
     */
    CjrlDynamicRobot* robot();
    
    /**
        \brief clone constructor
     */
    CjrlGikMotionConstraint* clone() const;

    /**
        \brief Get state constraint at a given time.
        \return null pointer if time out of bounds or if motion not planned yet
     */
    CjrlGikStateConstraint* stateConstraintAtTime ( double inTime );

    /**
        \brief reset the starting time.
     */
    virtual void startTime(double inStartTime);

    /**
        \brief Get the time when the solution motion to this task should start
     */
    virtual double startTime();

    /**
        \brief Get the time available to solve the task
     */
    virtual double endTime();



    /**
        \brief Get the time available to solve the task
     */
    virtual double duration();

    /**
        \brief Get the target constraint
     */
    ChppGikVectorizableConstraint* targetConstraint();

    /**
        \brief Destructor
     */
    ~ChppGikInterpolatedElement();


private:

    virtual bool planMotion();
    virtual bool planningAlgorithm();
    
    double attSamplingPeriod,attEps;
    double attStartTime, attDuration, attEndTime;
    vectorN attTarget;
    vectorN attSample;
    ChppGikMotionPlanRow* attMotionRow;
    ChppGikVectorizableConstraint* attConstraint;
    ublas::matrix<double, ublas::column_major > attInterpolationData;
    vectorN attInterpolationLine;
    bool attPlanSuccess;

};


#endif

