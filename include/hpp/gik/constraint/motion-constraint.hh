#ifndef HPP_GIK_MOTION_CONSTRAINT_H
#define HPP_GIK_MOTION_CONSTRAINT_H


#include "jrl/mal/matrixabstractlayer.hh"
#include "gikTask/jrlGikMotionConstraint.h"


/**
\brief In this implementation of CjrlGikMotionConstraint, the motion is a vector of CjrlGikStateConstraint which are samples at the sampling rate entered in the constructor. In the current state, when a stateconstraint is requested at a given time, the nearest sample found is returned. a more precise implementation could return an interpolation of the bounding samples.
The default start time is 0.
 */

class ChppGikMotionConstraint: public CjrlGikMotionConstraint
{
public:

    /**
    \brief Constructor
     */
    ChppGikMotionConstraint(CjrlDynamicRobot* inRobot, double inSamplingPeriod, double inStartTime = 0.0);
    
    /**
    \brief get the associated robot
    */
    CjrlDynamicRobot* robot();
    
    /**
    \brief Clone Constructor
     */
    CjrlGikMotionConstraint* clone() const;
    
    /**
    \brief Append a state constraint at the end of the motion and update the end time (A copy of the given state constraint object is made).
     */
    void pushbackStateConstraint(const CjrlGikStateConstraint* inStateConstraint);
    /**
    \brief Append a state constraint at the beginning of the motion and update the start time (A copy of the given state constraint object is made).
     */
    void pushfrontStateConstraint(const CjrlGikStateConstraint* inStateConstraint);
    /**
    \brief Get state constraint at a given time.
     */
    virtual CjrlGikStateConstraint* stateConstraintAtTime(double inTime);
    /**
    \brief Get state constraint at a given rank.
     */
    CjrlGikStateConstraint* stateConstraintAtRank(unsigned int inRank) const;
    /**
    \brief Get the number of state constraints contained in this motion constraint
     */
    unsigned int numberStateConstraints();
     /**
    \brief Tell if support vector motion is empty
      */
    bool empty();
    /**
    \brief Empty the Motion Constraint and reset start and end times
     */
    void clear();
    /**
    \brief Set lower bound of definition interval.
     */
    void startTime(double inStartTime);

    /**
    \brief Get lower bound of definition interval.
     */
    double startTime();
    /**
    \brief Get upper bound of definition interval.
     */
    double endTime();
    /**
    \brief Get the sampling period
     */
    double samplingPeriod();
    /**
    \brief Destructor
     */
    ~ChppGikMotionConstraint();

private:
    std::vector<CjrlGikStateConstraint*> attVectorStateConstraint;

    double attSamplingPeriod, attEps;

    double attStartTime;

    double attEndTime;
    CjrlDynamicRobot* attRobot;
    
};


#endif
