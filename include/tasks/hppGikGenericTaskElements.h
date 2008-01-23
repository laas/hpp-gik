#ifndef HPP_GIK_GENERICTASK_ELEMENTS_H
#define HPP_GIK_GENERICTASK_ELEMENTS_H

#include "constraints/hppGikPlannableConstraint.h"
#include "constraints/hppGikTransformationConstraint.h"
#include "constraints/hppGikMotionConstraint.h"
#include "robot/hppGikFootprintRelated.h"
#include "core/hppGikMotionPlan.h"

/**
\brief 
 */
class ChppGikPrioritizedStateConstraint
{
public:

    ChppGikPrioritizedStateConstraint(CjrlGikStateConstraint* inStateConstraint, unsigned int inPriority);

    CjrlGikStateConstraint* stateConstraint();

    unsigned int priority();

    ~ChppGikPrioritizedStateConstraint();

private:
    CjrlGikStateConstraint* attStateConstraint;
    unsigned int attPriority;
};

/**
\brief 
 */
class ChppGikPrioritizedMotionConstraint
{
public:

    ChppGikPrioritizedMotionConstraint(CjrlGikMotionConstraint* inMotionConstraint, unsigned int inPriority);

    CjrlGikMotionConstraint* motionConstraint();

    unsigned int priority();

    ~ChppGikPrioritizedMotionConstraint();

private:
    CjrlGikMotionConstraint* attMotionConstraint;
    unsigned int attPriority;
};

/**
\brief A GenericTaskElement is a motion planning task defined by a time block : a start time and a duration, and a mask on the robot joints. It should represent an elementary motion planning task that does not require further subtask planning.
 */
class ChppGikGenericTaskElement
{
public:
    /**
    \brief Set the joint mask put to work
     */
    void workingJoints(vectorN& inJointsMask)
    {
        attWorkingJoints = inJointsMask;
    }
    /**
    \brief Get the joint mask put to work
     */
    const vectorN& workingJoints() const
    {
        return attWorkingJoints;
    }
    /**
    \brief Destructor
     */
    virtual ~ChppGikGenericTaskElement()
    {}

protected:
    vectorN attWorkingJoints;
};

/**
\brief A single motion element is a motion planning task that produces a single motion constraint, based on a target state constraint, a priority of resolution, a start time and an end time.
 */
class ChppGikSingleMotionElement: public ChppGikGenericTaskElement, public CjrlGikMotionConstraint
{
public:
    /**
    \brief constructor
     */
    ChppGikSingleMotionElement(ChppGikPlannableConstraint* inTargetConstraint, unsigned int inPriority, double inStartTime=0.0, double inDuration=0.0);

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
    \brief Get the priority
     */
    unsigned int priority();

    /**
    \brief Get the target constraint
    */
    ChppGikPlannableConstraint* targetConstraint();

    /**
    \brief plan the motion constraint for this task. This automatically calls dereferMotion first.
    */
    virtual bool planMotion(double inSamplingPeriod);

    /**
    \brief add a pointer to the planned motion in the given motion plan.
    \return false if motion is already referred in a motion plan (in that case you have call dereferMotion first)
    */
    bool referMotion(ChppGikMotionPlan* inMotionPlan);

    /**
    \brief remove the pointer to the planned motion motion from the previously given motion plan if any.
    \return false if the motion is already dereferenced in the motion plan
    */
    bool dereferMotion();

    /**
    \brief Get a pointer to this Element's motion
    */
    CjrlGikMotionConstraint* motion();

    /**
    \brief Destructor
    */
    ~ChppGikSingleMotionElement();


protected:

    virtual bool planningAlgorithm();

    double attStartTime;

    double attEndTime;

    double attDuration;

    unsigned int attPriority;

    double attSamplingPeriod;

    double attEps;

    vectorN attTarget;

    vectorN attSample;

    ChppGikMotionPlanRow* attMotionRow;

    CjrlGikMotionConstraint* attPlannedMotion;

    ChppGikPlannableConstraint* attConstraint;

private:

    ublas::matrix<double, ublas::column_major > attInterpolationData;
    vectorN attInterpolationLine;

};


/**
\brief A locomotion element is a motion planning task that produces threes motion chunks: a 2D Zero Momentum Point motion, a foot motion constraint and a support polygon motion.
 */
class ChppGikLocomotionElement:public ChppGikGenericTaskElement
{
public:

    /**
    \brief Get the time when the solution motions to this task should start
     */
    virtual double startTime() = 0;
    /**
    \brief Get the time available to solve the task
     */
    virtual double duration() =0 ;

    virtual double endTime()
    {
        return startTime() + duration();
    }

    /**
    \brief Plan stability-consistent motions for support polygon, zmp and non support foot.
    The planned motion chunks are assumed to start with a static state (null velocity, null acceleration). The final planned state is static.
     */
    virtual bool planMotions(ChppGikSupportPolygonMotion& outSupportPolygonMotion, ChppGikMotionConstraint& outFootMotion, matrixNxP& outZMPmotion, double inSamplingPeriod) = 0;

    /**
    \brief Destructor
     */
    virtual ~ChppGikLocomotionElement()
    {}
}
;


/**
\brief A locomotion element for shifting the body mass by planning a linear zmp motion inside the support polygon.
 */
class ChppGikZMPshiftElement:public ChppGikLocomotionElement
{
public:

    /**
    \brief Constructor
     */
    ChppGikZMPshiftElement(double inNewZMPx, double inNewZMPy, double inStartTime, double inDuration);
    /**
    \brief Get the time when the solution motions to this task should start
     */
    double startTime();
    /**
    \brief Get the time available to solve the task
     */
    virtual double duration();
    /**
    \brief Plan a linear zmp motion inside the  support polygon.
    The planned motion chunks are assumed to start with a static state (null velocity, null acceleration). The final planned state is static.
    Prerequisites:
        0)The passed output motions contain at least one element (taken as initial state)
        a)Last planned support polygon is double 
        b)Target ZMP is in support polygon
    \return false if prerequisites not met. (only 0) and a) are checked as for now)
     */
    virtual bool planMotions(ChppGikSupportPolygonMotion& outSupportPolygonMotion, ChppGikMotionConstraint& outFootMotion, matrixNxP& outZMPmotion, double inSamplingPeriod);

    /**
    \brief Destructor
     */
    ~ChppGikZMPshiftElement();


private:

    double attStartTime;
    double attDuration;
    vectorN attTargetZMP;
};



/**
\brief A locomotion element for displacing a foot to the given footprint
 */
class ChppGikFootDisplaceElement:public ChppGikLocomotionElement
{
public:

    /**
    \brief Constructor
     */
    ChppGikFootDisplaceElement(double inStartTime, ChppGikFootprint * inTargetFootprint, bool isRight, double inDuration, double inHeight=0.05);

    /**
    \brief Tell if this step is to be done by the right foot
     */
    bool isRight() const;
    /**
    \brief set moving foot
     */
    void selectFoot(bool inIsRight);

    /**
    \brief Set the foot displace duration
     */
    void duration(double inTime);

    /**
    \brief Get the duration of motion of the stepping foot.
     */
    double height() const;

    /**
    \brief Set the foot displace height
     */
    void height(double inHeight);

    /**
    \brief Get the target footprint
     */
    const ChppGikFootprint& targetFootprint() const;
    /**
    \brief Set the target footprint (copied locally)
     */
    void targetFootprint(ChppGikFootprint* inTarget);

    /**
    \brief Plan for motions required for the foot displacement
    The planned motion chunks are assumed to start with a static state (null velocity, null acceleration). The final planned state is static.
    Prerequisites:
        a)Initial ZMP is under support foot
        b)Initial support polygon is double
    \return false if prerequisites not met. ( only b) is checked as for now)
     */
    virtual bool planMotions(ChppGikSupportPolygonMotion& outSupportPolygonMotion, ChppGikMotionConstraint& outFootMotion, matrixNxP& outZMPmotion, double inSamplingPeriod);

    /**
    \brief Get the start time
     */
    virtual double startTime();
    /**
    \brief Get the time available to solve the task
     */
    virtual double duration();
    /**
    \brief Destructor
     */
    ~ChppGikFootDisplaceElement();

protected:

    /**
    \brief Height of step
     */
    double attHeight;
    /**
    \brief True is the stepping foor is the right
     */
    bool attIsRight;
    /**
    \brief The target of the moving foot defined by the configuration of its target footprint: X Y and Theta (yaw)
     */
    ChppGikFootprint* attTargetFootprint;

    /**
    \brief the time when the solution motion to this task should start
    */
    double attStartTime;
    /**
    \brief The duration of the foot movement
     */
    double attDuration;

private:

    matrixNxP tempRot;

};

/**
\brief A ChppGikStepElement is a FootDisplaceElement preceded by a ZMPshiftElement and ended by another ZMP shift element
The additional parameters for the constructor are
    -a startZMPshift duration (which can be zero),
    -an endZMPshift duration (which can be zero) 
    -a coefficent in [0,1] indicating a point inside the double support polygonm where the final zmp would be.
 */
class ChppGikStepElement:public ChppGikFootDisplaceElement
{
public:

    /**
    \brief Constructor. The minimum time for the step motion (i.e parameter inFlightTime) is 0.5. So if the user constructs a step with anything below, it gets automatically thresholded.
     */
    ChppGikStepElement(double inStartTime, ChppGikFootprint* inFootprint, bool isRightFoot, double inFinalZMPCoefficient=0.75, double inEndShiftTime=0.8, double inStartZMPShiftTime=0.6, double inFootMotionDuration=0.8, double inStepHeight=0.05);

    /**
    \brief Constructor. The minimum time for the step motion (i.e parameter inFlightTime) is 0.5. So if the user constructs a step with anything below, it gets automatically thresholded.
     */
    ChppGikStepElement(ChppGikFootprint* inFootprint, double inStartTime, bool isRightFoot, double rightfoot2TargetZMPX, double rightfoot2TargetZMPY, double inEndShiftTime=0.8, double inStartZMPShiftTime=0.6, double inFootMotionDuration=0.8, double inStepHeight=0.05);

    /**
    \brief Get the time taken to shift the weight of the body from the moving foot to the support foot
     */
    double startShiftTime() const;
    /**
    \brief Set the start shift duration
     */
    void startShiftTime(double inTime);

    /**
    \brief Get the time taken to shift the weight of the body from the support foot to a new position lying between it and the moved feet
     */
    double endShiftTime() const;
    /**
    \brief Set the end shift duration
     */
    void endShiftTime(double inTime);

    /**
    \brief Get the coefficient that defines the final position of the ZMP in the final support polygon. A value of 0 means the ZMP (and thus the COM when the body is still) stays under the support foot's ankle. A value of one makes the ZMP planner move the ZMP under the ankle of the moved foot.
     */
    double finalZMPCoef() const;

    /**
    \brief set the final ZMP factor (see above for explanations)
     */
    void finalZMPCoef(double inZMPCoef);

    /**
    \brief Plan the stepping motions
    The planned motion chunks are assumed to start with a static state (null velocity, null acceleration). The final planned state is static.
    Prerequisite: last planned support polygon is double.
    \return false if the prerequisite is not met.
     */
    virtual bool planMotions(ChppGikSupportPolygonMotion& outSupportPolygonMotion, ChppGikMotionConstraint& outFootMotion, matrixNxP& outZMPmotion, double inSamplingPeriod );

    /**
    \brief Get the start time
     */
    virtual double startTime();
    /**
    \brief Get the time available to solve the task
     */
    virtual double duration();

    /**
    \brief Destructor
     */
    ~ChppGikStepElement();

private:

    /**
    \brief The time that is taken to shift the weight of the body from the moving foot to the support foot
     */
    double attStartShiftTime;
    /**
    \brief The time that is taken to shift the weight of the body from the support foot to a new position lying between it and the moved feet
     */
    double attEndShiftTime;
    /**
    \brief The coefficient that defines the final position of the ZMP in the final support polygon. A value of 0 means the ZMP (and thus the COM when the body is still) stays under the support foot's ankle. A value of one makes the ZMP planner move the ZMP under the ankle of the moved foot.
     */
    double attFinalZMPCoef;

    /**
    \brief 
    */
    bool attUseZMPcoefficient;

    /**
    \brief
    */
    double attRfoot2TargetZMPX;

    /**
    \brief
     */
    double attRfoot2TargetZMPY;
};


/**
\brief A locomotion element for moving a foot according to a custom foot motion (i.e. does not have to be a stepping).
The entered custom foot motion should converge to a static state.
 */
class ChppGikCustomFootMotionElement:public ChppGikLocomotionElement
{
public:

    ChppGikCustomFootMotionElement(double inStartTime, ChppGikMotionConstraint* inFootMotion)
    {}
    /**
    \brief Get the time when the solution motion to this task should start
    */
    double startTime()
    {
        return 0;
    }
    /**
    \brief Get the time available to solve the task
     */
    virtual double duration()
    {
        return 0;
    }

    /**
    \brief (not implemented yet) Append a custom foot motion constraint computed by the user.
    Prerequisite: 
        a)ZMP under support foot
    \return false if Prequisites not met
     */
    virtual bool planMotions(ChppGikSupportPolygonMotion* outSupportPolygonMotion, ChppGikMotionConstraint* outFootMotion, matrixNxP& outZMPmotion, double inSamplingPeriod)
    {
        return false;
    }
    /**
    \brief Destructor
     */
    ~ChppGikCustomFootMotionElement()
    {}

}
;

#endif
