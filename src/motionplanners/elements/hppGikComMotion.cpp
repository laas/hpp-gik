#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "hppGikTools.h"
#include "motionplanners/elements/hppGikComMotion.h"


ChppGikComMotion::ChppGikComMotion(CjrlHumanoidDynamicRobot* inRobot, double inStartTime, double inSamplingPeriod, const vectorN& inWorkingJoints, unsigned int inPriority):ChppGikPrioritizedMotion(inRobot, inPriority,this)
{
    vector3d pcom;
    pcom = attRobot->positionCenterOfMass();
    attComConstraint = new ChppGikComConstraint( *attRobot, pcom[0], pcom[1]);
    attStartTime = attEndTime = inStartTime;
    attSamplingPeriod = inSamplingPeriod;
    attEps = attSamplingPeriod/2;
    if (inWorkingJoints.size() == inRobot->numberDof())
        attWorkingJoints = inWorkingJoints;
}

ChppGikComMotion::~ChppGikComMotion()
{
    delete attComConstraint;
}

CjrlDynamicRobot* ChppGikComMotion::robot()
{
    return attRobot;
}


CjrlGikMotionConstraint* ChppGikComMotion::motionConstraint()
{
    return this;
}


CjrlGikMotionConstraint* ChppGikComMotion::clone() const
{
    return new ChppGikComMotion(attRobot, attStartTime, attSamplingPeriod, attWorkingJoints, attPriority);
}


CjrlGikStateConstraint* ChppGikComMotion::stateConstraintAtTime(double inTime)
{
    unsigned int i = ChppGikTools::timetoRank(attStartTime,inTime,attSamplingPeriod);
    
    if ((i > 0) && (i < attSamples.size2()))
    {
        attComConstraint->targetXY( attSamples(0,i), attSamples(1,i));
        return attComConstraint;
    }
    else
        return 0;
}


void ChppGikComMotion::startTime(double inStartTime)
{
    attStartTime = (inStartTime>0.0)?inStartTime:0.0;
}


double ChppGikComMotion::startTime()
{
    return attStartTime;
}


double ChppGikComMotion::endTime()
{
    return attEndTime;
}


bool ChppGikComMotion::setSamples(const matrixNxP& inSamples)
{
    if (inSamples.size1() != 2)
        return false;
    attSamples = inSamples;
    attEndTime = attStartTime + (inSamples.size2()-1)*attSamplingPeriod;
    return true;
}


