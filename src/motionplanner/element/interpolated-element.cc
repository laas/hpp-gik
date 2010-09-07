#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "hpp/gik/tools.hh"
#include "hpp/gik/motionplanner/element/interpolated-element.hh"

using namespace boost::numeric::ublas;

ChppGikInterpolatedElement::ChppGikInterpolatedElement(CjrlDynamicRobot* inRobot, ChppGikVectorizableConstraint* inTargetConstraint, unsigned int inPriority, double inStartTime, double inDuration, double inSamplingPeriod, double inDampingFactor):ChppGikPrioritizedMotion(inRobot, inPriority+2, this, inDampingFactor)
{
    attStartTime = inStartTime;
    attDuration = inDuration;
    attEndTime = inDuration + inStartTime;
    attSamplingPeriod = inSamplingPeriod;
    attEps = inSamplingPeriod/2;

    attConstraint = dynamic_cast<ChppGikVectorizableConstraint*>(inTargetConstraint->clone());
    attConstraint->computeVectorizedTarget();
    attTarget = attConstraint->vectorizedTarget();
    attSample = attTarget;

    if (attStartTime < 0.0)
        startTime( 0.0 );
    attWorkingJoints = inTargetConstraint->influencingDofs();
    attPlanSuccess = false;
}

CjrlDynamicRobot* ChppGikInterpolatedElement::robot()
{
    return attRobot;
}

CjrlGikMotionConstraint* ChppGikInterpolatedElement::clone() const
{
    attConstraint->vectorizedTarget ( attTarget );
    return (CjrlGikMotionConstraint*) new ChppGikInterpolatedElement(attRobot,attConstraint, attPriority, attStartTime, attDuration, attSamplingPeriod);
}

double ChppGikInterpolatedElement::startTime()
{
    return attStartTime;
}

void ChppGikInterpolatedElement::startTime(double inStartTime)
{
    attEndTime = attDuration + inStartTime;
    attStartTime = inStartTime;
}

double ChppGikInterpolatedElement::endTime()
{
    return attEndTime;
}

double ChppGikInterpolatedElement::duration()
{
    return attDuration;
}

ChppGikVectorizableConstraint* ChppGikInterpolatedElement::targetConstraint()
{
    attConstraint->vectorizedTarget ( attTarget );
    return attConstraint;
}

ChppGikInterpolatedElement::~ChppGikInterpolatedElement()
{
    delete attConstraint;
}


CjrlGikStateConstraint* ChppGikInterpolatedElement::stateConstraintAtTime ( double inTime )
{
    unsigned int i = ChppGikTools::timetoRank ( attStartTime,inTime,attSamplingPeriod );

    if (i==1)
    {
      if (!planMotion())
      {
          std::cout<< "Failed to plan motion for interpolated element with priority"<< attPriority<<"\n";
          return 0;
      }
    }
    
    if ( (i > 0) && (i < attInterpolationData.size2()) && attPlanSuccess)
    {
        attSample = column ( attInterpolationData,i );
        attConstraint->vectorizedTarget ( attSample );
        //std::cout << "returning prioritized motion constraint with priority "<< attPriority-2<<"\n";
        return attConstraint;
    }
    else
        return 0;
}

bool ChppGikInterpolatedElement::planningAlgorithm()
{
    attPlanSuccess = false;
    unsigned int stateSize = attTarget.size();

    unsigned int nsamples = ChppGikTools::timetoRank ( 0, attDuration, attSamplingPeriod )+1;

    attInterpolationData.resize(stateSize, nsamples,false);
    attInterpolationLine.resize(nsamples,false);

    attConstraint->vectorizedTarget( attTarget);
    attConstraint->computeVectorizedTarget();
    attTarget = attConstraint->vectorizedTarget();
    
    attConstraint->computeVectorizedState();
    const vectorN& currentState = attConstraint->vectorizedState();

    for ( unsigned int i=0; i<stateSize;i++ )
    {
        bool contn = ChppGikTools::minJerkCurve ( attDuration, attSamplingPeriod, currentState ( i ),currentState ( stateSize + i ),currentState ( 2*stateSize + i ),attTarget ( i ), attInterpolationLine );
        if ( !contn )
            return false;
        row ( attInterpolationData,i ) = attInterpolationLine;
    }
    attPlanSuccess = true;
    return true;
}

bool ChppGikInterpolatedElement::planMotion()
{
    attPlanSuccess = false;
            
    if (attDuration < attSamplingPeriod)
        return false;

    if (attSamplingPeriod == 0)
        return false;

    bool planok = planningAlgorithm();

    if ( !planok )
    {
        std::cout << "ChppGikInterpolatedElement::planMotion() failed to plan motion\n";
        return false;
    }

    attPlanSuccess = true;
            
    return true;
}
