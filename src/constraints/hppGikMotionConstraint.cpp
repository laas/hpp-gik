#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikMotionConstraint.h"
#include "hppGikTools.h"


ChppGikMotionConstraint::ChppGikMotionConstraint(double inSamplingPeriod, double inStartTime)
{
    attSamplingPeriod = inSamplingPeriod;
    attStartTime = inStartTime;
    attEndTime = attStartTime;
}

double ChppGikMotionConstraint::samplingPeriod()
{
    return attSamplingPeriod;
}

void ChppGikMotionConstraint::pushbackStateConstraint(const CjrlGikStateConstraint* inStateConstraint)
{
    CjrlGikStateConstraint* stateConstraint = inStateConstraint->clone();
    attVectorStateConstraint.push_back(stateConstraint);
    attEndTime += attSamplingPeriod ;
}

void ChppGikMotionConstraint::pushfrontStateConstraint(const CjrlGikStateConstraint* inStateConstraint)
{
    CjrlGikStateConstraint* stateConstraint = inStateConstraint->clone();
    attVectorStateConstraint.insert(attVectorStateConstraint.begin(),stateConstraint);
    attStartTime -= attSamplingPeriod;
}

CjrlGikStateConstraint* ChppGikMotionConstraint::stateConstraintAtTime(double inTime)
{
    if (inTime <= attStartTime-attSamplingPeriod/2)
        return 0;
    
    unsigned int i = ChppGikTools::timetoRank(attStartTime,inTime,attSamplingPeriod);

    //std::cout <<  i << " / "<< attVectorStateConstraint.size() << std::endl;
    
    return stateConstraintAtRank(i);
}

CjrlGikStateConstraint* ChppGikMotionConstraint::stateConstraintAtRank(unsigned int inRank)
{
    if (inRank > (attVectorStateConstraint.size() -1))
        return 0;
    else
        return attVectorStateConstraint[inRank];
}

unsigned int ChppGikMotionConstraint::numberStateConstraints()
{
    return attVectorStateConstraint.size();
}

bool ChppGikMotionConstraint::empty()
{
    return (attVectorStateConstraint.size()==0);
}

void ChppGikMotionConstraint::clear()
{
    std::vector<CjrlGikStateConstraint*>::iterator iter;
    for( iter = attVectorStateConstraint.begin(); iter != attVectorStateConstraint.end(); iter++)
        delete *iter;
    attVectorStateConstraint.clear();
    attStartTime = 0.0;
    attEndTime = attStartTime;
}

void ChppGikMotionConstraint::startTime(double inStartTime)
{
    attEndTime = attEndTime - attStartTime + inStartTime;
    attStartTime = inStartTime;
}


double ChppGikMotionConstraint::startTime()
{
    return attStartTime;
}


double ChppGikMotionConstraint::endTime()
{
    return attEndTime-attSamplingPeriod;
}


ChppGikMotionConstraint::~ChppGikMotionConstraint()
{
    std::vector<CjrlGikStateConstraint*>::iterator iter;
    for( iter = attVectorStateConstraint.begin(); iter != attVectorStateConstraint.end(); iter++)
        delete *iter;
}
