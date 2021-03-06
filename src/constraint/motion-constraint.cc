#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/constraint/motion-constraint.hh"
#include "hpp/gik/tools.hh"


ChppGikMotionConstraint::ChppGikMotionConstraint(CjrlDynamicRobot* inRobot, double inSamplingPeriod, double inStartTime)
{
    attSamplingPeriod = inSamplingPeriod;
    attStartTime = (inStartTime>0.0)?inStartTime:0.0;
    attEndTime = attStartTime;
    attEps = attSamplingPeriod/2;
    attRobot = inRobot;
}

CjrlDynamicRobot* ChppGikMotionConstraint::robot() 
{
    return attRobot;
}

CjrlGikMotionConstraint* ChppGikMotionConstraint::clone() const
{
    ChppGikMotionConstraint* ret =  new ChppGikMotionConstraint(attRobot, attSamplingPeriod, attStartTime);
    CjrlGikStateConstraint *cstr = 0;
    unsigned int i = 0;
    cstr = stateConstraintAtRank( i);
    while (cstr)
    {
        ret->pushbackStateConstraint( cstr );
        i++;
        cstr = stateConstraintAtRank( i );
    }
    return ret;
}

double ChppGikMotionConstraint::samplingPeriod()
{
    return attSamplingPeriod;
}

void ChppGikMotionConstraint::pushbackStateConstraint(const CjrlGikStateConstraint* inStateConstraint)
{
    CjrlGikStateConstraint* stateConstraint = inStateConstraint->clone();
    if (attVectorStateConstraint.size() != 0)
        attEndTime += attSamplingPeriod;
    attVectorStateConstraint.push_back(stateConstraint);
}

void ChppGikMotionConstraint::pushfrontStateConstraint(const CjrlGikStateConstraint* inStateConstraint)
{
    CjrlGikStateConstraint* stateConstraint = inStateConstraint->clone();
    if (attVectorStateConstraint.size() != 0)
        attStartTime -= attSamplingPeriod;
    attVectorStateConstraint.insert(attVectorStateConstraint.begin(),stateConstraint);
}

CjrlGikStateConstraint* ChppGikMotionConstraint::stateConstraintAtTime(double inTime)
{
    if (inTime < attStartTime+attEps)
        return 0;
    
    unsigned int i = ChppGikTools::timetoRank(attStartTime,inTime,attSamplingPeriod);

    //std::cout <<  i << " / "<< attVectorStateConstraint.size() << std::endl;
    
    return stateConstraintAtRank(i);
}

CjrlGikStateConstraint* ChppGikMotionConstraint::stateConstraintAtRank(unsigned int inRank) const
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
    double temp = (inStartTime>0.0)?inStartTime:0.0;
    attEndTime = attEndTime - attStartTime + temp;
    attStartTime = temp;
}


double ChppGikMotionConstraint::startTime()
{
    return attStartTime;
}


double ChppGikMotionConstraint::endTime()
{
    return attEndTime;
}


ChppGikMotionConstraint::~ChppGikMotionConstraint()
{
    std::vector<CjrlGikStateConstraint*>::iterator iter;
    for( iter = attVectorStateConstraint.begin(); iter != attVectorStateConstraint.end(); iter++)
        delete *iter;
}
