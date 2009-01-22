#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "core/hppGikMotionPlanRow.h"
#include "hppGikTools.h"
using namespace boost::numeric::ublas;

ChppGikMotionPlanRow::ChppGikMotionPlanRow(CjrlHumanoidDynamicRobot* inRobot,unsigned int inPriority)
{
    attRobot = inRobot;
    attPriority = inPriority;
    attStartTime = 0.0;
    attEndTime = attStartTime;
    attWorkElement = new ChppGikMotionPlanElement(inRobot,inPriority);
    attVector = scalar_vector<double>(inRobot->numberDof(),0);
}

ChppGikMotionPlanRow::~ChppGikMotionPlanRow()
{
    delete attWorkElement;
}

unsigned int ChppGikMotionPlanRow::priority() const
{
    return attPriority;
}

bool ChppGikMotionPlanRow::empty() const
{
    return (attMotions.size() == 0);
}

void ChppGikMotionPlanRow::addMotion(ChppGikPrioritizedMotion* inMotion)
{
    attMotions.push_back(inMotion);
}

bool ChppGikMotionPlanRow::removeMotion(ChppGikPrioritizedMotion* inMotion)
{
    bool ret = false;
    std::vector<ChppGikPrioritizedMotion*>::iterator iter;
    for( iter = attMotions.begin(); iter != attMotions.end(); iter++)
        if ((*iter) == inMotion)
        {
            attMotions.erase(iter);
            ret = true;
            break;
        }

    return ret;
}


ChppGikMotionPlanElement* ChppGikMotionPlanRow::elementAtTime(double inTime)
{
    attWorkElement->clear();
    CjrlGikStateConstraint* constraint = 0;
    std::vector<ChppGikPrioritizedMotion*>::iterator iter;
    attVector.clear();
    for( iter = attMotions.begin(); iter != attMotions.end(); iter++)
    {
        constraint = 0;
        constraint = (*iter)->motionConstraint()->stateConstraintAtTime(inTime);
        if (constraint)
        {
            attWorkElement->addConstraint(constraint);
            ChppGikTools::combineMasks( attVector, (*iter)->workingJoints(), attVector );
        }
    }
    attWorkElement->workingJoints( attVector );
    return attWorkElement;
}

double ChppGikMotionPlanRow::startTime()
{
    updateStartTime();
    return attStartTime;
}

double ChppGikMotionPlanRow::endTime()
{
    updateEndTime();
    return attEndTime;
}

void ChppGikMotionPlanRow::updateStartTime()
{
    if (empty())
        attStartTime = 0.0;
    else
    {
        double newStartTime = attEndTime;
        std::vector<ChppGikPrioritizedMotion*>::iterator iter;
        for( iter = attMotions.begin(); iter != attMotions.end(); iter++)
            if ((*iter)->motionConstraint()->startTime() < newStartTime)
                newStartTime = (*iter)->motionConstraint()->startTime();
        attStartTime = newStartTime;
    }
}

void ChppGikMotionPlanRow::updateEndTime()
{
    if (empty())
        attEndTime = 0.0;
    else
    {
        double newEndTime = attStartTime;
        std::vector<ChppGikPrioritizedMotion*>::iterator iter;
        for( iter = attMotions.begin(); iter != attMotions.end(); iter++)
            if ((*iter)->motionConstraint()->endTime() > newEndTime)
                newEndTime = (*iter)->motionConstraint()->endTime();
        attEndTime = newEndTime;
    }
}
