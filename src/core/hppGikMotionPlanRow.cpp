#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "core/hppGikMotionPlanRow.h"


ChppGikMotionPlanRow::ChppGikMotionPlanRow(CjrlHumanoidDynamicRobot* inRobot,unsigned int inPriority)
{
    attRobot = inRobot;
    attPriority = inPriority;
    attStartTime = 0.0;
    attEndTime = attStartTime;
    attWorkElement = new ChppGikMotionPlanElement(inRobot,inPriority);
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
    return (attMotionConstraints.size() == 0);
}

void ChppGikMotionPlanRow::addMotionConstraint(CjrlGikMotionConstraint* inMotionConstraint)
{
    attMotionConstraints.push_back(inMotionConstraint);
}

bool ChppGikMotionPlanRow::removeMotionConstraint(CjrlGikMotionConstraint* inMotionConstraint)
{
    bool ret = false;
    std::vector<CjrlGikMotionConstraint*>::iterator iter;
    for( iter = attMotionConstraints.begin(); iter != attMotionConstraints.end(); iter++)
        if ((*iter) == inMotionConstraint)
        {
            attMotionConstraints.erase(iter);
            ret = true;
            break;
        }

    return ret;
}


ChppGikMotionPlanElement* ChppGikMotionPlanRow::elementAtTime(double inTime)
{
    attWorkElement->clear();
    CjrlGikStateConstraint* constraint = 0;
    std::vector<CjrlGikMotionConstraint*>::iterator iter;
    for( iter = attMotionConstraints.begin(); iter != attMotionConstraints.end(); iter++)
    {
        constraint = 0;
        constraint = (*iter)->stateConstraintAtTime(inTime);
        if (constraint)
            attWorkElement->addConstraint(constraint);
    }
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
    double newStartTime = attEndTime;
    std::vector<CjrlGikMotionConstraint*>::iterator iter;
    for( iter = attMotionConstraints.begin(); iter != attMotionConstraints.end(); iter++)
        if ((*iter)->startTime() < newStartTime)
            newStartTime = (*iter)->startTime();
    attStartTime = newStartTime;
}

void ChppGikMotionPlanRow::updateEndTime()
{
    double newEndTime = attStartTime;
    std::vector<CjrlGikMotionConstraint*>::iterator iter;
    for( iter = attMotionConstraints.begin(); iter != attMotionConstraints.end(); iter++)
        if ((*iter)->endTime() > newEndTime)
            newEndTime = (*iter)->endTime();
    attEndTime = newEndTime;
}
