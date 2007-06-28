#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "core/hppGikMotionPlan.h"


ChppGikMotionPlan::ChppGikMotionPlan(CjrlHumanoidDynamicRobot* inRobot)
{
    attRobot = inRobot;
    attStartTime = 0.0;
    attEndTime = 0.0;
    attWorkColumn = new ChppGikMotionPlanColumn(inRobot);
}

ChppGikMotionPlan::~ChppGikMotionPlan()
{
    std::vector<ChppGikMotionPlanRow*>::iterator iter;
    for( iter = attRows.begin(); iter != attRows.end(); iter++)
        delete *iter;
    delete attWorkColumn;
}


ChppGikMotionPlanRow* ChppGikMotionPlan::addMotionConstraint(CjrlGikMotionConstraint* inMotionConstraint, unsigned int inPriority)
{
    bool foundSamePriority = false;
    std::vector<ChppGikMotionPlanRow*>::iterator iter;
    std::vector<ChppGikMotionPlanRow*>::iterator insertionLocation = attRows.end();

    for (iter = attRows.begin(); iter != attRows.end(); iter++)
    {

        if ((*iter)->priority() == inPriority)
        {
            foundSamePriority = true;
            break;
        }
        
        if ((*iter)->priority() > inPriority)
        {
            insertionLocation = iter;
            break;
        }

    }
    
    if (foundSamePriority)
    {
        (*iter)->addMotionConstraint(inMotionConstraint);
        return (*iter);
    }

    ChppGikMotionPlanRow* newRow = new ChppGikMotionPlanRow(attRobot, inPriority);
    newRow->addMotionConstraint(inMotionConstraint);
    attRows.insert(insertionLocation, newRow);
    return newRow;

}

ChppGikMotionPlanRow* ChppGikMotionPlan::getRow(unsigned int inRank)
{
    if (inRank >= attRows.size())
        return 0;
    else
        return attRows[inRank];
}

bool ChppGikMotionPlan::rankPriority(unsigned int inPriority, unsigned int& outRownumber)
{
    bool foundSamePriority = false;
    unsigned int rank = 0; 
    std::vector<ChppGikMotionPlanRow*>::iterator iter;
    for (iter = attRows.begin(); iter != attRows.end(); iter++)
    {
        if ((*iter)->priority() == inPriority)
        {
            foundSamePriority = true;
            outRownumber = rank;
            break;
        }
        rank++;
    }
    return foundSamePriority;
}

bool ChppGikMotionPlan::empty()
{
    std::vector<ChppGikMotionPlanRow*>::iterator iter;
    for( iter = attRows.begin(); iter != attRows.end(); iter++)
        if (!((*iter)->empty()))
            return false;
    return true;
}

ChppGikMotionPlanColumn* ChppGikMotionPlan::columnAtTime(double inTime)
{
    attWorkColumn->clear();
    ChppGikMotionPlanElement* element =0;
    std::vector<ChppGikMotionPlanRow*>::iterator iter;
    for( iter = attRows.begin(); iter != attRows.end(); iter++)
    {
        element = (*iter)->elementAtTime(inTime);
        if (element->dimension())
            attWorkColumn->addElement(element);
    }
    return attWorkColumn;
}

double ChppGikMotionPlan::startTime()
{
    updateStartTime();
    return attStartTime;
}

double ChppGikMotionPlan::endTime()
{
    updateEndTime();
    return attEndTime;
}

void ChppGikMotionPlan::updateStartTime()
{
    double newStartTime = attEndTime;
    std::vector<ChppGikMotionPlanRow*>::iterator iter;
    for( iter = attRows.begin(); iter != attRows.end(); iter++)
        if ((*iter)->startTime() < newStartTime)
            newStartTime = (*iter)->startTime();
    attStartTime = newStartTime;
}

void ChppGikMotionPlan::updateEndTime()
{
    double newEndTime = attStartTime;
    std::vector<ChppGikMotionPlanRow*>::iterator iter;
    for( iter = attRows.begin(); iter != attRows.end(); iter++)
        if ((*iter)->endTime() > newEndTime)
            newEndTime = (*iter)->endTime();
    attEndTime = newEndTime;
}

CjrlHumanoidDynamicRobot* ChppGikMotionPlan::robot()
{
    return attRobot;
}
