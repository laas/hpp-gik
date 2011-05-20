#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "hpp/gik/core/motion-plan.hh"
#include "hpp/gik/tools.hh"

using namespace boost::numeric::ublas;

ChppGikMotionPlan::ChppGikMotionPlan(CjrlDynamicRobot* inRobot)
{
    attRobot = inRobot;
    attStartTime = 0.0;
    attEndTime = 0.0;
    attWorkColumn = new ChppGikMotionPlanColumn(inRobot);
    attVector = scalar_vector<double>(inRobot->numberDof(),0);
}

ChppGikMotionPlan::~ChppGikMotionPlan()
{
    std::vector<ChppGikMotionPlanRow*>::iterator iter;
    for( iter = attRows.begin(); iter != attRows.end(); iter++)
        delete *iter;
    delete attWorkColumn;
}

void ChppGikMotionPlan::removeMotion(ChppGikPrioritizedMotion* inMotion)
{
  std::vector<ChppGikMotionPlanRow*>::iterator iter;
  for (iter = attRows.begin(); iter != attRows.end(); iter++)
    {
      (*iter)->removeMotion(inMotion);
    }
}


ChppGikMotionPlanRow* ChppGikMotionPlan::addMotion(ChppGikPrioritizedMotion* inMotion)
{
    bool foundSamePriority = false;
    unsigned int inPriority = inMotion->priority();
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
        (*iter)->addMotion(inMotion);
        //std::cout << "Added a motion from time "<< inMotion->motionConstraint()->startTime() <<" to time "<< inMotion->motionConstraint()->endTime() <<" in row "<< inMotion->priority() <<"\n";
        return (*iter);
    }

    ChppGikMotionPlanRow* newRow = new ChppGikMotionPlanRow(attRobot, inPriority);
    newRow->addMotion(inMotion);
    attRows.insert(insertionLocation, newRow);
    
    //std::cout << "Added a motion from time "<< inMotion->motionConstraint()->startTime() <<" to time "<< inMotion->motionConstraint()->endTime() <<" in row "<< inMotion->priority()  <<"\n";
    return newRow;

}

ChppGikMotionPlanRow* ChppGikMotionPlan::getRow(unsigned int inRank)
{
    if (inRank >= attRows.size())
        return 0;
    else
        return attRows[inRank];
}

bool ChppGikMotionPlan::getRankForPriority(unsigned int inPriority, unsigned int& outRownumber)
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
    attVector.clear();
    for( iter = attRows.begin(); iter != attRows.end(); iter++)
    {
        element = (*iter)->elementAtTime(inTime);
        if (element->dimension())
        {
            attWorkColumn->addElement(element);
            ChppGikTools::combineMasks( attVector, element->workingJoints(), attVector );
        }
    }
    attWorkColumn->workingJoints( attVector );
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

CjrlDynamicRobot* ChppGikMotionPlan::robot()
{
    return attRobot;
}
