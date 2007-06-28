#include "core/hppGikMotionPlanColumn.h"

ChppGikMotionPlanColumn::ChppGikMotionPlanColumn(CjrlHumanoidDynamicRobot* inRobot)
{
    attRobot = inRobot;
}


void ChppGikMotionPlanColumn::clear()
{
    attTaskStack.clear();
}

void ChppGikMotionPlanColumn::addElement(ChppGikMotionPlanElement* inTask)
{
    std::vector<ChppGikMotionPlanElement*>::iterator iter;
    for (iter = attTaskStack.begin(); iter != attTaskStack.end(); iter++)
    {
        if ((*iter)->priority() > inTask->priority())
            break;
    }
    attTaskStack.insert(iter,inTask);
}

unsigned int ChppGikMotionPlanColumn::numberElements() const
{
    return attTaskStack.size();
}



std::vector<CjrlGikStateConstraint*> ChppGikMotionPlanColumn::constraints()
{
    std::vector<CjrlGikStateConstraint*> constraintVector;
    for (unsigned int i=0; i<numberElements(); i++)
        constraintVector.push_back(attTaskStack[i]);
    return constraintVector;
}

ChppGikMotionPlanColumn::~ChppGikMotionPlanColumn()
{
}

