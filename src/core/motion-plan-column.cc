#include "hpp/gik/core/motion-plan-column.hh"
using namespace boost::numeric::ublas;

ChppGikMotionPlanColumn::ChppGikMotionPlanColumn(CjrlDynamicRobot* inRobot)
{
    attRobot = inRobot;
    attWorkingJoints = scalar_vector<double>(inRobot->numberDof(),0);
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

const vectorN& ChppGikMotionPlanColumn::workingJoints() const
{
    return attWorkingJoints;
}


void ChppGikMotionPlanColumn::workingJoints(const vectorN& inVec)
{
    if (inVec.size() == attWorkingJoints.size())
        attWorkingJoints = inVec;
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

