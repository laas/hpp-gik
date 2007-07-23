#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "core/hppGikMotionPlanElement.h"


ChppGikMotionPlanElement::ChppGikMotionPlanElement(CjrlDynamicRobot* inRobot, unsigned int inPriority)
{
    attDimension = 0;
    attPriority = inPriority;
    attRobot = inRobot;
    //initialize the jacobians' length with the number of internal degrees of freedom in the robot
    unsigned int dof;
    if (attRobot->countFixedJoints()>0){
	dof = attRobot->numberDof() - 6;
    }else{
	dof = 6;
    }
    attJacobian.resize(0,dof,false);
    attInfluencingDofs.resize(dof,false);
    attInfluencingDofsTemp.resize(attRobot->numberDof(),false);
}


CjrlGikStateConstraint* ChppGikMotionPlanElement::clone() const
{
    return (CjrlGikStateConstraint*) new ChppGikMotionPlanElement(*this);
}

CjrlDynamicRobot& ChppGikMotionPlanElement::robot()
{
    return *attRobot;
}

unsigned int ChppGikMotionPlanElement::priority() const
{
    return attPriority;
}


void ChppGikMotionPlanElement::addConstraint(CjrlGikStateConstraint* inStateConstraint)
{
    attConstraints.push_back(inStateConstraint);
    attDimension += inStateConstraint->dimension();
}


unsigned int ChppGikMotionPlanElement::dimension() const
{
    return attDimension;
}

void ChppGikMotionPlanElement::clear()
{
    attConstraints.clear();
    attDimension = 0;

}

vectorN& ChppGikMotionPlanElement::influencingDofs()
{
    attInfluencingDofs.clear();
    unsigned int offset = attRobot->countFixedJoints() > 0 ? 6 : 0;
    
    std::vector<CjrlGikStateConstraint*>::iterator iter;
    for (iter = attConstraints.begin(); iter != attConstraints.end(); iter++)
    {
        attInfluencingDofsTemp =  (*iter)->influencingDofs();
        for (unsigned int i=0; i<attInfluencingDofs.size(); i++)
            if (attInfluencingDofsTemp(i+offset) > 0)
                attInfluencingDofs(i) = 1;
    }
    return attInfluencingDofs;
}



void ChppGikMotionPlanElement::computeValue()
{
    if (attDimension!= attValue.size())
        attValue.resize(attDimension,false);

    std::vector<CjrlGikStateConstraint*>::iterator iter;
    unsigned int chunk_start = 0;
    unsigned int chunk_end = 0;
    for (iter = attConstraints.begin(); iter != attConstraints.end(); iter++)
    {
        chunk_end = chunk_start+(*iter)->dimension();
        (*iter)->computeValue();
        ublas::subrange(attValue, chunk_start, chunk_end) = (*iter)->value();
        chunk_start = chunk_end;
    }
}


void ChppGikMotionPlanElement::computeJacobian()
{
    if (attDimension!= attJacobian.size1())
        attJacobian.resize(attDimension,attJacobian.size2(),false);

    std::vector<CjrlGikStateConstraint*>::iterator iter;
    unsigned int chunk_start = 0;
    unsigned int chunk_end = 0;

    for (iter = attConstraints.begin(); iter != attConstraints.end(); iter++)
    {
        chunk_end = chunk_start+(*iter)->dimension();
        (*iter)->computeJacobian();
        ublas::subrange(attJacobian, chunk_start, chunk_end, 0, attJacobian.size2()) = (*iter)->jacobian();
        chunk_start = chunk_end;
    }
}

const vectorN& ChppGikMotionPlanElement::value()
{
    return attValue;
}


const matrixNxP& ChppGikMotionPlanElement::jacobian()
{
    return attJacobian;
}

ChppGikMotionPlanElement::~ChppGikMotionPlanElement()
{}
