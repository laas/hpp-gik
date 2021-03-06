#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/core/motion-plan-element.hh"

using namespace boost::numeric::ublas;

ChppGikMotionPlanElement::ChppGikMotionPlanElement(CjrlDynamicRobot* inRobot, unsigned int inPriority, double inDampingFactor)
{
  attDimension = 0;
  attPriority = inPriority;
  attRobot = inRobot;
  attJacobian.resize(0,attRobot->numberDof(),false);
  attInfluencingDofs.resize(attRobot->numberDof(),false);
  attWorkingJoints = scalar_vector<double>(attRobot->numberDof(),0);
  attDampingFactor = inDampingFactor>=0.0?inDampingFactor:0.0;
}

void ChppGikMotionPlanElement::dampingFactor(double inDF)
{
  attDampingFactor = inDF>=0.0?inDF:0.0;
}

double ChppGikMotionPlanElement::dampingFactor() const
{
  return attDampingFactor;
}

CjrlGikStateConstraint* ChppGikMotionPlanElement::clone() const
{
  return (CjrlGikStateConstraint*) new ChppGikMotionPlanElement(*this);
}

CjrlDynamicRobot& ChppGikMotionPlanElement::robot()
{
  return *attRobot;
}

void ChppGikMotionPlanElement::jacobianRoot(CjrlJoint& inJoint)
{
  for (unsigned int i =0; i<attConstraints.size();i++)
    attConstraints[i]->jacobianRoot(inJoint);
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
bool ChppGikMotionPlanElement::removeConstraint(const CjrlGikStateConstraint* inJrlStateConstraint)
{
  std::vector<CjrlGikStateConstraint*>::iterator iter;
  for (iter = attConstraints.begin();iter!=attConstraints.end();iter++)
    if (*iter == inJrlStateConstraint)
    {
      attDimension -= (*iter)->dimension();
      attConstraints.erase(iter);
      return true;
    }
  return false;
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

const vectorN& ChppGikMotionPlanElement::workingJoints() const
{
  return attWorkingJoints;
}


void ChppGikMotionPlanElement::workingJoints(const vectorN& inVec)
{
  if (inVec.size() == attWorkingJoints.size())
    attWorkingJoints = inVec;
}


void ChppGikMotionPlanElement::computeInfluencingDofs()
{
  if (attConstraints.size() == 1)
    attConstraints[0]->computeInfluencingDofs();
  else
  {
    std::vector<CjrlGikStateConstraint*>::iterator iter;
    for (iter = attConstraints.begin(); iter != attConstraints.end(); iter++)
    {
      (*iter)->computeInfluencingDofs();
    }
  }
}

vectorN& ChppGikMotionPlanElement::influencingDofs()
{
  if (attConstraints.size() == 1)
    return attConstraints[0]->influencingDofs();
  else
  {
    attInfluencingDofs.clear();
    std::vector<CjrlGikStateConstraint*>::iterator iter;
    for (iter = attConstraints.begin(); iter != attConstraints.end(); iter++)
    {
      vectorN& attInfluencingDofsTemp = (*iter)->influencingDofs();
      for (unsigned int i=0; i<attInfluencingDofs.size(); i++)
        if (attInfluencingDofsTemp(i) > 0)
          attInfluencingDofs(i) = 1;
    }
    return attInfluencingDofs;
  }
}



void ChppGikMotionPlanElement::computeValue()
{
  if (attConstraints.size() == 1)
  {
    attConstraints[0]->computeValue();
  }
  else
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
      subrange(attValue, chunk_start, chunk_end) = (*iter)->value();
      chunk_start = chunk_end;
    }
  }
}


void ChppGikMotionPlanElement::computeJacobian()
{
  if (attConstraints.size() == 1)
  {
    attConstraints[0]->computeJacobian();
  }
  else
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
      subrange(attJacobian, chunk_start, chunk_end, 0, attJacobian.size2()) = (*iter)->jacobian();
      chunk_start = chunk_end;
    }
  }
}

const vectorN& ChppGikMotionPlanElement::value()
{
  if (attConstraints.size() == 1)
    return attConstraints[0]->value();
  else
    return attValue;
}


const matrixNxP& ChppGikMotionPlanElement::jacobian()
{
  if (attConstraints.size() == 1)
    return attConstraints[0]->jacobian();
  else
    return attJacobian;
}


ChppGikMotionPlanElement::~ChppGikMotionPlanElement()
{}
