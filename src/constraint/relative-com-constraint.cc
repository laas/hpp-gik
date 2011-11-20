#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/constraint/relative-com-constraint.hh"
#include "hpp/gik/tools.hh"


ChppGikRelativeComConstraint::ChppGikRelativeComConstraint(CjrlDynamicRobot& inRobot, 
							   CjrlJoint* inJoint, 
							   double inX, 
							   double inY):
  ChppGikComConstraint(inRobot,inX,inY)
{
  robot_ = &inRobot;
  rootJoint_ = inJoint;
  jacobian_.resize(2,robot_->numberDof(),false);
  comJacobian_.resize(3,robot_->numberDof(),false);
  posJacobian_.resize(3,robot_->numberDof(),false);

  jointRot_.resize(3,3,false);
  jointTr_.resize(3,false);

  comPos_.resize(3,false);
  worldTarget_.resize(3,false);
  value_.resize(2,false);

  localTarget_.resize(3,false);
  localTarget_(0) = inX;
  localTarget_(1) = inY;
  localTarget_(2) = 0;

  localTargetVector3d_(0) = inX;
  localTargetVector3d_(1) = inY;
  localTargetVector3d_(2) = 0;   


}

ChppGikRelativeComConstraint::~ChppGikRelativeComConstraint()
{
}

CjrlGikStateConstraint* 
ChppGikRelativeComConstraint::clone() const
{
  CjrlGikStateConstraint* ret = new ChppGikRelativeComConstraint(*this);
  return ret;
}

void
ChppGikRelativeComConstraint::computeValue()
{
  ChppGikTools::Vector3toUblas( robot_->positionCenterOfMass(), comPos_);
  ChppGikTools::HtoRT(rootJoint_->currentTransformation(),jointRot_,jointTr_);
  worldTarget_ = prod(jointRot_,localTarget_) + jointTr_;

  value_ = subrange(comPos_ - worldTarget_,0,2);
}

void
ChppGikRelativeComConstraint::computeJacobian()
{
  robot().getJacobianCenterOfMass(*robot().rootJoint(),comJacobian_,6,false);
  robot().getPositionJacobian(*robot().rootJoint(),*rootJoint_,localTargetVector3d_,posJacobian_,6,false);

  row(jacobian_,0) = row(comJacobian_,0) - row(posJacobian_,0);
  row(jacobian_,1) = row(comJacobian_,1) - row(posJacobian_,1);
}

const vectorN& 
ChppGikRelativeComConstraint::value()
{
    return value_;
}

const matrixNxP& 
ChppGikRelativeComConstraint::jacobian()
{
    return jacobian_;
}
