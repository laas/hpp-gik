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

  localTarget_(0) = inX;
  localTarget_(1) = inY;
  localTarget_(2) = 0;
  localTarget_(3) = 1;
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
  MAL_S4x4_C_eq_A_by_B (worldTarget_,
			rootJoint_->currentTransformation(),
			localTarget_);
  targetXY(worldTarget_[0],worldTarget_[1]);

  ChppGikComConstraint::computeValue();
}

void
ChppGikRelativeComConstraint::computeJacobian()
{
  robot().getJacobianCenterOfMass(*rootJoint_,comJacobian_);

  noalias(row(jacobian_,0)) = row(comJacobian_,0);
  noalias(row(jacobian_,1)) = row(comJacobian_,1);

  for(unsigned int i =0;i<6;i++){
    jacobian_(0,i) = 0;
    jacobian_(1,i) = 0;
  }
}

const matrixNxP& 
ChppGikRelativeComConstraint::jacobian()
{
    return jacobian_;
}
