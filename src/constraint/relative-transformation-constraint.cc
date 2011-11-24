#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/constraint/relative-transformation-constraint.hh"
#include "hpp/gik/tools.hh"

using namespace boost::numeric::ublas;

ChppGikRelativeTransformationConstraint::ChppGikRelativeTransformationConstraint(CjrlDynamicRobot& inRobot, 
										 CjrlJoint& inJoint, 
										 CjrlJoint * inRootJoint,
										 const matrix4d& inRelativeTransformation):
  ChppGikTransformationConstraint(inRobot,inJoint,vector3d(0,0,0),inRelativeTransformation)
{

  rootJoint_ = inRootJoint;
  targetTransformation4_ = inRelativeTransformation;
  targetR_.resize(3,3,false);
  targetT_.resize(3,false);

  jointR_.resize(3,3,false);
  jointT_.resize(3,false);

  gapR_.resize(3,3,false);
  gapREuler_.resize(3,false);
}

CjrlGikStateConstraint* 
ChppGikRelativeTransformationConstraint::clone() const
{
  return (new ChppGikRelativeTransformationConstraint(*this));
}

void
ChppGikRelativeTransformationConstraint::computeValue()
{
  MAL_S4x4_C_eq_A_by_B ( globalTargetTransformation_,
			 rootJoint_->currentTransformation(),
			 targetTransformation4_);
  targetTransformation(globalTargetTransformation_);
  ChppGikTransformationConstraint::computeValue();
  /*
 ChppGikTools::HtoRT(joint()->currentTransformation(),jointR_,jointT_);

 ChppGikTools::HtoRT(globalTargetTransformation_,targetR_,targetT_);
 subrange(attValue,0,3) = targetT_ - jointT_;
 
 gapR_ = prod(trans(jointR_),targetR_);
 ChppGikTools::RottoOmega(gapR_,gapREuler_);

 noalias(subrange(attValue,3,6)) = -prod(jointR_,gapREuler_);
  */
}


void 
ChppGikRelativeTransformationConstraint::computeJacobian()
{
  robot().getJacobian( *rootJoint_,*joint(),vector3d(0,0,0),attJacobian);
  for (int i=0;i<6;i++) {
    for (int j=0;j<6;j++) {
      attJacobian(i,j) = 0;
    }
  }
}
