#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikJointStateConstraint.h"

using namespace ublas;

ChppGikJointStateConstraint::ChppGikJointStateConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint)
{
    attJoint = &inJoint;
    attRobot = &inRobot;
    attInfluencingDofs = zero_vector<double>(attRobot->numberDof());
    
    if (attRobot->countFixedJoints()>0){
        attNumberActuatedDofs = inRobot.numberDof()-6;
    }else{
        attNumberActuatedDofs = 6;
    }
    
    computeInfluencingDofs();
}

void ChppGikJointStateConstraint::computeInfluencingDofs()
{
    unsigned int i;
    attInfluencingDofs.clear();
    std::vector<CjrlJoint *> joints;

    //Free flyer dofs
    CjrlJoint *root = attRobot->rootJoint();
    unsigned int start = root->rankInConfiguration();
    for (i=0; i<root->numberDof(); i++)
        attInfluencingDofs[start+i] = 1;

    //Dofs from free flyer to effector joint
    joints = attJoint->jointsFromRootToThis();
    for(i=1; i< joints.size(); i++)
        attInfluencingDofs(joints[i]->rankInConfiguration()) = 1;
}

vectorN& ChppGikJointStateConstraint::influencingDofs()
{
    return attInfluencingDofs;
}

const vectorN& ChppGikJointStateConstraint::value()
{
    return attValue;
}

const matrixNxP& ChppGikJointStateConstraint::jacobian()
{
    return attJacobian;
}

CjrlJoint* ChppGikJointStateConstraint::joint()
{
    return attJoint;
}

void ChppGikJointStateConstraint::joint(CjrlJoint* inJoint)
{
    attJoint = inJoint;
    computeInfluencingDofs();
}

unsigned int ChppGikJointStateConstraint::dimension() const
{
    return attDimension;
}

CjrlDynamicRobot& ChppGikJointStateConstraint::robot()
{
    return *attRobot;
}

const vectorN& ChppGikJointStateConstraint::interpolatedTarget()
{
    return attInterpolatedTarget;
}
        
const vectorN& ChppGikJointStateConstraint::vectorizedState() const
{
    return attVectorizedState;
}
const vectorN& ChppGikJointStateConstraint::vectorizedTarget() const
{
    return attVectorizedTarget;
}
void ChppGikJointStateConstraint::computeInterpolatedTarget(double inLambda)
{
    if (attInterpolatedTarget.size() != attVectorizedTarget.size())
        attInterpolatedTarget.resize(attVectorizedTarget.size(), false);

    if (inLambda < 0)
    {
        attInterpolatedTarget = ublas::subrange (attVectorizedState, 0,  attVectorizedState.size());
    }
    else
    if (inLambda > 1)
    {
        attInterpolatedTarget =  attVectorizedTarget;
    }
    else
    {
    ublas::noalias (attInterpolatedTarget )=  (1-inLambda)* ublas::subrange (attVectorizedState, 0,  attInterpolatedTarget.size());
    ublas::noalias (attInterpolatedTarget ) += inLambda*attVectorizedTarget;
    }
}
