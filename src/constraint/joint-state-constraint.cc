#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
# include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/constraint/joint-state-constraint.hh"

using namespace boost::numeric::ublas;

ChppGikJointStateConstraint::ChppGikJointStateConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, unsigned int inDimension) : attDimension(inDimension)
{
    attJoint = &inJoint;
    attRobot = &inRobot;
    attRootJoint = attRobot->rootJoint();
    attInfluencingDofs = zero_vector<double>(attRobot->numberDof());
    computeInfluencingDofs();
}

void ChppGikJointStateConstraint::computeInfluencingDofs()
{
    attInfluencingDofs.clear();
    std::vector<CjrlJoint*> vecJoints = attRobot->jointsBetween( *attRootJoint, *attJoint);
    unsigned int i,j;
    for (i=0;i<vecJoints.size();i++)
        for (j=vecJoints[i]->rankInConfiguration();j<vecJoints[i]->rankInConfiguration()+vecJoints[i]->numberDof();j++)
            attInfluencingDofs(j) = 1;

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

void ChppGikJointStateConstraint::joint(CjrlJoint* inJoint)
{
    if (attJoint!=inJoint)
        ;
    {
        attJoint = inJoint;
        computeInfluencingDofs();
    }
}

void ChppGikJointStateConstraint::jacobianRoot(CjrlJoint& inJoint)
{
    if (attRootJoint!=&inJoint)
    {
        attRootJoint = &inJoint;
        computeInfluencingDofs();
    }
}

unsigned int ChppGikJointStateConstraint::dimension() const
{
    return attDimension;
}

