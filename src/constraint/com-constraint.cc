#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikComConstraint.h"
#include "hppGikTools.h"
#include <time.h>
#include <sys/time.h>
#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J

using namespace boost::numeric::ublas;



ChppGikComConstraint::ChppGikComConstraint(CjrlDynamicRobot& inRobot, double inX, double inY)
{
    attRobot = &inRobot;
    attRootJoint = attRobot->rootJoint();
    attDimension = 2;

    attJacobian.resize(attDimension,attRobot->numberDof(),false);
    attValue.resize(attDimension, false);
    attWorldTarget.resize(attDimension,false);
    attWorldTarget(0) = inX;
    attWorldTarget(1) = inY;

    tempJacobian.resize(3,attRobot->numberDof(),false);

    temp3DVec.resize(3,false);

    attInfluencingDofs = scalar_vector<double>(attRobot->numberDof(), 1);
}

CjrlGikStateConstraint* ChppGikComConstraint::clone() const
{
    return new ChppGikComConstraint(*this);
}


void ChppGikComConstraint::dimension(unsigned int inDim)
{

    if (inDim != attDimension)
    {

        attDimension = inDim;
        attJacobian.resize(attDimension,attRobot->numberDof(),false);
        attValue.resize(attDimension, false);
        attWorldTarget.resize(attDimension,false);
    }
}


unsigned int ChppGikComConstraint::dimension() const
{
    return attDimension;
}


CjrlDynamicRobot& ChppGikComConstraint::robot()
{
    return *attRobot;
}


void ChppGikComConstraint::targetXY(double inX, double inY)
{
    dimension(2);
    attWorldTarget(0) = inX;
    attWorldTarget(1) = inY;
}


void ChppGikComConstraint::targetXYZ(const vectorN& inTarget)
{
    dimension(3);
    attWorldTarget = inTarget;
}

const vectorN& ChppGikComConstraint::worldTarget()
{
    return attWorldTarget;
}

void ChppGikComConstraint::computeValue()
{
    ChppGikTools::Vector3toUblas( attRobot->positionCenterOfMass(), temp3DVec);
    attValue = subrange(temp3DVec,0,attDimension) - attWorldTarget ;
}

void ChppGikComConstraint::computeInfluencingDofs()
{}


vectorN& ChppGikComConstraint::influencingDofs()
{
    return attInfluencingDofs;
}

void ChppGikComConstraint::jacobianRoot(CjrlJoint& inJoint)
{
    attRootJoint = &inJoint;
}

//certainly not optimal here. should be moved to Humanoid robot.
void ChppGikComConstraint::computeJacobian()
{
    if (attDimension ==2)
    {
        robot().getJacobianCenterOfMass( *attRootJoint, tempJacobian);
        noalias(row(attJacobian,0)) = row(tempJacobian,0);
        noalias(row(attJacobian,1)) = row(tempJacobian,1);
    }
    else
        robot().getJacobianCenterOfMass( *attRootJoint, attJacobian);
}


const vectorN& ChppGikComConstraint::value()
{
    return attValue;
}

const matrixNxP& ChppGikComConstraint::jacobian()
{
    return attJacobian;
}
