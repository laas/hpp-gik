#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikComConstraint.h"
#include "hppGikTools.h"

#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J

using namespace ublas;



ChppGikComConstraint::ChppGikComConstraint(CjrlDynamicRobot& inRobot, double inX, double inY)
{
    attRobot = &inRobot;

    attNumberActuatedDofs = inRobot.numberDof()-6;

    attDimension = 2;

    attJacobian.resize(attDimension,attNumberActuatedDofs,false);
    attValue.resize(attDimension, false);
    attWorldTarget.resize(attDimension,false);
    attWorldTarget(0) = inX;
    attWorldTarget(1) = inY;

    //tempJacobian.resize(3,inRobot.numberDof(),false);
    tempJacobian.resize(3,attNumberActuatedDofs,false);
    tempJointJacobian.resize(6,inRobot.numberDof(),false);

    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);

    attInfluencingDofs = ublas::scalar_vector<double>(inRobot.numberDof(), 1);
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
        attJacobian.resize(attDimension,attNumberActuatedDofs,false);
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
    attValue = attWorldTarget - ublas::subrange(temp3DVec,0,attDimension);
}

void ChppGikComConstraint::computeInfluencingDofs()
{}


vectorN& ChppGikComConstraint::influencingDofs()
{
    return attInfluencingDofs;
}

//certainly not optimal here. should be moved to Humanoid robot.
void ChppGikComConstraint::computeJacobian()
{
/*
    tempFixedJoint = &(attRobot->fixedJoint(0));

    if (!tempFixedJoint)
    {
        std::cout << "ChppGikComConstraint::computeJacobian() expected a fixed joint on the robot.\n";
        return;
    }

    attRobot->computeJacobianCenterOfMass();

    tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
    tempEffectorJointJacobian = &(attRobot->jacobianCenterOfMass());
    if (!tempFixedJointJacobian || !tempEffectorJointJacobian)
    {
        std::cout << "ChppGikComConstraint::computeJacobian() could not retrieve partial jacobians.\n";
        return;
    }

    ChppGikTools::Vector3toUblas( attRobot->positionCenterOfMass(), temp3DVec);

    ChppGikTools::HtoRT(tempFixedJoint->currentTransformation(),tempRot,temp3DVec1);
    temp3DVec.minus_assign(temp3DVec1);//COM in world - ankle joint center in world
    
    ChppGikTools::equivAsymMat(temp3DVec,tempRot);
    for (unsigned int i = 0; i< attDimension;i++)
    {
    noalias (row(tempJacobian,i)) = row(*tempEffectorJointJacobian,i) - row(*tempFixedJointJacobian,i);
    for (unsigned int j =0;j<3;j++)
    noalias (row(tempJacobian,i)) += tempRot(i,j) * row(*tempFixedJointJacobian,j+3);
    }

    noalias (subrange(attJacobian,0,attDimension,0, attNumberActuatedDofs) )= subrange(tempJacobian,0,attDimension,6,attRobot->numberDof());
    

*/
    ///*
    
    tempFixedJoint = &(attRobot->fixedJoint(0));

    if (!tempFixedJoint)
    {
        std::cout << "ChppGikComConstraint::computeJacobian() expected a fixed joint on the robot.\n";
        return;
    }

    tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
    
    if (!tempFixedJointJacobian )
    {
        std::cout << "ChppGikComConstraint::computeJacobian() could not retrieve partial jacobians.\n";
        return;
    }
    
    attRobot->computeJacobianCenterOfMass();
    tempJacobian = subrange(attRobot->jacobianCenterOfMass(),0,3,6,attRobot->numberDof());

    std::vector<CjrlJoint*> routeJoints;

    unsigned int rank;
    routeJoints = tempFixedJoint->jointsFromRootToThis();
    for (unsigned int k= 1; k<routeJoints.size();k++)
    {
        rank = routeJoints[k]->rankInConfiguration(); 
        for (unsigned int l=0; l<3;l++)
            tempJacobian(l,rank-6) -= (*tempFixedJointJacobian)(l,rank);
    }
    

    ChppGikTools::Vector3toUblas( attRobot->positionCenterOfMass(), temp3DVec);
    ChppGikTools::HtoRT(tempFixedJoint->currentTransformation(),tempRot,temp3DVec1);
    temp3DVec.minus_assign(temp3DVec1);
    ChppGikTools::equivAsymMat(temp3DVec,tempRot);
    
    ublas::noalias(tempJacobian) += ublas::prod(tempRot, ublas::subrange(*tempFixedJointJacobian,3,6,6,attRobot->numberDof()));

    attJacobian = ublas::subrange(tempJacobian,0,attDimension,0,attNumberActuatedDofs);

    //*/


}


const vectorN& ChppGikComConstraint::value()
{
    return attValue;
}

const matrixNxP& ChppGikComConstraint::jacobian()
{
    return attJacobian;
}
