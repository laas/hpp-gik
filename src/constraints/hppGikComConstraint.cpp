#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikComConstraint.h"
#include "hppGikTools.h"


ChppGikComConstraint::ChppGikComConstraint(CjrlHumanoidDynamicRobot& inRobot)
{
    attRobot = &inRobot;

    tempNumJoints = inRobot.numberDof()-6;

    attDimension = 3;

    attJacobian.resize(attDimension,tempNumJoints,false);
    attValue.resize(attDimension, false);
    tempJacobian.resize(3,tempNumJoints,false);
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
        attJacobian.resize(attDimension,tempNumJoints,false);
        attValue.resize(attDimension, false);
        attWorldTarget.resize(attDimension,false);
    }
}


unsigned int ChppGikComConstraint::dimension() const
{
    return attDimension;
}


CjrlHumanoidDynamicRobot& ChppGikComConstraint::robot()
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

vectorN& ChppGikComConstraint::influencingDofs()
{
    return attInfluencingDofs;
}

//certainly not optimal here. should be moved to Humanoid robot.
void ChppGikComConstraint::computeJacobian()
{

    tempFixedJoint = &(attRobot->fixedJoint(0));

    if (!tempFixedJoint)
    {
        std::cout << "ChppGikComConstraint::computeJacobian() expected a fixed joint on the robot.\n";
        return;
    }
    
    attRobot->computeJacobianCenterOfMass();
    //tempFixedJoint->computeJacobianJointWrtConfig();
    
    tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
    tempEffectorJointJacobian = &(attRobot->jacobianCenterOfMass());
    if (!tempFixedJointJacobian || !tempEffectorJointJacobian)
    {
        std::cout << "ChppGikComConstraint::computeJacobian() could not retrieve partial jacobians.\n";
        return;
    }

    /////*****//////
    /*
    std::vector<CjrlJoint*> vectorJoints = attRobot->jointVector();
    CjrlBody* body;
    CjrlJoint* joint;
    tempJacobian.clear();
    vector3d localCOM;
    vectorN localCOMU(3);
    double weight;
    
    for (unsigned int i=0; i<vectorJoints.size();i++)
    {
        joint = vectorJoints[i];
        if (joint == attRobot->rootJoint())
            continue;
        
        //center of mass of link i
        body = joint->linkedBody();
        localCOM = body->localCenterOfMass();
        ChppGikTools::Vector3toUblas( localCOM,localCOMU);
        //weight
        weight = body->mass()/attRobot->mass();
        
        //Jacobian of the current link com
        joint->computeJacobianJointWrtConfig();
        
        ChppGikTools::HtoRT(joint->currentTransformation(),tempRot,temp3DVec);
        ublas::noalias(temp3DVec) = -ublas::prod(tempRot,localCOMU);
        ChppGikTools::equivAsymMat(temp3DVec,tempRot);

        ublas::noalias(tempJacobian) +=weight*(ublas::subrange(joint->jacobianJointWrtConfig(),0,3,6,attRobot->numberDof()) + ublas::prod(tempRot,ublas::subrange(joint->jacobianJointWrtConfig(),3,6,6,attRobot->numberDof())));
    }
    
    //modded
    ublas::noalias(tempJacobian) -= ublas::subrange(*tempFixedJointJacobian,0,3,6,attRobot->numberDof());
    */
    /////*******///////
    
    
    ChppGikTools::Vector3toUblas( attRobot->positionCenterOfMass(), temp3DVec);

    ChppGikTools::HtoRT(tempFixedJoint->currentTransformation(),tempRot,temp3DVec1);
    temp3DVec.minus_assign(temp3DVec1);//COM in world - ankle joint center in world

    ublas::noalias(tempJacobian) = ublas::subrange(*tempEffectorJointJacobian,0,3,6,attRobot->numberDof()) - ublas::subrange(*tempFixedJointJacobian,0,3,6,attRobot->numberDof());

    ChppGikTools::equivAsymMat(temp3DVec,tempRot);
    ublas::noalias(tempJacobian) += ublas::prod(tempRot,ublas::subrange(*tempFixedJointJacobian,3,6,6,attRobot->numberDof()));

    attJacobian = ublas::subrange(tempJacobian,0,attDimension,0,tempNumJoints);

//     std::cout << "comJacobian\n";
//     ChppGikTools::printBlasMat( tempJacobian );
//      std::cout << "Config of computations\n";
//      std::cout << attRobot->currentConfiguration() << std::endl ;
}


const vectorN& ChppGikComConstraint::value()
{
    return attValue;
}

const matrixNxP& ChppGikComConstraint::jacobian()
{
    return attJacobian;
}
