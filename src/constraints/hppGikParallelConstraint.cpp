#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikParallelConstraint.h"
#include "hppGikTools.h"


ChppGikParallelConstraint::ChppGikParallelConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalVector, const vector3d& inTargetVector):ChppGikJointStateConstraint(inRobot, inJoint)
{
    attLocalVectorVector3 =  inLocalVector;
    attTargetVectorVector3 =  inTargetVector;

    attLocalVector.resize(3,false);
    attTargetVector.resize(3,false);

    ChppGikTools::Vector3toUblas(attLocalVectorVector3, attLocalVector);
    ChppGikTools::Vector3toUblas(attTargetVectorVector3, attTargetVector);

    attLocalVector =  attLocalVector/boost_ublas::norm_2(attLocalVector);
    attTargetVector =  attTargetVector/boost_ublas::norm_2(attTargetVector);

    tempJointOrientJacobian.resize(3,attNumberActuatedDofs,false);
    attJacobian.resize(3,attNumberActuatedDofs,false);
    attValue.resize(3, false);

    tempRot.resize(3,3,false);
    tempRot1.resize(3,3,false);
    tempRot2.resize(3,3,false);
    tempRot3.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);

    attVectorizedState.resize(9,false);
    attVectorizedTarget.resize(3,false);

    attDimension = 3;
}

CjrlGikStateConstraint* ChppGikParallelConstraint::clone() const
{
    CjrlGikParallelConstraint* ret =  new ChppGikParallelConstraint(*this);
    return ret;
}

void  ChppGikParallelConstraint::localVector(const vector3d& inVector)
{
    attLocalVectorVector3 =  inVector;
    ChppGikTools::Vector3toUblas(attLocalVectorVector3, attLocalVector);
    attLocalVector =  attLocalVector/boost_ublas::norm_2(attLocalVector);
}

const vector3d& ChppGikParallelConstraint::localVector()
{
    return attLocalVectorVector3;
}

void  ChppGikParallelConstraint::targetVector (const vector3d& inVector)
{
    attTargetVectorVector3 =  inVector;
    ChppGikTools::Vector3toUblas(attTargetVectorVector3, attTargetVector);
    attTargetVector =  attTargetVector/boost_ublas::norm_2(attTargetVector);
}

const vector3d& ChppGikParallelConstraint::targetVector()
{
    return attTargetVectorVector3;
}


void ChppGikParallelConstraint::computeValue()
{

    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);
    //std::cout << "local vector "<< attLocalVector << "\n";
    temp3DVec =  boost_ublas::prod(tempRot,attLocalVector);
    //std::cout << "current world vector "<< temp3DVec << "\n";
    ChppGikTools::CrossProduct(attTargetVector,temp3DVec,attValue);
    //std::cout << "attValue "<< attValue << "\n";
}

void ChppGikParallelConstraint::computeJacobian()
{
    tempFixedJoint = &(attRobot->fixedJoint(0));
    if (!tempFixedJoint)
    {
        std::cout << "ChppGikParallelConstraint::computeJacobian() expected a fixed joint on the robot.\n";
        return;
    }

    attJoint->computeJacobianJointWrtConfig();
    //tempFixedJoint->computeJacobianJointWrtConfig();

    tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
    tempEffectorJointJacobian = &(attJoint->jacobianJointWrtConfig());
    if (!tempFixedJointJacobian || !tempEffectorJointJacobian)
    {
        std::cout << "ChppGikParallelConstraint::computeJacobian() could not retrieve partial jacobians.\n";
        return;
    }

    boost_ublas::noalias(tempJointOrientJacobian) = boost_ublas::subrange(*tempEffectorJointJacobian,3,6,6,attRobot->numberDof()) - boost_ublas::subrange(*tempFixedJointJacobian,3,6,6,attRobot->numberDof());

    //     std::cout << "tempJointOrientJacobian\n";
    //     ChppGikTools::printBlasMat( tempJointOrientJacobian );

    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);
    temp3DVec =  boost_ublas::prod(tempRot,attLocalVector);

    ChppGikTools::equivAsymMat(attTargetVector,tempRot1);
    ChppGikTools::equivAsymMat(temp3DVec,tempRot2);

    //     std::cout << "attTargetVector "<< attTargetVector << "\n";
    //     std::cout << "temp3DVec "<< temp3DVec << "\n";

    boost_ublas::noalias(tempRot3) = boost_ublas::prod(tempRot1,tempRot2);
    boost_ublas::noalias(attJacobian) = boost_ublas::prod(tempRot3,tempJointOrientJacobian);
}



void ChppGikParallelConstraint::computeVectorizedState()
{
    vectorN curpos(3);
    vectorN curvel(3);
    vectorN curaccel(3);

    //matrixNxP tempMat4(4,4);
    vectorN rotvel(3);
    vectorN rotvelCrossLocal(3);
    vectorN rotaccel(3);
    vectorN temp(3);

    //To DO
    //correct a known bug in this hurriedly coded method: determin the nearest target vector (the given one or its opposite, because a parallel constraint does not care about orientation like a pointing constraint)

    //constraint position
    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);
    curpos =  boost_ublas::prod(tempRot,attLocalVector);

    //std::cout << "curpos " << curpos << std::endl;

    //constraint velocity
    ChppGikTools::Vector3toUblas(attJoint->jointVelocity().rotationVelocity(),rotvel);
    ChppGikTools::CrossProduct(rotvel,curpos,curvel);

    //std::cout << "curvel " << curvel << std::endl;

    //constraint acceleration
    curaccel.clear();
    ChppGikTools::Vector3toUblas(attJoint->jointAcceleration().rotationAcceleration(),rotaccel);
    ChppGikTools::CrossProduct(rotaccel,curpos,curaccel);
    ChppGikTools::CrossProduct(rotvel,curvel,temp);
    curaccel.plus_assign(temp);

    subrange(attVectorizedState,0,3) = curpos;
    subrange(attVectorizedState,3,6) = curvel;
    subrange(attVectorizedState,6,9) = curaccel;



}


bool ChppGikParallelConstraint::vectorizedTarget ( const vectorN& inVector )
{
    if ( inVector.size() !=3 )
    {
        std::cout <<"ChppGikParallelConstraint::vectorizedTarget(inVector) wrong size\n";
        return false;
    }

    double n = norm_2(inVector);
    if (n==0)
    {
        std::cout << "ChppGikParallelConstraint::vectorizedTarget(inVector) given target vector is null. Aborting target assignement \n";
        return false;
    }
    ChppGikTools::UblastoVector3(inVector, attTargetVectorVector3);
    attTargetVector =  inVector/n;
    attVectorizedTarget = inVector;
    return true;
}

void ChppGikParallelConstraint::computeVectorizedTarget()
{
    attVectorizedTarget = attTargetVector;
}
