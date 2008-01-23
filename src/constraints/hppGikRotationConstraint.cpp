#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikRotationConstraint.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikRotationConstraint::ChppGikRotationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const matrix3d& inTargetOrientation):ChppGikJointStateConstraint(inRobot, inJoint)
{
    attTargetOrientationMatrix3 = inTargetOrientation;
    attTargetOrientation.resize(3,3,false);

    ChppGikTools::Matrix3toUblas(attTargetOrientationMatrix3, attTargetOrientation);

    attJacobian.resize(3,attNumberActuatedDofs,false);
    attValue.resize(3, false);

    temp3DVec.resize(3,false);
    tempRot.resize(3,3,false);
    tempGapRot.resize(3,3,false);

    attVectorizedState.resize(9,false);
    attVectorizedTarget.resize(3,false);

    attDimension = 3;
}

CjrlGikStateConstraint* ChppGikRotationConstraint::clone() const
{
    CjrlGikRotationConstraint* ret = new ChppGikRotationConstraint(*this);
    return ret;
}

void  ChppGikRotationConstraint::targetOrientation(const matrix3d& inTargetOrientation)
{
    attTargetOrientationMatrix3 = inTargetOrientation;
    ChppGikTools::Matrix3toUblas(attTargetOrientationMatrix3, attTargetOrientation);
}

const matrix3d& ChppGikRotationConstraint::targetOrientation()
{
    return attTargetOrientationMatrix3;
}


void ChppGikRotationConstraint::computeValue()
{
    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);

    noalias(tempGapRot) =  prod(trans(tempRot),attTargetOrientation);

    ChppGikTools::RottoOmega(tempGapRot,temp3DVec);

    noalias(attValue) = prod(tempRot,temp3DVec);
}


void ChppGikRotationConstraint::computeJacobian()
{
    tempFixedJoint = &(attRobot->fixedJoint(0));
    if (!tempFixedJoint)
    {
        std::cout << "ChppGikRotationConstraint::computeJacobian() expected a fixed joint on the robot.\n";
        return;
    }

    attJoint->computeJacobianJointWrtConfig();
    //tempFixedJoint->computeJacobianJointWrtConfig();

    tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
    tempEffectorJointJacobian = &(attJoint->jacobianJointWrtConfig());
    if (!tempFixedJointJacobian || !tempEffectorJointJacobian)
    {
        std::cout << "ChppGikRotationConstraint::computeJacobian() could not retrieve partial jacobians.\n";
        return;
    }

    noalias(attJacobian) = subrange(*tempEffectorJointJacobian,3,6,6,attRobot->numberDof()) - subrange(*tempFixedJointJacobian,3,6,6,attRobot->numberDof());
}

void ChppGikRotationConstraint::computeVectorizedState()
{
    vectorN curEuler(3);
    vectorN curEulerVel(3);
    vectorN curEulerAccel(3);
    vectorN targetEuler(3);

    vectorN omega(3);
    vectorN dotOmega(3);
    matrixNxP tmpRot(3,3);

    //constraint position
    ChppGikTools::HtoRT(attJoint->currentTransformation(),tmpRot,temp3DVec);
    ChppGikTools::RottoEulerZYX(tmpRot, curEuler);
    //constraint velocity
    ChppGikTools::Vector3toUblas(attJoint->jointVelocity().rotationVelocity(),omega);
    ChppGikTools::OmegatoEulerZYX(omega,curEulerVel);
    //constraint acceleration
    ChppGikTools::Vector3toUblas(attJoint->jointAcceleration().rotationAcceleration(),dotOmega);
    ChppGikTools::OmegatoEulerZYX(dotOmega,curEulerAccel);


    subrange(attVectorizedState,0,3) = curEuler;
    subrange(attVectorizedState,3,6) = curEulerVel;
    subrange(attVectorizedState,6,9) = curEulerAccel;

    
}

bool ChppGikRotationConstraint::vectorizedTarget ( const vectorN& inVector )
{
    if ( inVector.size() !=3 )
    {
        std::cout <<"ChppGikRotationConstraint::vectorizedTarget(inVector) wrong size\n";
        return false;
    }

    ChppGikTools::EulerZYXtoRot ( inVector,attTargetOrientation );
    ChppGikTools::UblastoMatrix3(attTargetOrientation, attTargetOrientationMatrix3);
    attVectorizedTarget = inVector;
    return true;
}

void ChppGikRotationConstraint::computeVectorizedTarget()
{
    ChppGikTools::RottoEulerZYX ( attTargetOrientation, temp3DVec );
    attVectorizedTarget = temp3DVec;

}
