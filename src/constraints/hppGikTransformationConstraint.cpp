#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikTransformationConstraint.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikTransformationConstraint::ChppGikTransformationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inPointInWorldFrame, const matrix3d& inTargetOrientation):ChppGikJointStateConstraint(inRobot, inJoint)
{
    attLocalPointVector3 =  inPointInBodyLocalFrame;
    attWorldTargetVector3 =  inPointInWorldFrame;
    attTargetOrientationMatrix3 = inTargetOrientation;

    attLocalPoint.resize(3,false);
    attWorldTarget.resize(3,false);
    attTargetOrientation.resize(3,3,false);
    ChppGikTools::Vector3toUblas(attLocalPointVector3, attLocalPoint);
    ChppGikTools::Vector3toUblas(attWorldTargetVector3, attWorldTarget);
    ChppGikTools::Matrix3toUblas(attTargetOrientationMatrix3, attTargetOrientation);

    attJacobian.resize(6,attNumberActuatedDofs,false);
    tempJacobian.resize(6,inRobot.numberDof(),false);
    attValue.resize(6, false);
    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);
    tempGapRot.resize(3,3,false);

    attVectorizedState.resize(18,false);
    attVectorizedTarget.resize(6,false);
    
    attDimension = 6;
    
}

ChppGikTransformationConstraint::ChppGikTransformationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame,  const matrix4d& inTransformation):ChppGikJointStateConstraint(inRobot, inJoint)
{
    attLocalPointVector3 =  inPointInBodyLocalFrame;

    attLocalPoint.resize(3,false);
    attWorldTarget.resize(3,false);
    attTargetOrientation.resize(3,3,false);

    ChppGikTools::Vector3toUblas(attLocalPointVector3, attLocalPoint);
    ChppGikTools::HtoRT(inTransformation,attTargetOrientation,attWorldTarget);
    ChppGikTools::UblastoVector3(attWorldTarget, attWorldTargetVector3);
    ChppGikTools::UblastoMatrix3(attTargetOrientation, attTargetOrientationMatrix3);

    attJacobian.resize(6,attNumberActuatedDofs,false);
    attValue.resize(6, false);

    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);
    tempGapRot.resize(3,3,false);
   
    attVectorizedState.resize(18,false);
    attVectorizedTarget.resize(6,false);
    
    attDimension = 6;

}

CjrlGikStateConstraint* ChppGikTransformationConstraint::clone() const
{
    CjrlGikTransformationConstraint* ret = new ChppGikTransformationConstraint(*this);
    return ret;
}

void  ChppGikTransformationConstraint::localPoint(const vector3d& inPoint)
{
    attLocalPointVector3 = inPoint;
    ChppGikTools::Vector3toUblas(attLocalPointVector3, attLocalPoint);
}

const vector3d& ChppGikTransformationConstraint::localPoint()
{
    return attLocalPointVector3;
}

void  ChppGikTransformationConstraint::worldTarget(const vector3d& inPoint)
{
    attWorldTargetVector3 = inPoint;
    ChppGikTools::Vector3toUblas(attWorldTargetVector3, attWorldTarget);
}

const vector3d& ChppGikTransformationConstraint::worldTarget()
{
    return attWorldTargetVector3;
}

void  ChppGikTransformationConstraint::targetOrientation(const matrix3d& inTargetOrientation)
{
    attTargetOrientationMatrix3 = inTargetOrientation;
    ChppGikTools::Matrix3toUblas(attTargetOrientationMatrix3, attTargetOrientation);
}

const matrix3d& ChppGikTransformationConstraint::targetOrientation()
{
    return attTargetOrientationMatrix3;
}



void  ChppGikTransformationConstraint::worldTargetU(const vectorN& inPoint)
{
    attWorldTarget = inPoint;
    ChppGikTools::UblastoVector3(attWorldTarget, attWorldTargetVector3);
}

const vectorN& ChppGikTransformationConstraint::worldTargetU()
{
    return attWorldTarget;
}

void  ChppGikTransformationConstraint::targetOrientationU(const matrixNxP& inTargetOrientation)
{
    attTargetOrientation = inTargetOrientation;
    ChppGikTools::UblastoMatrix3(attTargetOrientation, attTargetOrientationMatrix3);
}

const matrixNxP& ChppGikTransformationConstraint::targetOrientationU()
{
    return attTargetOrientation;
}


void ChppGikTransformationConstraint::computeValue()
{
    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);

    noalias(temp3DVec1) = prod(tempRot,attLocalPoint);

    subrange(attValue,0,3) =  attWorldTarget;
    subrange(attValue,0,3).minus_assign(temp3DVec1);
    subrange(attValue,0,3).minus_assign(temp3DVec);

    noalias(tempGapRot) =  prod(trans(tempRot),attTargetOrientation);
    ChppGikTools::RottoOmega(tempGapRot,temp3DVec);

    noalias(subrange(attValue,3,6)) = prod(tempRot,temp3DVec);
}




void ChppGikTransformationConstraint::computeJacobian()
{

    tempFixedJoint = &(attRobot->fixedJoint(0));
    if (!tempFixedJoint)
    {
        std::cout << "ChppGikTransformationConstraint::computeJacobian() expected a fixed joint on the robot.\n";
        return;
    }
    tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
    if (!tempFixedJointJacobian)
    {
        std::cout << "ChppGikTransformationConstraint::computeJacobian() could not retrieve partial jacobians.\n";
        return;
    }

    attJoint->getJacobianPointWrtConfig(attLocalPointVector3, tempJacobian);

    attJacobian = subrange(tempJacobian,0,6,6,attRobot->numberDof());
    attJacobian.minus_assign(subrange(*tempFixedJointJacobian,0,6,6,attRobot->numberDof()));

    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);
    noalias(temp3DVec1) = prod(tempRot,attLocalPoint);
    temp3DVec.plus_assign(temp3DVec1);//joint point in world

    ChppGikTools::HtoT(tempFixedJoint->currentTransformation(),temp3DVec1);
    temp3DVec.minus_assign(temp3DVec1);//joint point in world - ankle joint center in world

    ChppGikTools::equivAsymMat(temp3DVec,tempRot);

    noalias(subrange(attJacobian,0,3,0,attNumberActuatedDofs)) += prod(tempRot,subrange(*tempFixedJointJacobian,3,6,6,attRobot->numberDof()));

}


const vectorN& ChppGikTransformationConstraint::vectorizedState()
{
    vectorN curpos(3);
    vectorN curvel(3);
    vectorN curaccel(3);
    vectorN curEuler(3);
    vectorN curEulerVel(3);
    vectorN curEulerAccel(3);
    vectorN targetEuler(3);

    vectorN worldLocalPoint(3);
    vectorN rotvel(3);
    vectorN rotvelCrossLocal(3);
    vectorN rotaccel(3);

    //constraint position
    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,curpos);
    worldLocalPoint = prod(tempRot, attLocalPoint);
    curpos += worldLocalPoint;
    ChppGikTools::RottoEulerZYX(tempRot, curEuler);
    //constraint velocity
    ChppGikTools::Vector3toUblas(attJoint->jointVelocity().linearVelocity(),curvel);
    ChppGikTools::Vector3toUblas(attJoint->jointVelocity().rotationVelocity(),rotvel);
    ChppGikTools::CrossProduct(rotvel,worldLocalPoint,rotvelCrossLocal);
    curvel += rotvelCrossLocal;
    ChppGikTools::OmegatoEulerZYX(rotvel,curEulerVel);
    //constraint acceleration
    ChppGikTools::Vector3toUblas(attJoint->jointAcceleration().linearAcceleration(),curaccel);
    ChppGikTools::Vector3toUblas(attJoint->jointAcceleration().rotationAcceleration(),rotaccel);
    ChppGikTools::CrossProduct(rotaccel,worldLocalPoint,temp3DVec);
    curaccel += temp3DVec;
    ChppGikTools::CrossProduct(rotvel,rotvelCrossLocal,temp3DVec);
    curaccel += temp3DVec;
    ChppGikTools::OmegatoEulerZYX(rotaccel,curEulerAccel);
    
    subrange(attVectorizedState,0,3) = curpos;
    subrange(attVectorizedState,3,6) = curEuler;
    subrange(attVectorizedState,6,9) = curvel;
    subrange(attVectorizedState,9,12) = curEulerVel;
    subrange(attVectorizedState,12,15) = curaccel;
    subrange(attVectorizedState,15,18) = curEulerAccel;
    
    return attVectorizedState;
}

const vectorN& ChppGikTransformationConstraint::vectorizedTarget()
{
    subrange(attVectorizedTarget,0,3) = attWorldTarget;
    ChppGikTools::RottoEulerZYX(attTargetOrientation, temp3DVec1);
    subrange(attVectorizedTarget,3,6) = temp3DVec1;
    return attVectorizedTarget;
}

bool ChppGikTransformationConstraint::vectorizedTarget(const vectorN& inVector )
{
    if ( inVector.size() !=6 )
    {
        std::cout <<"ChppGikTransformationConstraint::vectorizedTarget(inVector) incorrect input size\n";
        return false;
    }

    temp3DVec = subrange(inVector,0,3);
    temp3DVec1 = subrange(inVector,3,6);
    ChppGikTools::EulerZYXtoRot ( temp3DVec1,tempRot );
    worldTargetU ( temp3DVec );
    targetOrientationU ( tempRot );

    return true;
}
