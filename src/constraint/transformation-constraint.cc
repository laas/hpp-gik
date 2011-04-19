#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/constraint/transformation-constraint.hh"
#include "hpp/gik/tools.hh"
#include <time.h>
#include <sys/time.h>
#include <fstream>

#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J

using namespace boost::numeric::ublas;

ChppGikTransformationConstraint::ChppGikTransformationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inPointInWorldFrame, const matrix3d& inTargetOrientation) : ChppGikJointStateConstraint(inRobot, inJoint, 6)
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

    attJacobian.resize(6,inRobot.numberDof(),false);
    attValue.resize(6, false);
    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);
    tempGapRot.resize(3,3,false);

    attVectorizedState.resize(18,false);
    attVectorizedTarget.resize(6,false);
}

ChppGikTransformationConstraint::ChppGikTransformationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const matrix4d& inTransformation) : ChppGikJointStateConstraint(inRobot, inJoint, 6)
{
    attLocalPointVector3 =  inPointInBodyLocalFrame;

    attLocalPoint.resize(3,false);
    attWorldTarget.resize(3,false);
    attTargetOrientation.resize(3,3,false);

    ChppGikTools::Vector3toUblas(attLocalPointVector3, attLocalPoint);
    ChppGikTools::HtoRT(inTransformation,attTargetOrientation,attWorldTarget);
    ChppGikTools::UblastoVector3(attWorldTarget, attWorldTargetVector3);
    ChppGikTools::UblastoMatrix3(attTargetOrientation, attTargetOrientationMatrix3);

    attJacobian.resize(6,inRobot.numberDof(),false);
    attValue.resize(6, false);

    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);
    tempGapRot.resize(3,3,false);

    attVectorizedState.resize(18,false);
    attVectorizedTarget.resize(6,false);
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


void  ChppGikTransformationConstraint::targetTransformation(const matrix4d& inTransformation)
{
    ChppGikTools::HtoRT(inTransformation,attTargetOrientation,attWorldTarget);
    ChppGikTools::UblastoVector3(attWorldTarget, attWorldTargetVector3);
    ChppGikTools::UblastoMatrix3(attTargetOrientation, attTargetOrientationMatrix3);
}

const matrix4d& ChppGikTransformationConstraint::targetTransformation()
{
    for (unsigned int i=0; i< 3; i++)
        for (unsigned int j=0; j< 3; j++)
            M4_IJ(attTargetTransformation4, i, j) = attTargetOrientation(i,j);

    for (unsigned int i=0; i< 3; i++)
        M4_IJ(attTargetTransformation4, i, 3) = attWorldTarget(i);

    return attTargetTransformation4;
}


void ChppGikTransformationConstraint::computeValue()
{
    ChppGikTools::HtoRT(joint()->currentTransformation(),tempRot,temp3DVec);

    noalias(temp3DVec1) = prod(tempRot,attLocalPoint);

    subrange(attValue,0,3) = -attWorldTarget;
    subrange(attValue,0,3).plus_assign(temp3DVec1);
    subrange(attValue,0,3).plus_assign(temp3DVec);

    noalias(tempGapRot) =  prod(trans(tempRot),attTargetOrientation);
    ChppGikTools::RottoOmega(tempGapRot,temp3DVec);

    noalias(subrange(attValue,3,6)) = -prod(tempRot,temp3DVec);
}

void ChppGikTransformationConstraint::computeJacobian()
{
    robot().getJacobian( *attRootJoint,*joint(),attLocalPointVector3,attJacobian);
}


void ChppGikTransformationConstraint::computeVectorizedState()
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
    ChppGikTools::HtoRT(joint()->currentTransformation(),tempRot,curpos);
    worldLocalPoint = prod(tempRot, attLocalPoint);
    curpos += worldLocalPoint;
    ChppGikTools::RottoEulerZYX(tempRot, curEuler);
    //constraint velocity
    ChppGikTools::Vector3toUblas(joint()->jointVelocity().linearVelocity(),curvel);
    ChppGikTools::Vector3toUblas(joint()->jointVelocity().rotationVelocity(),rotvel);
    ChppGikTools::CrossProduct(rotvel,worldLocalPoint,rotvelCrossLocal);
    curvel += rotvelCrossLocal;
    ChppGikTools::OmegatoEulerZYX(rotvel,curEulerVel);
    //constraint acceleration
    ChppGikTools::Vector3toUblas(joint()->jointAcceleration().linearAcceleration(),curaccel);
    ChppGikTools::Vector3toUblas(joint()->jointAcceleration().rotationAcceleration(),rotaccel);
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


}


void ChppGikTransformationConstraint::computeVectorizedTarget()
{
    ChppGikTools::HtoRT(joint()->currentTransformation(),tempRot,temp3DVec);
    ChppGikTools::RottoEulerZYX(tempRot, temp3DVec);

    subrange(attVectorizedTarget,0,3) = attWorldTarget;
    ChppGikTools::RottoEulerZYX(attTargetOrientation, temp3DVec1);

    for (unsigned int i=0; i<3;i++)
    {
        if (temp3DVec(i) > 0)
        {
            if (temp3DVec1(i) < temp3DVec(i) - M_PI)
                temp3DVec1(i) += 2*M_PI; 
        }
        else
        {
            if (temp3DVec1(i) > temp3DVec(i) + M_PI)
                temp3DVec1(i) -= 2*M_PI; 
        }
    }
    
    subrange(attVectorizedTarget,3,6) = temp3DVec1;
}

bool ChppGikTransformationConstraint::vectorizedTarget(const vectorN& inVector )
{
    if ( inVector.size() !=6 )
    {
        std::cout <<"ChppGikTransformationConstraint::vectorizedTarget(inVector) incorrect input size\n";
        return false;
    }

    attVectorizedTarget = inVector;

    attWorldTarget = subrange(inVector,0,3);
    ChppGikTools::UblastoVector3(attWorldTarget, attWorldTargetVector3);

    temp3DVec1 = subrange(inVector,3,6);
    ChppGikTools::EulerZYXtoRot ( temp3DVec1,attTargetOrientation );
    ChppGikTools::UblastoMatrix3(attTargetOrientation, attTargetOrientationMatrix3);

    return true;
}
