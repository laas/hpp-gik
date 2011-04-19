#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/constraint/rotation-constraint.hh"
#include "hpp/gik/tools.hh"

using namespace boost::numeric::ublas;

ChppGikRotationConstraint::ChppGikRotationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint,  const matrix3d& inTargetOrientation) :ChppGikJointStateConstraint(inRobot, inJoint, 3)
{
    attTargetOrientationMatrix3 = inTargetOrientation;
    attTargetOrientation.resize(3,3,false);

    ChppGikTools::Matrix3toUblas(attTargetOrientationMatrix3, attTargetOrientation);

    attJacobian.resize(3,inRobot.numberDof(),false);
    attValue.resize(3, false);

    temp3DVec.resize(3,false);
    tempRot.resize(3,3,false);
    tempGapRot.resize(3,3,false);

    attVectorizedState.resize(9,false);
    attVectorizedTarget.resize(3,false);
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
    ChppGikTools::HtoRT(joint()->currentTransformation(),tempRot,temp3DVec);
    noalias(tempGapRot) =  prod(trans(tempRot),attTargetOrientation);
    ChppGikTools::RottoOmega(tempGapRot,temp3DVec);
    noalias(attValue) = -prod(tempRot,temp3DVec);
}


void ChppGikRotationConstraint::computeJacobian()
{
    robot().getOrientationJacobian( *attRootJoint,*joint(),attJacobian);
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
    ChppGikTools::HtoRT(joint()->currentTransformation(),tmpRot,temp3DVec);
    ChppGikTools::RottoEulerZYX(tmpRot, curEuler);
    //constraint velocity
    ChppGikTools::Vector3toUblas(joint()->jointVelocity().rotationVelocity(),omega);
    ChppGikTools::OmegatoEulerZYX(omega,curEulerVel);
    //constraint acceleration
    ChppGikTools::Vector3toUblas(joint()->jointAcceleration().rotationAcceleration(),dotOmega);
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
    ChppGikTools::HtoRT(joint()->currentTransformation(),tempRot,temp3DVec);
    ChppGikTools::RottoEulerZYX(tempRot, temp3DVec);
    ChppGikTools::RottoEulerZYX ( attTargetOrientation, attVectorizedTarget );

    for (unsigned int i=0; i<3;i++)
    {
        if (temp3DVec(i) > 0)
        {
            if (attVectorizedTarget(i) < temp3DVec(i) - M_PI)
                attVectorizedTarget(i) += 2*M_PI; 
        }
        else
        {
            if (attVectorizedTarget(i) > temp3DVec(i) + M_PI)
                attVectorizedTarget(i) -= 2*M_PI; 
        }
    }
}

