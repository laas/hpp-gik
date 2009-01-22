#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikPointingConstraint.h"
#include "hppGikTools.h"

using namespace boost::numeric::ublas;

ChppGikPointingConstraint::ChppGikPointingConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint,  const vector3d& inLocalOrigin, const vector3d& inLocalVector, const vector3d& inTargetWorldPoint) :  ChppGikJointStateConstraint(inRobot, inJoint, 3)
{
    attLocalOriginVector3 = inLocalOrigin;
    attLocalVectorVector3 = inLocalVector;
    attWorldTargetVector3 = inTargetWorldPoint;

    attLocalOrigin.resize(3,false);
    attLocalVector.resize(3,false);
    attWorldTarget.resize(3,false);

    ChppGikTools::Vector3toUblas( attLocalOriginVector3,attLocalOrigin);
    ChppGikTools::Vector3toUblas( attLocalVectorVector3,attLocalVector);
    ChppGikTools::Vector3toUblas( attWorldTargetVector3,attWorldTarget);

    attJacobian.resize(3,robot().numberDof(),false);
    attValue.resize(3, false);

    tempJacobian.resize(6,robot().numberDof(),false);
    tempRot.resize(3,3,false);
    matOP.resize(3,3,false);
    matOT.resize(3,3,false);
    matFO.resize(3,3,false);
    jointrot.resize(3,3,false);
    rotF.resize(3,3,false);
    jointpos.resize(3,false);
    posO.resize(3,false);
    posF.resize(3,false);
    posP.resize(3,false);
    vecFO.resize(3,false);
    vecOP.resize(3,false);
    vecOT.resize(3,false);

    computeValue();
    double NvecOP = norm_2(vecOP);
    if ( NvecOP < 1e-2 )
    {
        std::cout << "Warning ChppGikPointingConstraint: constructor: local vector must not be null. Recreate this object with appropriate input.\n";
    }

    attVectorizedState.resize(9,false);
    attVectorizedTarget.resize(3,false);
}

CjrlGikStateConstraint* ChppGikPointingConstraint::clone() const
{
    CjrlGikPointingConstraint* ret = new ChppGikPointingConstraint(*this);
    return ret;
}


void  ChppGikPointingConstraint::localOrigin(const vector3d& inPoint)
{
    attLocalOriginVector3 = inPoint;
    ChppGikTools::Vector3toUblas(attLocalOriginVector3, attLocalOrigin);
}

const vector3d& ChppGikPointingConstraint::localOrigin()
{
    return attLocalOriginVector3;
}

void  ChppGikPointingConstraint::localVector(const vector3d& inPoint)
{
    attLocalVectorVector3 = inPoint;
    ChppGikTools::Vector3toUblas(attLocalVectorVector3, attLocalVector);
}

const vector3d& ChppGikPointingConstraint::localVector()
{
    return attLocalVectorVector3;
}

void  ChppGikPointingConstraint::worldTarget(const vector3d& inPoint)
{
    attWorldTargetVector3 = inPoint;
    ChppGikTools::Vector3toUblas(attWorldTargetVector3, attWorldTarget);
}

const vector3d& ChppGikPointingConstraint::worldTarget()
{
    return attWorldTargetVector3;
}

void ChppGikPointingConstraint::computeValue()
{
    ChppGikTools::HtoRT(joint()->currentTransformation(),jointrot,jointpos);
    noalias(posO) = prod(jointrot,attLocalOrigin);//origin point in world
    posO.plus_assign(jointpos);
    noalias(posP) =  prod(jointrot,attLocalVector);//second point in world
    posP.plus_assign(posO);
    vecOP = posP - posO;
    vecOT = attWorldTarget - posO;
    double normOP = norm_2(vecOP);
    double normOT = norm_2(vecOT);

    if (normOP != 0)
        vecOP /= normOP;
    if (normOT != 0)
        vecOT /= normOT;
    ChppGikTools::CrossProduct(vecOT,vecOP,attValue);
}


void ChppGikPointingConstraint::computeJacobian()
{
    robot().getJacobian( *attRootJoint,*joint(),attLocalOriginVector3,tempJacobian);

    //compute the jacobian of this constraint
    noalias(posP) =  prod(jointrot,attLocalVector);//second point in world
    posP.plus_assign(posO);
    vecOP = posP - posO;
    vecOT = attWorldTarget - posO;

    double normOP = norm_2(vecOP);
    double normOT = norm_2(vecOT);

    if (normOP != 0)
        vecOP /= normOP;
    if (normOT != 0)
        vecOT /= normOT;

    ChppGikTools::equivAsymMat(vecOT,matOT);
    ChppGikTools::equivAsymMat(vecOP,matOP);

    noalias(tempRot) = prod(matOT,matOP);

    noalias(attJacobian) = prod(tempRot,subrange(tempJacobian,3,6,0,robot().numberDof()));
    noalias(attJacobian) -= prod(matOP,subrange(tempJacobian,0,3,0,robot().numberDof()));
}

void ChppGikPointingConstraint::computeVectorizedState()
{
    vectorN curpos(3);
    vectorN curvel(3);
    vectorN curaccel(3);

    //matrixNxP tempMat4(4,4);
    vectorN lever(3);
    vectorN rotvel(3);
    vectorN rotvelCrossLocal(3);
    vectorN rotaccel(3);
    vectorN temp(3);

    //Check for bad configurations
    computeValue();
    //double NvecOP = norm_2(vecOP);
    //double NvecOT = norm_2(vecOT);

    //constraint position
    curpos = posP;//(NvecOT/NvecOP)*vecOP + posO;
    lever = curpos - jointpos;
    //constraint velocity
    ChppGikTools::Vector3toUblas(joint()->jointVelocity().linearVelocity(),curvel);
    ChppGikTools::Vector3toUblas(joint()->jointVelocity().rotationVelocity(),rotvel);
    ChppGikTools::CrossProduct(rotvel,lever,rotvelCrossLocal);
    curvel += rotvelCrossLocal;

    //constraint acceleration
    ChppGikTools::Vector3toUblas(joint()->jointAcceleration().linearAcceleration(),curaccel);
    ChppGikTools::Vector3toUblas(joint()->jointAcceleration().rotationAcceleration(),rotaccel);
    ChppGikTools::CrossProduct(rotaccel,lever,temp);
    curaccel += temp;
    ChppGikTools::CrossProduct(rotvel,rotvelCrossLocal,temp);
    curaccel += temp;

    subrange(attVectorizedState,0,3) = curpos;
    subrange(attVectorizedState,3,6) = curvel;
    subrange(attVectorizedState,6,9) = curaccel;


}

bool ChppGikPointingConstraint::vectorizedTarget ( const vectorN& inVector )
{
    if ( inVector.size() !=3 )
    {
        std::cout <<"ChppGikPointingConstraint::vectorizedTarget(inVector) wrong size\n";
        return false;
    }

    attWorldTarget = inVector;
    ChppGikTools::UblastoVector3(attWorldTarget, attWorldTargetVector3);
    attVectorizedTarget = inVector;
    return true;
}

void ChppGikPointingConstraint::computeVectorizedTarget()
{
    attVectorizedTarget = attWorldTarget;
}



ChppGikPointingConstraint::~ChppGikPointingConstraint()
{}
