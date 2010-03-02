#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikPlaneConstraint.h"
#include "hppGikTools.h"

using namespace boost::numeric::ublas;

ChppGikPlaneConstraint::ChppGikPlaneConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inWorldPlanePoint, const vector3d& inWorldPlaneNormal):ChppGikJointStateConstraint(inRobot, inJoint, 1)
{
    attLocalPoint.resize(3,false);
    attWorldPlanePoint.resize(3,false);
    attWorldPlaneNormal.resize(3,false);
    attWorldPlaneNormalTrans.resize(1,3);

    localPoint(inPointInBodyLocalFrame);
    worldPlanePoint(inWorldPlanePoint);
    worldPlaneNormal(inWorldPlaneNormal);

    tempJacobian.resize(6,inRobot.numberDof(),false);
    tempJointPositionJacobian.resize(3,inRobot.numberDof(),false);
    attJacobian.resize(1,inRobot.numberDof(),false);
    attValue.resize(1, false);

    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);

    attVectorizedState.resize(18,false);
    attVectorizedTarget.resize(6,false);

}

CjrlGikStateConstraint* ChppGikPlaneConstraint::clone() const
{
    CjrlGikPlaneConstraint* ret = new ChppGikPlaneConstraint(*this);
    return ret;
}

void  ChppGikPlaneConstraint::localPoint(const vector3d& inPoint)
{
    attLocalPointVector3 = inPoint;
    ChppGikTools::Vector3toUblas(attLocalPointVector3, attLocalPoint);
}

const vector3d& ChppGikPlaneConstraint::localPoint()
{
    return attLocalPointVector3;
}

void  ChppGikPlaneConstraint::worldPlanePoint(const vector3d& inPlanePoint)
{
    attWorldPlanePointVector3 = inPlanePoint;
    ChppGikTools::Vector3toUblas(attWorldPlanePointVector3, attWorldPlanePoint);
}

const vector3d& ChppGikPlaneConstraint::worldPlanePoint()
{
    return attWorldPlanePointVector3;
}

void  ChppGikPlaneConstraint::worldPlaneNormal(const vector3d& inPlaneNormal)
{
    attWorldPlaneNormalVector3 = inPlaneNormal;
    ChppGikTools::Vector3toUblas(attWorldPlaneNormalVector3, attWorldPlaneNormal);
    attWorldPlaneNormal /=  norm_2(attWorldPlaneNormal);
    ChppGikTools::UblastoVector3(attWorldPlaneNormal,attWorldPlaneNormalVector3);
    for (unsigned int i=0; i<attWorldPlaneNormal.size(); i++)
    {
        attWorldPlaneNormalTrans(0, i) = attWorldPlaneNormal(i);
    }
}

const vector3d& ChppGikPlaneConstraint::worldPlaneNormal()
{
    return attWorldPlaneNormalVector3;
}

void ChppGikPlaneConstraint::computeValue()
{

    ChppGikTools::HtoRT(joint()->currentTransformation(),tempRot,temp3DVec);
    noalias(temp3DVec1) = prod(tempRot,attLocalPoint);
    temp3DVec += temp3DVec1;
    attValue(0) = inner_prod(temp3DVec-attWorldPlanePoint, attWorldPlaneNormal);
}

//certainly not optimal here. should be moved to Humanoid robot.
void ChppGikPlaneConstraint::computeJacobian()
{
    robot().getPositionJacobian( *attRootJoint,*joint(),attLocalPointVector3,tempJointPositionJacobian);
    noalias(attJacobian) = prod(attWorldPlaneNormalTrans,tempJointPositionJacobian);
}


void ChppGikPlaneConstraint::computeVectorizedState()
{
    vectorN curpos(3);
    vectorN zeroVec = zero_vector<double>(3);
    vectorN curvel = zeroVec;
    vectorN curaccel = zeroVec;

    //matrixNxP tempMat4(4,4);
    vectorN worldLocalPoint(3);
    vectorN rotvel(3);
    vectorN rotvelCrossLocal(3);
    vectorN rotaccel(3);
    vectorN temp(3);
    matrixNxP tmpRot(3,3);

    //constraint position
    ChppGikTools::HtoRT(joint()->currentTransformation(),tmpRot,curpos);
    worldLocalPoint = prod(tmpRot, attLocalPoint);
    curpos += worldLocalPoint;

    //constraint velocity
    ChppGikTools::Vector3toUblas(joint()->jointVelocity().linearVelocity(),curvel);
    ChppGikTools::Vector3toUblas(joint()->jointVelocity().rotationVelocity(),rotvel);
    ChppGikTools::CrossProduct(rotvel,worldLocalPoint,rotvelCrossLocal);
    curvel += rotvelCrossLocal;

    //constraint acceleration
    ChppGikTools::Vector3toUblas(joint()->jointAcceleration().linearAcceleration(),curaccel);
    ChppGikTools::Vector3toUblas(joint()->jointAcceleration().rotationAcceleration(),rotaccel);
    ChppGikTools::CrossProduct(rotaccel,worldLocalPoint,temp);
    curaccel += temp;
    ChppGikTools::CrossProduct(rotvel,rotvelCrossLocal,temp);
    curaccel += temp;

    attVectorizedState.clear();
    subrange(attVectorizedState,0,3) = curpos;
    subrange(attVectorizedState,6,9) = curvel;
    subrange(attVectorizedState,12,15) = curaccel;
}


bool ChppGikPlaneConstraint::vectorizedTarget ( const vectorN& inVector )
{
    if ( inVector.size() !=6 )
    {
        std::cout <<"ChppGikPlaneConstraint::vectorizedTarget(inVector) wrong size\n";
        return false;
    }

    double n = norm_2(subrange(inVector,3,6));
    if (n==0)
    {
        std::cout << "ChppGikPlaneConstraint::vectorizedTarget(inVector) given target plane normal is null. Aborting target assignement \n";
        return false;
    }
    
    attWorldPlaneNormal = subrange(inVector,3,6)/n;
    ChppGikTools::UblastoVector3(attWorldPlaneNormal,attWorldPlaneNormalVector3);
    for (unsigned int i=0; i<attWorldPlaneNormal.size(); i++)
        attWorldPlaneNormalTrans(0, i) = attWorldPlaneNormal(i);
    
    attWorldPlanePoint = subrange(inVector,0,3);
    ChppGikTools::UblastoVector3(attWorldPlanePoint, attWorldPlanePointVector3);
    
    attVectorizedTarget = inVector;
    return true;
}

void ChppGikPlaneConstraint::computeVectorizedTarget()
{
    subrange(attVectorizedTarget,0,3) = attWorldPlanePoint;
    subrange(attVectorizedTarget,3,6) = attWorldPlaneNormal;
}


