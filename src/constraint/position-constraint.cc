#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "hpp/gik/constraint/position-constraint.hh"
#include "hpp/gik/tools.hh"

using namespace boost::numeric::ublas;

ChppGikPositionConstraint::ChppGikPositionConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inPointInWorldFrame) : ChppGikJointStateConstraint(inRobot, inJoint, 3)
{
    attLocalPointVector3 =  inPointInBodyLocalFrame;
    attWorldTargetVector3 =  inPointInWorldFrame;

    attLocalPoint.resize(3,false);
    attWorldTarget.resize(3,false);
    ChppGikTools::Vector3toUblas(attLocalPointVector3, attLocalPoint);
    ChppGikTools::Vector3toUblas(attWorldTargetVector3, attWorldTarget);


    attJacobian.resize(3,inRobot.numberDof(),false);
    attValue.resize(3, false);

    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);

    attVectorizedState.resize(9,false);
    attVectorizedTarget.resize(3,false);
}

CjrlGikStateConstraint* ChppGikPositionConstraint::clone() const
{
    CjrlGikPositionConstraint* ret = new ChppGikPositionConstraint(*this);
    return ret;
}

void  ChppGikPositionConstraint::localPoint(const vector3d& inPoint)
{
    attLocalPointVector3 = inPoint;
    ChppGikTools::Vector3toUblas(attLocalPointVector3, attLocalPoint);
}

const vector3d& ChppGikPositionConstraint::localPoint()
{
    return attLocalPointVector3;
}

void  ChppGikPositionConstraint::worldTarget(const vector3d& inPoint)
{
    attWorldTargetVector3 = inPoint;
    ChppGikTools::Vector3toUblas(attWorldTargetVector3, attWorldTarget);
}

const vector3d& ChppGikPositionConstraint::worldTarget()
{
    return attWorldTargetVector3;
}


void ChppGikPositionConstraint::computeValue()
{
    ChppGikTools::HtoRT(joint()->currentTransformation(),tempRot,temp3DVec);
    attValue = -attWorldTarget;
    attValue += temp3DVec;
    noalias(temp3DVec1) = prod(tempRot,attLocalPoint);
    attValue += temp3DVec1;
}

void ChppGikPositionConstraint::computeJacobian()
{
    robot().getPositionJacobian( *attRootJoint, *joint(), attLocalPointVector3,attJacobian);
}


void ChppGikPositionConstraint::computeVectorizedState()
{
    vectorN curpos(3);
    vectorN curvel = zero_vector<double>(3);
    vectorN curaccel = zero_vector<double>(3);

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
    //std::cout << "curaccel " << curaccel << "\n";
    ChppGikTools::Vector3toUblas(joint()->jointAcceleration().rotationAcceleration(),rotaccel);
    ChppGikTools::CrossProduct(rotaccel,worldLocalPoint,temp);
    curaccel += temp;
    //std::cout << "curaccel " << curaccel << "\n";
    ChppGikTools::CrossProduct(rotvel,rotvelCrossLocal,temp);
    curaccel += temp;

    //std::cout << "curaccel " << curaccel << "\n";

    subrange(attVectorizedState,0,3) = curpos;
    subrange(attVectorizedState,3,6) = curvel;
    subrange(attVectorizedState,6,9) = curaccel;
}

bool ChppGikPositionConstraint::vectorizedTarget ( const vectorN& inVector )
{
    if ( inVector.size() !=3 )
    {
        std::cout <<"ChppGikPositionConstraint::vectorizedTarget(inVector) wrong size\n";
        return false;
    }

    attWorldTarget = inVector;
    ChppGikTools::UblastoVector3(attWorldTarget, attWorldTargetVector3);
    attVectorizedTarget = inVector;
    return true;
}

void ChppGikPositionConstraint::computeVectorizedTarget()
{
    attVectorizedTarget = attWorldTarget;
}
