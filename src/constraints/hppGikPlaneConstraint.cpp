#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikPlaneConstraint.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikPlaneConstraint::ChppGikPlaneConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inWorldPlanePoint, const vector3d& inWorldPlaneNormal):ChppGikJointStateConstraint(inRobot, inJoint)
{
    attLocalPoint.resize(3,false);
    attWorldPlanePoint.resize(3,false);
    attWorldPlaneNormal.resize(3,false);
    attWorldPlaneNormalTrans.resize(1,3);

    localPoint(inPointInBodyLocalFrame);
    worldPlanePoint(inWorldPlanePoint);
    worldPlaneNormal(inWorldPlaneNormal);

    tempEffectorJointJacobian.resize(3,inRobot.numberDof(),false);
    tempJointPositionJacobian.resize(3,attNumberActuatedDofs,false);
    attJacobian.resize(1,attNumberActuatedDofs,false);
    attValue.resize(1, false);

    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);
   
    attVectorizedState.resize(9,false);
    attVectorizedTarget.resize(3,false);
    
    attDimension = 1;

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
    attWorldPlaneNormal /=  ublas::norm_2(attWorldPlaneNormal);
    ChppGikTools::UblastoVector3(attWorldPlaneNormal,attWorldPlaneNormalVector3);
    for (unsigned int i=0; i<attWorldPlaneNormal.size(); i++){
	attWorldPlaneNormalTrans(0, i) = attWorldPlaneNormal(i);
    }
}

const vector3d& ChppGikPlaneConstraint::worldPlaneNormal()
{
    return attWorldPlaneNormalVector3;
}


void  ChppGikPlaneConstraint::worldPlanePointU(const vectorN& inPlanePoint)
{
    attWorldPlanePoint = inPlanePoint;
    ChppGikTools::UblastoVector3(attWorldPlanePoint, attWorldPlanePointVector3);
}

const vectorN& ChppGikPlaneConstraint::worldPlanePointU()
{
    return attWorldPlanePoint;
}

void  ChppGikPlaneConstraint::worldPlaneNormalU(const vectorN& inPlaneNormal)
{
    attWorldPlaneNormal = inPlaneNormal;
    attWorldPlaneNormal /= ublas::norm_2(attWorldPlaneNormal);
    ChppGikTools::UblastoVector3(attWorldPlaneNormal,attWorldPlaneNormalVector3);
    for (unsigned int i=0; i<attWorldPlaneNormal.size(); i++){
	attWorldPlaneNormalTrans(0, i) = attWorldPlaneNormal(i);
    }
}

const vectorN& ChppGikPlaneConstraint::worldPlaneNormalU()
{
    return attWorldPlaneNormal;
}

void ChppGikPlaneConstraint::computeValue()
{

    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);
    noalias(temp3DVec1) = prod(tempRot,attLocalPoint);
    temp3DVec += temp3DVec1;
    attValue(0) = inner_prod(attWorldPlanePoint - temp3DVec, attWorldPlaneNormal);
}

//certainly not optimal here. should be moved to Humanoid robot.
void ChppGikPlaneConstraint::computeJacobian()
{
    attJoint->getJacobianPointWrtConfig(attLocalPointVector3, 
					tempEffectorJointJacobian);

    int start = attRobot->numberDof() - attNumberActuatedDofs;
    tempJointPositionJacobian = subrange(tempEffectorJointJacobian,
					 0,3,start,attRobot->numberDof());

    if (attRobot->countFixedJoints()>0){
	tempFixedJoint = &(attRobot->fixedJoint(0));
	tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
	if (!tempFixedJointJacobian)
	{
	    std::cout << "ChppGikPlaneConstraint::computeJacobian() could not retrieve partial jacobians.\n";
	    return;
	}
	tempJointPositionJacobian -= subrange(*tempFixedJointJacobian,
					      0,3,start,attRobot->numberDof());

	ChppGikTools::HtoRT(attJoint->currentTransformation(),
			    tempRot,temp3DVec);
	noalias(temp3DVec1) = prod(tempRot,attLocalPoint);
	temp3DVec += temp3DVec1;

	ChppGikTools::HtoT(tempFixedJoint->currentTransformation(),temp3DVec1);
	temp3DVec -= temp3DVec1;

	ChppGikTools::equivAsymMat(temp3DVec,tempRot);
	tempJointPositionJacobian += prod(tempRot,subrange(*tempFixedJointJacobian,3,6,start,attRobot->numberDof()));
    }

    noalias(attJacobian) = prod(attWorldPlaneNormalTrans,
				tempJointPositionJacobian);
}

const vectorN& ChppGikPlaneConstraint::vectorizedState()
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
    ChppGikTools::HtoRT(attJoint->currentTransformation(),tmpRot,curpos);
    worldLocalPoint = prod(tmpRot, attLocalPoint);
    curpos += worldLocalPoint;

    //constraint velocity
    ChppGikTools::Vector3toUblas(attJoint->jointVelocity().linearVelocity(),curvel);
    ChppGikTools::Vector3toUblas(attJoint->jointVelocity().rotationVelocity(),rotvel);
    ChppGikTools::CrossProduct(rotvel,worldLocalPoint,rotvelCrossLocal);
    curvel += rotvelCrossLocal;

    //constraint acceleration
    ChppGikTools::Vector3toUblas(attJoint->jointAcceleration().linearAcceleration(),curaccel);
    ChppGikTools::Vector3toUblas(attJoint->jointAcceleration().rotationAcceleration(),rotaccel);
    ChppGikTools::CrossProduct(rotaccel,worldLocalPoint,temp);
    curaccel += temp;
    ChppGikTools::CrossProduct(rotvel,rotvelCrossLocal,temp);
    curaccel += temp;

    subrange(attVectorizedState,0,3) = curpos;
    subrange(attVectorizedState,3,6) = curvel;
    subrange(attVectorizedState,6,9) = curaccel;

    return attVectorizedState;

}


bool ChppGikPlaneConstraint::vectorizedTarget ( const vectorN& inVector )
{
    if ( inVector.size() !=3 )
    {
        std::cout <<"ChppGikPlaneConstraint::vectorizedTarget(inVector) wrong size\n";
        return false;
    }

    worldPlanePointU ( inVector );

    return true;
}

const vectorN& ChppGikPlaneConstraint::vectorizedTarget()
{
    return attWorldPlanePoint;
}
