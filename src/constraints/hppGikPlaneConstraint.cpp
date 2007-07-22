#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikPlaneConstraint.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikPlaneConstraint::ChppGikPlaneConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inWorldPlanePoint, const vector3d& inWorldPlaneNormal)
{


    attJoint = &inJoint;
    attRobot = &inRobot;

    if (attRobot->countFixedJoints()>0){
	tempNumJoints = attRobot->numberDof()-6;
    }else{
	tempNumJoints = 6;
    }

    attLocalPoint.resize(3,false);
    attWorldPlanePoint.resize(3,false);
    attWorldPlaneNormal.resize(3,false);
    attWorldPlaneNormalTrans.resize(1,3);

    localPoint(inPointInBodyLocalFrame);
    worldPlanePoint(inWorldPlanePoint);
    worldPlaneNormal(inWorldPlaneNormal);

    tempEffectorJointJacobian.resize(3,inRobot.numberDof(),false);
    tempJointPositionJacobian.resize(3,tempNumJoints,false);
    attJacobian.resize(1,tempNumJoints,false);
    attValue.resize(1, false);

    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);
    
    attInfluencingDofs = scalar_vector<double>(inRobot.numberDof(),0);

}
CjrlGikStateConstraint* ChppGikPlaneConstraint::clone() const
{
    CjrlGikPlaneConstraint* ret = new ChppGikPlaneConstraint(*this);
    return ret;
}

unsigned int ChppGikPlaneConstraint::dimension() const
{
    return 1;
}


CjrlDynamicRobot& ChppGikPlaneConstraint::robot()
{
    return *attRobot;
}

void  ChppGikPlaneConstraint::joint(CjrlJoint* inJoint)
{
    attJoint = inJoint;
}

CjrlJoint* ChppGikPlaneConstraint::joint()
{
    return attJoint;
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

vectorN& ChppGikPlaneConstraint::influencingDofs()
{
    unsigned int i;
    attInfluencingDofs.clear();
    std::vector<CjrlJoint *> joints;
    if (attRobot->countFixedJoints()>0){
	joints = attRobot->fixedJoint(0).jointsFromRootToThis();
	for( i=1; i< joints.size(); i++)
	    attInfluencingDofs(joints[i]->rankInConfiguration()) = 1;
    }else{
	CjrlJoint *root = attRobot->rootJoint();
	unsigned int start = root->rankInConfiguration();
	for (i=0; i<root->numberDof(); i++){
	    attInfluencingDofs[start+i] = 1;
	}
    }
    joints = attJoint->jointsFromRootToThis();
    for(i=1; i< joints.size(); i++)
        attInfluencingDofs(joints[i]->rankInConfiguration()) = 1;
    return attInfluencingDofs;
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

    int start = attRobot->numberDof() - tempNumJoints;
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


const vectorN& ChppGikPlaneConstraint::value()
{
    return attValue;
}

const matrixNxP& ChppGikPlaneConstraint::jacobian()
{
    return attJacobian;
}


bool ChppGikPlaneConstraint::minimumJerkInterpolation(ChppGikMotionConstraint* outMotionConstraint, double inSamplingPeriod, double inTime)
{
    unsigned int nsamples = ChppGikTools::timetoRank(0,inTime,inSamplingPeriod)+1;

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

    //motion samples computation
    matrixNxP data(3,nsamples);
    vectorN result(nsamples);
    
    //projection of the effector point on the target plane
    vectorN targetPoint(3);
    vectorN OM(3);
    vectorN OP(3);
    OM = attWorldPlanePoint - curpos;
    OP = OM - inner_prod(OM,attWorldPlaneNormal)*attWorldPlaneNormal;
    targetPoint = attWorldPlanePoint + OP;
    
    for (unsigned int i=0; i<3;i++)
    {
        bool contn = ChppGikTools::minJerkCurve( inTime,inSamplingPeriod,curpos(i),curvel(i),curaccel(i),targetPoint(i),result);
        if (!contn)
            return false;
        row(data,i) = result;
        
    }
    //filling in motion constraint with interpolation samples
    ChppGikPlaneConstraint* tempSC = new ChppGikPlaneConstraint(*this);

    for (unsigned int i=0; i<nsamples;i++)
    {
        tempSC->worldPlaneNormalU(attWorldPlaneNormal); tempSC->worldPlanePointU(column(data,i));
        outMotionConstraint->pushbackStateConstraint((CjrlGikPlaneConstraint*)tempSC);
    }
    delete tempSC;

    return true;
}
