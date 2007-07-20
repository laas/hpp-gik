#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikPositionConstraint.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikPositionConstraint::ChppGikPositionConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inPointInWorldFrame)
{
    attJoint = &inJoint;
    attRobot = &inRobot;

    if (attRobot->countFixedJoints()){
	tempNumJoints = inRobot.numberDof()-6;
    }else{
	tempNumJoints = 6;
    }

    attLocalPointVector3 =  inPointInBodyLocalFrame;
    attWorldTargetVector3 =  inPointInWorldFrame;

    attLocalPoint.resize(3,false);
    attWorldTarget.resize(3,false);
    ChppGikTools::Vector3toUblas(attLocalPointVector3, attLocalPoint);
    ChppGikTools::Vector3toUblas(attWorldTargetVector3, attWorldTarget);


    attJacobian.resize(3,tempNumJoints,false);
    tempJacobian.resize(6,inRobot.numberDof(),false);
    tempJacobian.clear();
    attValue.resize(3, false);

    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);
    attInfluencingDofs = ublas::scalar_vector<double>(inRobot.numberDof(),0);
}

CjrlGikStateConstraint* ChppGikPositionConstraint::clone() const
{
    CjrlGikPositionConstraint* ret = new ChppGikPositionConstraint(*this);
    return ret;
}

unsigned int ChppGikPositionConstraint::dimension() const
{
    return 3;
}


CjrlDynamicRobot& ChppGikPositionConstraint::robot()
{
    return *attRobot;
}

void  ChppGikPositionConstraint::joint(CjrlJoint* inJoint)
{
    attJoint = inJoint;
}

CjrlJoint* ChppGikPositionConstraint::joint()
{
    return attJoint;
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

void  ChppGikPositionConstraint::worldTargetU(const vectorN& inPoint)
{
    if (inPoint.size() !=3)
    {
        std::cout << "ChppGikPositionConstraint::worldTargetU() received a vector of incorrect size\n";
        return;
    }
    attWorldTarget = inPoint;
    ChppGikTools::UblastoVector3(attWorldTarget, attWorldTargetVector3);
}

const vectorN& ChppGikPositionConstraint::worldTargetU()
{
    return attWorldTarget;
}


void ChppGikPositionConstraint::computeValue()
{
    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);
    attValue = attWorldTarget;
    attValue -= temp3DVec;
    noalias(temp3DVec1) = prod(tempRot,attLocalPoint);
    attValue -= temp3DVec1;

}

vectorN& ChppGikPositionConstraint::influencingDofs()
{
    unsigned int i;
    std::vector<CjrlJoint *> joints;
    attInfluencingDofs.clear();
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
void ChppGikPositionConstraint::computeJacobian()
{
    attJoint->getJacobianPointWrtConfig(attLocalPointVector3, tempJacobian);

    int start = attRobot->numberDof() - tempNumJoints;
    attJacobian = subrange(tempJacobian,0,3,start,attRobot->numberDof());
    

    if (attRobot->countFixedJoints()>0){
	tempFixedJoint = &(attRobot->fixedJoint(0));
	tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
	if (!tempFixedJointJacobian)
	{
	    std::cout << "ChppGikPositionConstraint::computeJacobian() could not retrieve partial jacobians.\n";
	    return;
	}
	attJacobian.minus_assign(subrange(*tempFixedJointJacobian,
					  0,3,start,attRobot->numberDof()));
    
	ChppGikTools::HtoRT(attJoint->currentTransformation(),
			    tempRot,temp3DVec);
	noalias(temp3DVec1) = prod(tempRot,attLocalPoint);
	temp3DVec.plus_assign(temp3DVec1);//joint point in world

	ChppGikTools::HtoT(tempFixedJoint->currentTransformation(),temp3DVec1);
	temp3DVec.minus_assign(temp3DVec1);//joint point in world - ankle joint center in world
    
	ChppGikTools::equivAsymMat(temp3DVec,tempRot);
	noalias(attJacobian) += prod(tempRot,subrange(*tempFixedJointJacobian,3,6,start,attRobot->numberDof()));
    }
    
//     std::cout << "attJacobian\n";
//     ChppGikTools::printBlasMat( attJacobian );
//     std::cout << "Config of computations\n";
//     std::cout << attRobot->currentConfiguration() << std::endl ;
}



const vectorN& ChppGikPositionConstraint::value()
{
    return attValue;
}


const matrixNxP& ChppGikPositionConstraint::jacobian()
{
    return attJacobian;
}

bool ChppGikPositionConstraint::minimumJerkInterpolation(ChppGikMotionConstraint* outMotionConstraint, double inSamplingPeriod, double inTime)
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
    for (unsigned int i=0; i<3;i++)
    {
        bool contn = ChppGikTools::minJerkCurve( inTime,inSamplingPeriod,curpos(i),curvel(i),curaccel(i),attWorldTarget(i),result);
        if (!contn)
            return false;
        row(data,i) = result;
        
    }
    //filling in motion constraint with interpolation samples
    ChppGikPositionConstraint* tempSC = new ChppGikPositionConstraint(*this);

    for (unsigned int i=0; i<nsamples;i++)
    {
        tempSC->worldTargetU(column(data,i));
        outMotionConstraint->pushbackStateConstraint((CjrlGikPositionConstraint*)tempSC);
    }
    delete tempSC;

    return true;
}
