#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikPointingConstraint.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikPointingConstraint::ChppGikPointingConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalOrigin, const vector3d& inLocalVector, const vector3d& inTargetWorldPoint)
{
    attJoint = &inJoint;
    attRobot = &inRobot;

    tempNumJoints = attRobot->numberDof()-6;

    attLocalOriginVector3 = inLocalOrigin;
    attLocalVectorVector3 = inLocalVector;
    attWorldTargetVector3 = inTargetWorldPoint;

    attLocalOrigin.resize(3,false);
    attLocalVector.resize(3,false);
    attWorldTarget.resize(3,false);

    ChppGikTools::Vector3toUblas( attLocalOriginVector3,attLocalOrigin);
    ChppGikTools::Vector3toUblas( attLocalVectorVector3,attLocalVector);
    ChppGikTools::Vector3toUblas( attWorldTargetVector3,attWorldTarget);

    attJacobian.resize(3,tempNumJoints,false);
    attValue.resize(3, false);


    tempJacobian.resize(6,tempNumJoints,false);
    tempJacobian0.resize(6,attRobot->numberDof(),false);
    tempJacobian0.clear();
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

    attInfluencingDofs = ublas::scalar_vector<double>(inRobot.numberDof(),0);
}

CjrlGikStateConstraint* ChppGikPointingConstraint::clone() const
{
    CjrlGikPointingConstraint* ret = new ChppGikPointingConstraint(*this);
    return ret;
}


unsigned int ChppGikPointingConstraint::dimension() const
{
    return 3;
}


CjrlDynamicRobot& ChppGikPointingConstraint::robot()
{
    return *attRobot;
}

void  ChppGikPointingConstraint::joint(CjrlJoint* inJoint)
{
    attJoint = inJoint;
}

CjrlJoint* ChppGikPointingConstraint::joint()
{
    return attJoint;
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
    ChppGikTools::HtoRT(attJoint->currentTransformation(),jointrot,jointpos);
    noalias(posO) = prod(jointrot,attLocalOrigin);//origin point in world
    posO.plus_assign(jointpos);
    noalias(posP) =  prod(jointrot,attLocalVector);//second point in world
    posP.plus_assign(posO);
    vecOP = posP - posO;
    vecOT = attWorldTarget - posO;
    ChppGikTools::CrossProduct(vecOT,vecOP,attValue);
}


void ChppGikPointingConstraint::computeJacobian()
{
    tempFixedJoint = &(attRobot->fixedJoint(0));
    if (!tempFixedJoint)
    {
        std::cout << "ChppGikPointingConstraint::computeJacobian() expected a fixed joint on the robot.\n";
        return;
    }
    
    tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
    if (!tempFixedJointJacobian)
    {
        std::cout << "ChppGikPointingConstraint::computeJacobian() could not retrieve partial jacobians.\n";
        return;
    }

    attJoint->getJacobianPointWrtConfig(attLocalOriginVector3,tempJacobian0);
    tempJacobian = subrange(tempJacobian0,0,6,6,attRobot->numberDof()) - subrange(*tempFixedJointJacobian,0,6,6,attRobot->numberDof());

    //build the jacobian of the joint wrt to fixed joint
    ChppGikTools::HtoRT(attJoint->currentTransformation(),jointrot,jointpos);
    noalias(posO) = prod(jointrot,attLocalOrigin);//origin point in world
    posO.plus_assign(jointpos);

    ChppGikTools::HtoT(tempFixedJoint->currentTransformation(),posF);
    vecFO = posO - posF;//origin point in world - ankle joint center in world

    ChppGikTools::equivAsymMat(vecFO,matFO);

    noalias(subrange(tempJacobian,0,3,0,tempNumJoints)) += prod(matFO,subrange(*tempFixedJointJacobian,3,6,6,attRobot->numberDof()));

    //compute the jacobian of this constraint
    noalias(posP) =  prod(jointrot,attLocalVector);//second point in world
    posP.plus_assign(posO);
    vecOP = posP - posO;
    vecOT = attWorldTarget - posO;

    ChppGikTools::equivAsymMat(vecOT,matOT);
    ChppGikTools::equivAsymMat(vecOP,matOP);

    noalias(tempRot) = prod(matOT,matOP);

    //     std::cout << "tempRot\n";
    //     ChppGikTools::printBlasMat( tempRot );

    noalias(attJacobian) = prod(tempRot,subrange(tempJacobian,3,6,0,tempNumJoints));
    noalias(attJacobian) -= prod(matOP,subrange(tempJacobian,0,3,0,tempNumJoints));

    //     std::cout << "attJacobian\n";
    //     ChppGikTools::printBlasMat( attJacobian );
    //     std::cout << "Config of computations\n";
    //     std::cout << attRobot->currentConfiguration() << std::endl ;
}
vectorN& ChppGikPointingConstraint::influencingDofs()
{
    unsigned int i;
    attInfluencingDofs.clear();
    for( i=1; i< attRobot->fixedJoint(0).jointsFromRootToThis().size(); i++)
        attInfluencingDofs(attRobot->fixedJoint(0).jointsFromRootToThis()[i]->rankInConfiguration()) = 1;
    for(i=1; i< attJoint->jointsFromRootToThis().size(); i++)
        attInfluencingDofs(attJoint->jointsFromRootToThis()[i]->rankInConfiguration()) = 1;
    return attInfluencingDofs;
}
const vectorN& ChppGikPointingConstraint::value()
{
    return attValue;
}


const matrixNxP& ChppGikPointingConstraint::jacobian()
{
    return attJacobian;
}

bool ChppGikPointingConstraint::minimumJerkInterpolation(ChppGikMotionConstraint* outMotionConstraint, double inSamplingPeriod, double inTime)
{
    unsigned int nsamples = ChppGikTools::timetoRank(0,inTime,inSamplingPeriod)+1;

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
    double NvecOP = norm_2(vecOP);
    double NvecOT = norm_2(vecOT);
    if ( NvecOP < 1e-2 || NvecOT < 1e-2)
    {
        std::cout << "ChppGikPointingConstraint::minimumJerkInterpolation() bad configuration found for involved points.\n";
        return false;
    }
    //initial state:

    //constraint position
    curpos = (NvecOT/NvecOP)*vecOP + posO;
    lever = curpos - jointpos;
    //constraint velocity
    ChppGikTools::Vector3toUblas(attJoint->jointVelocity().linearVelocity(),curvel);
    ChppGikTools::Vector3toUblas(attJoint->jointVelocity().rotationVelocity(),rotvel);
    ChppGikTools::CrossProduct(rotvel,lever,rotvelCrossLocal);
    curvel += rotvelCrossLocal;

    //     std::cout << "curvel " << curvel << std::endl;

    //constraint acceleration
    ChppGikTools::Vector3toUblas(attJoint->jointAcceleration().linearAcceleration(),curaccel);
    ChppGikTools::Vector3toUblas(attJoint->jointAcceleration().rotationAcceleration(),rotaccel);
    ChppGikTools::CrossProduct(rotaccel,lever,temp);
    curaccel += temp;
    ChppGikTools::CrossProduct(rotvel,rotvelCrossLocal,temp);
    curaccel += temp;

    //     std::cout << "curaccel " << curaccel << std::endl;
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

    //     ChppGikTools::dumpMatrix("PlannedHead.txt", data, 0.0, inSamplingPeriod);


    //filling in motion constraint with interpolation samples
    ChppGikPointingConstraint* tempSC = new ChppGikPointingConstraint(*this);

    for (unsigned int i=0; i<nsamples;i++)
    {
        tempSC->worldTargetU(column(data,i));
        outMotionConstraint->pushbackStateConstraint((CjrlGikPointingConstraint*)tempSC);
    }
    delete tempSC;

    return true;
}

void  ChppGikPointingConstraint::worldTargetU(const vectorN& inPoint)
{
    if (inPoint.size() !=3)
    {
        std::cout << "ChppGikPointingConstraint::worldTargetU() received a vector of incorrect size\n";
        return;
    }
    attWorldTarget = inPoint;
    ChppGikTools::UblastoVector3(attWorldTarget, attWorldTargetVector3);
}

const vectorN& ChppGikPointingConstraint::worldTargetU()
{
    return attWorldTarget;
}



ChppGikPointingConstraint::~ChppGikPointingConstraint()
{}
