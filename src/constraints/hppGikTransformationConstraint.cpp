#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikTransformationConstraint.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikTransformationConstraint::ChppGikTransformationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame, const vector3d& inPointInWorldFrame, const matrix3d& inTargetOrientation)
{
    attJoint = &inJoint;
    attRobot = &inRobot;

    tempNumJoints = inRobot.numberDof()-6;

    attLocalPointVector3 =  inPointInBodyLocalFrame;
    attWorldTargetVector3 =  inPointInWorldFrame;
    attTargetOrientationMatrix3 = inTargetOrientation;

    attLocalPoint.resize(3,false);
    attWorldTarget.resize(3,false);
    attTargetOrientation.resize(3,3,false);
    ChppGikTools::Vector3toUblas(attLocalPointVector3, attLocalPoint);
    ChppGikTools::Vector3toUblas(attWorldTargetVector3, attWorldTarget);
    ChppGikTools::Matrix3toUblas(attTargetOrientationMatrix3, attTargetOrientation);


    attJacobian.resize(6,tempNumJoints,false);
    tempJacobian.resize(6,inRobot.numberDof(),false);
    attValue.resize(6, false);
    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);
    tempGapRot.resize(3,3,false);
    attInfluencingDofs.resize(inRobot.numberDof(),false);
}

ChppGikTransformationConstraint::ChppGikTransformationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inPointInBodyLocalFrame,  const matrix4d& inTransformation)
{
    attJoint = &inJoint;
    attRobot = &inRobot;

    tempNumJoints = inRobot.numberDof()-6;

    attLocalPointVector3 =  inPointInBodyLocalFrame;

    attLocalPoint.resize(3,false);
    attWorldTarget.resize(3,false);
    attTargetOrientation.resize(3,3,false);

    ChppGikTools::Vector3toUblas(attLocalPointVector3, attLocalPoint);
    ChppGikTools::HtoRT(inTransformation,attTargetOrientation,attWorldTarget);
    ChppGikTools::UblastoVector3(attWorldTarget, attWorldTargetVector3);
    ChppGikTools::UblastoMatrix3(attTargetOrientation, attTargetOrientationMatrix3);

    attJacobian.resize(6,tempNumJoints,false);
    attValue.resize(6, false);


    tempRot.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);
    tempGapRot.resize(3,3,false);
    attInfluencingDofs.resize(inRobot.numberDof(),false);

}

CjrlGikStateConstraint* ChppGikTransformationConstraint::clone() const
{
    CjrlGikTransformationConstraint* ret = new ChppGikTransformationConstraint(*this);
    return ret;
}

unsigned int ChppGikTransformationConstraint::dimension() const
{
    return 6;
}


CjrlDynamicRobot& ChppGikTransformationConstraint::robot()
{
    return *attRobot;
}

void  ChppGikTransformationConstraint::joint(CjrlJoint* inJoint)
{
    attJoint = inJoint;
}

CjrlJoint* ChppGikTransformationConstraint::joint()
{
    return attJoint;
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


vectorN& ChppGikTransformationConstraint::influencingDofs()
{
    unsigned int i;
    attInfluencingDofs.clear();

    for( i=1; i< attRobot->fixedJoint(0).jointsFromRootToThis().size(); i++)
        attInfluencingDofs(attRobot->fixedJoint(0).jointsFromRootToThis()[i]->rankInConfiguration()) = 1;
    for(i=1; i< attJoint->jointsFromRootToThis().size(); i++)
        attInfluencingDofs(attJoint->jointsFromRootToThis()[i]->rankInConfiguration()) = 1;

    return attInfluencingDofs;
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

    noalias(subrange(attJacobian,0,3,0,tempNumJoints)) += prod(tempRot,subrange(*tempFixedJointJacobian,3,6,6,attRobot->numberDof()));


    //     std::cout << "attJacobian\n";
    //     ChppGikTools::printBlasMat( attJacobian );
    //     std::cout << "Config of computations\n";
    //     std::cout << attRobot->currentConfiguration() << std::endl ;
}



const vectorN& ChppGikTransformationConstraint::value()
{
    return attValue;
}

const matrixNxP& ChppGikTransformationConstraint::jacobian()
{
    return attJacobian;
}

bool ChppGikTransformationConstraint::minimumJerkInterpolation(ChppGikMotionConstraint* outMotionConstraint, double inSamplingPeriod, double inTime)
{
    unsigned int nsamples = ChppGikTools::timetoRank(0,inTime,inSamplingPeriod)+1;

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
    //target quaternion
    ChppGikTools::RottoEulerZYX(attTargetOrientation, targetEuler);

    //motion samples computation
    matrixNxP dataPos(3,nsamples);
    vectorN result(nsamples);
    for (unsigned int i=0; i<3;i++)
    {
        bool contn = ChppGikTools::minJerkCurve( inTime,inSamplingPeriod,curpos(i),curvel(i),curaccel(i),attWorldTarget(i),result);
        row(dataPos,i) = result;

        if (!contn)
            return false;
    }

    //ChppGikTools::dumpMatrix("Plannedpos.txt", dataPos, 0.0, inSamplingPeriod);

//     std::cout << "curpos "<< curpos << "\n";
//     std::cout << "curvel "<< curvel << "\n";
//     std::cout << "curaccel "<< curaccel << "\n";
//     
//     std::cout << "curEuler "<< curEuler << "\n";
//     std::cout << "curEulerVel "<< curEulerVel << "\n";
//     std::cout << "curEulerAccel "<< curEulerAccel << "\n";
//     std::cout << "targetEuler "<< targetEuler << "\n";

    matrixNxP dataEuler(3,nsamples);
    for (unsigned int i=0; i<3;i++)
    {
        bool contn = ChppGikTools::minJerkCurve( inTime,inSamplingPeriod,curEuler(i),curEulerVel(i),curEulerAccel(i),targetEuler(i),result);
        if (!contn)
            return false;
        row(dataEuler,i) = result;

    }

    //ChppGikTools::dumpMatrix("PlannedEuler.txt", dataEuler, 0.0, inSamplingPeriod);

    //filling in motion constraint with interpolation samples
    ChppGikTransformationConstraint* tempSC = new ChppGikTransformationConstraint(*this);

    for (unsigned int i=0; i<nsamples;i++)
    {
        tempSC->worldTargetU(column(dataPos,i));
        ChppGikTools::EulerZYXtoRot(column(dataEuler,i),tempRot);
        tempSC->targetOrientationU(tempRot);
        outMotionConstraint->pushbackStateConstraint((CjrlGikTransformationConstraint*)tempSC);
    }
    delete tempSC;
    return true;
}
