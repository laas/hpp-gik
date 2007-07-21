#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikRotationConstraint.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikRotationConstraint::ChppGikRotationConstraint(CjrlDynamicRobot& inRobot, CjrlJoint& inJoint, const matrix3d& inTargetOrientation)
{


    attJoint = &inJoint;
    attRobot = &inRobot;

    tempNumJoints = inRobot.numberDof()-6;

    attTargetOrientationMatrix3 = inTargetOrientation;
    attTargetOrientation.resize(3,3,false);

    ChppGikTools::Matrix3toUblas(attTargetOrientationMatrix3, attTargetOrientation);


    attJacobian.resize(3,tempNumJoints,false);
    attValue.resize(3, false);

    temp3DVec.resize(3,false);
    tempRot.resize(3,3,false);
    tempGapRot.resize(3,3,false);
    attInfluencingDofs = ublas::scalar_vector<double>(inRobot.numberDof(),0);
}

CjrlGikStateConstraint* ChppGikRotationConstraint::clone() const
{
    CjrlGikRotationConstraint* ret = new ChppGikRotationConstraint(*this);
    return ret;
}

unsigned int ChppGikRotationConstraint::dimension() const
{
    return 3;
}


CjrlDynamicRobot& ChppGikRotationConstraint::robot()
{
    return *attRobot;
}

void  ChppGikRotationConstraint::joint(CjrlJoint* inJoint)
{
    attJoint = inJoint;
}

CjrlJoint* ChppGikRotationConstraint::joint()
{
    return attJoint;
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

void  ChppGikRotationConstraint::targetOrientationU(const matrixNxP& inTargetOrientation)
{
    attTargetOrientation = inTargetOrientation;
    ChppGikTools::UblastoMatrix3(attTargetOrientation, attTargetOrientationMatrix3);
}

const matrixNxP& ChppGikRotationConstraint::targetOrientationU()
{
    return attTargetOrientation;
}

void ChppGikRotationConstraint::computeValue()
{
    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);

    noalias(tempGapRot) =  prod(trans(tempRot),attTargetOrientation);

    ChppGikTools::RottoOmega(tempGapRot,temp3DVec);

    noalias(attValue) = prod(tempRot,temp3DVec);
}


void ChppGikRotationConstraint::computeJacobian()
{
    tempFixedJoint = &(attRobot->fixedJoint(0));
    if (!tempFixedJoint)
    {
        std::cout << "ChppGikRotationConstraint::computeJacobian() expected a fixed joint on the robot.\n";
        return;
    }

    attJoint->computeJacobianJointWrtConfig();
    //tempFixedJoint->computeJacobianJointWrtConfig();

    tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
    tempEffectorJointJacobian = &(attJoint->jacobianJointWrtConfig());
    if (!tempFixedJointJacobian || !tempEffectorJointJacobian)
    {
        std::cout << "ChppGikRotationConstraint::computeJacobian() could not retrieve partial jacobians.\n";
        return;
    }

    noalias(attJacobian) = subrange(*tempEffectorJointJacobian,3,6,6,attRobot->numberDof()) - subrange(*tempFixedJointJacobian,3,6,6,attRobot->numberDof());
}



const vectorN& ChppGikRotationConstraint::value()
{
    return attValue;
}

vectorN& ChppGikRotationConstraint::influencingDofs()
{
    unsigned int i;
    attInfluencingDofs.clear();
    for( i=1; i< attRobot->fixedJoint(0).jointsFromRootToThis().size(); i++)
        attInfluencingDofs(attRobot->fixedJoint(0).jointsFromRootToThis()[i]->rankInConfiguration()) = 1;
    for(i=1; i< attJoint->jointsFromRootToThis().size(); i++)
        attInfluencingDofs(attJoint->jointsFromRootToThis()[i]->rankInConfiguration()) = 1;
    return attInfluencingDofs;
}


const matrixNxP& ChppGikRotationConstraint::jacobian()
{
    return attJacobian;
}

bool ChppGikRotationConstraint::minimumJerkInterpolation(ChppGikMotionConstraint* outMotionConstraint, double inSamplingPeriod, double inTime)
{

    unsigned int nsamples = ChppGikTools::timetoRank(0,inTime,inSamplingPeriod)+1;

    vectorN curEuler(3);
    vectorN curEulerVel(3);
    vectorN curEulerAccel(3);
    vectorN targetEuler(3);

    vectorN omega(3);
    vectorN dotOmega(3);
    matrixNxP tmpRot(3,3);

    //constraint position
    ChppGikTools::HtoRT(attJoint->currentTransformation(),tmpRot,temp3DVec);
    ChppGikTools::RottoEulerZYX(tmpRot, curEuler);
    //constraint velocity
    ChppGikTools::Vector3toUblas(attJoint->jointVelocity().rotationVelocity(),omega);
    ChppGikTools::OmegatoEulerZYX(omega,curEulerVel);
    //constraint acceleration
    ChppGikTools::Vector3toUblas(attJoint->jointAcceleration().rotationAcceleration(),dotOmega);
    ChppGikTools::OmegatoEulerZYX(dotOmega,curEulerAccel);
    //target quaternion
    ChppGikTools::RottoEulerZYX(attTargetOrientation, targetEuler);

    std::cout << "curEuler "<< curEuler << "\n";
    std::cout << "curEulerVel "<< curEulerVel << "\n";
    std::cout << "curEulerAccel "<< curEulerAccel << "\n";
    std::cout << "targetEuler "<< targetEuler << "\n";
    
    //motion samples computation
    matrixNxP data(3,nsamples);
    vectorN result(nsamples);
    for (unsigned int i=0; i<3;i++)
    {
        bool contn = ChppGikTools::minJerkCurve( inTime,inSamplingPeriod,curEuler(i),curEulerVel(i),curEulerAccel(i),targetEuler(i),result);
        if (!contn)
            return false;
        row(data,i) = result;
    }
    
    //ChppGikTools::dumpMatrix("PlannedEulerRot.txt", data, 0.0, inSamplingPeriod);
    
    //filling in motion constraint with interpolation samples
    ChppGikRotationConstraint* tempSC = new ChppGikRotationConstraint(*this);

    for (unsigned int i=0; i<nsamples;i++)
    {
        ChppGikTools::EulerZYXtoRot(column(data,i),tmpRot);
        tempSC->targetOrientationU(tmpRot);
        outMotionConstraint->pushbackStateConstraint((CjrlGikRotationConstraint*)tempSC);
    }
    delete tempSC;
    return true;
}
