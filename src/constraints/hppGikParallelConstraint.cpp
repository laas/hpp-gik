#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikParallelConstraint.h"
#include "hppGikTools.h"


ChppGikParallelConstraint::ChppGikParallelConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalVector, const vector3d& inTargetVector)
{


    attJoint = &inJoint;
    attRobot = &inRobot;

    tempNumJoints = inRobot.numberDof()-6;

    attLocalVectorVector3 =  inLocalVector;
    attTargetVectorVector3 =  inTargetVector;

    attLocalVector.resize(3,false);
    attTargetVector.resize(3,false);

    ChppGikTools::Vector3toUblas(attLocalVectorVector3, attLocalVector);
    ChppGikTools::Vector3toUblas(attTargetVectorVector3, attTargetVector);

    attLocalVector =  attLocalVector/ublas::norm_2(attLocalVector);
    attTargetVector =  attTargetVector/ublas::norm_2(attTargetVector);


    tempJointOrientJacobian.resize(3,tempNumJoints,false);
    attJacobian.resize(3,tempNumJoints,false);
    attValue.resize(3, false);

    tempRot.resize(3,3,false);
    tempRot1.resize(3,3,false);
    tempRot2.resize(3,3,false);
    tempRot3.resize(3,3,false);
    temp3DVec.resize(3,false);
    temp3DVec1.resize(3,false);

    attInfluencingDofs = ublas::scalar_vector<double>(inRobot.numberDof(),0);
}

CjrlGikStateConstraint* ChppGikParallelConstraint::clone() const
{
    CjrlGikParallelConstraint* ret =  new ChppGikParallelConstraint(*this);
    return ret;
}

unsigned int ChppGikParallelConstraint::dimension() const
{
    return 3;// yeah, well, this is convenient although the task is 2D
}


CjrlHumanoidDynamicRobot& ChppGikParallelConstraint::robot()
{
    return *attRobot;
}

void  ChppGikParallelConstraint::joint(CjrlJoint* inJoint)
{
    attJoint = inJoint;
}

CjrlJoint* ChppGikParallelConstraint::joint()
{
    return attJoint;
}

void  ChppGikParallelConstraint::localVector(const vector3d& inVector)
{
    attLocalVectorVector3 =  inVector;
    ChppGikTools::Vector3toUblas(attLocalVectorVector3, attLocalVector);
    attLocalVector =  attLocalVector/ublas::norm_2(attLocalVector);
}

const vector3d& ChppGikParallelConstraint::localVector()
{
    return attLocalVectorVector3;
}

void  ChppGikParallelConstraint::targetVector (const vector3d& inVector)
{
    attTargetVectorVector3 =  inVector;
    ChppGikTools::Vector3toUblas(attTargetVectorVector3, attTargetVector);
    attTargetVector =  attTargetVector/ublas::norm_2(attTargetVector);
}

const vector3d& ChppGikParallelConstraint::targetVector()
{
    return attTargetVectorVector3;
}


void  ChppGikParallelConstraint::targetVectorU(const vectorN& inVector)
{
    attTargetVector =  inVector;
    ChppGikTools::UblastoVector3(attTargetVector, attTargetVectorVector3);
    attTargetVector =  attTargetVector/ublas::norm_2(attTargetVector);
}

const vectorN& ChppGikParallelConstraint::targetVectorU()
{
    return attTargetVector;
}


void ChppGikParallelConstraint::computeValue()
{

    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);
    //std::cout << "local vector "<< attLocalVector << "\n";
    temp3DVec =  ublas::prod(tempRot,attLocalVector);
    //std::cout << "current world vector "<< temp3DVec << "\n";
    ChppGikTools::CrossProduct(attTargetVector,temp3DVec,attValue);
    //std::cout << "attValue "<< attValue << "\n";
}

vectorN& ChppGikParallelConstraint::influencingDofs()
{
    unsigned int i;
    attInfluencingDofs.clear();
    for( i=1; i< attRobot->fixedJoint(0).jointsFromRootToThis().size(); i++)
        attInfluencingDofs(attRobot->fixedJoint(0).jointsFromRootToThis()[i]->rankInConfiguration()) = 1;
    for(i=1; i< attJoint->jointsFromRootToThis().size(); i++)
        attInfluencingDofs(attJoint->jointsFromRootToThis()[i]->rankInConfiguration()) = 1;
    return attInfluencingDofs;
}


void ChppGikParallelConstraint::computeJacobian()
{
    tempFixedJoint = &(attRobot->fixedJoint(0));
    if (!tempFixedJoint)
    {
        std::cout << "ChppGikParallelConstraint::computeJacobian() expected a fixed joint on the robot.\n";
        return;
    }

    attJoint->computeJacobianJointWrtConfig();
    //tempFixedJoint->computeJacobianJointWrtConfig();

    tempFixedJointJacobian = &(tempFixedJoint->jacobianJointWrtConfig());
    tempEffectorJointJacobian = &(attJoint->jacobianJointWrtConfig());
    if (!tempFixedJointJacobian || !tempEffectorJointJacobian)
    {
        std::cout << "ChppGikParallelConstraint::computeJacobian() could not retrieve partial jacobians.\n";
        return;
    }

    ublas::noalias(tempJointOrientJacobian) = ublas::subrange(*tempEffectorJointJacobian,3,6,6,attRobot->numberDof()) - ublas::subrange(*tempFixedJointJacobian,3,6,6,attRobot->numberDof());

    //     std::cout << "tempJointOrientJacobian\n";
    //     ChppGikTools::printBlasMat( tempJointOrientJacobian );

    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);
    temp3DVec =  ublas::prod(tempRot,attLocalVector);

    ChppGikTools::equivAsymMat(attTargetVector,tempRot1);
    ChppGikTools::equivAsymMat(temp3DVec,tempRot2);

    //     std::cout << "attTargetVector "<< attTargetVector << "\n";
    //     std::cout << "temp3DVec "<< temp3DVec << "\n";

    ublas::noalias(tempRot3) = ublas::prod(tempRot1,tempRot2);
    ublas::noalias(attJacobian) = ublas::prod(tempRot3,tempJointOrientJacobian);
}



const vectorN& ChppGikParallelConstraint::value()
{
    return attValue;
}


const matrixNxP& ChppGikParallelConstraint::jacobian()
{
    return attJacobian;
}

bool ChppGikParallelConstraint::minimumJerkInterpolation(ChppGikMotionConstraint* outMotionConstraint, double inSamplingPeriod, double inTime)
{
    unsigned int nsamples = ChppGikTools::timetoRank(0,inTime,inSamplingPeriod)+1;

    vectorN curpos(3);
    vectorN curvel(3);
    vectorN curaccel(3);

    //matrixNxP tempMat4(4,4);
    vectorN rotvel(3);
    vectorN rotvelCrossLocal(3);
    vectorN rotaccel(3);
    vectorN temp(3);



    //To DO
    //correct a known bug in this hurriedly coded method: determin the nearest target vector (the given one or its opposite, because a parallel constraint does not care about orientation like a pointing constraint)

    //constraint position
    ChppGikTools::HtoRT(attJoint->currentTransformation(),tempRot,temp3DVec);
    curpos =  ublas::prod(tempRot,attLocalVector);
    
    //std::cout << "curpos " << curpos << std::endl;

    //constraint velocity
    ChppGikTools::Vector3toUblas(attJoint->jointVelocity().rotationVelocity(),rotvel);
    ChppGikTools::CrossProduct(rotvel,curpos,curvel);

    //std::cout << "curvel " << curvel << std::endl;

    //constraint acceleration
    curaccel.clear();
    ChppGikTools::Vector3toUblas(attJoint->jointAcceleration().rotationAcceleration(),rotaccel);
    ChppGikTools::CrossProduct(rotaccel,curpos,curaccel);
    ChppGikTools::CrossProduct(rotvel,curvel,temp);
    curaccel.plus_assign(temp);

    //std::cout << "curaccel " << curaccel << std::endl;
    
    //motion samples computation
    matrixNxP data(3,nsamples);
    vectorN result(nsamples);
    for (unsigned int i=0; i<3;i++)
    {
        bool contn = ChppGikTools::minJerkCurve( inTime,inSamplingPeriod,curpos(i),curvel(i),curaccel(i),attTargetVector(i),result);
        if (!contn)
            return false;
        row(data,i) = result;

    }

    //filling in motion constraint with interpolation samples
    ChppGikParallelConstraint* tempSC = new ChppGikParallelConstraint(*this);

    for (unsigned int i=0; i<nsamples;i++)
    {
        tempSC->targetVectorU(column(data,i));
        outMotionConstraint->pushbackStateConstraint((CjrlGikParallelConstraint*)tempSC);
    }
    delete tempSC;

    return true;
}
