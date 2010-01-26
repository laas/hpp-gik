#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "robot/hppGikMaskFactory.h"
#include "hppGikTools.h"

using namespace boost::numeric::ublas;

ChppGikMaskFactory::ChppGikMaskFactory(CjrlHumanoidDynamicRobot* inRobot)
{
    attRobot = inRobot;
    attNumJoints = attRobot->numberDof();

    //prepare joint masks

    attLegs.resize(attNumJoints,false);
    attLegs.clear();
    subrange(attLegs,0,6) = scalar_vector<double>(6,1);

    attUpperBody = attLegs;
    attWholeBody = attLegs;
    attChestAndArms = attLegs;
    attChestAndHead= attLegs;
    attLeftArm= attLegs;
    attRightArm= attLegs;

    std::vector<CjrlJoint*> root2jointVector = 
      (attRobot->leftAnkle())->jointsFromRootToThis();

    for (unsigned int i =1; i< root2jointVector.size();i++)
        attLegs(root2jointVector[i]->rankInConfiguration()) = (char)1;

    root2jointVector = attRobot->rightAnkle()->jointsFromRootToThis();

    for (unsigned int i =1; i< root2jointVector.size();i++)
        attLegs(root2jointVector[i]->rankInConfiguration()) = (char)1;


    root2jointVector = attRobot->leftWrist()->jointsFromRootToThis();

    for (unsigned int i =1; i< root2jointVector.size();i++)
    {
        attChestAndArms(root2jointVector[i]->rankInConfiguration()) = (char)1;
        attUpperBody(root2jointVector[i]->rankInConfiguration()) = (char)1;
    }


    root2jointVector = attRobot->rightWrist()->jointsFromRootToThis();

    for (unsigned int i =1; i< root2jointVector.size();i++)
    {
        attChestAndArms(root2jointVector[i]->rankInConfiguration()) = (char)1;
        attUpperBody(root2jointVector[i]->rankInConfiguration()) = (char)1;
    }


    root2jointVector = attRobot->gazeJoint()->jointsFromRootToThis();

    for (unsigned int i =1; i< root2jointVector.size();i++)
    {
        attChestAndHead(root2jointVector[i]->rankInConfiguration()) = (char)1;
        attUpperBody(root2jointVector[i]->rankInConfiguration()) = (char)1;
    }

    ChppGikTools::combineMasks( attLegs,attUpperBody,attWholeBody);


    root2jointVector = attRobot->rightWrist()->jointsFromRootToThis();
    for (unsigned int i =3; i< root2jointVector.size();i++)
        attRightArm(root2jointVector[i]->rankInConfiguration()) = (char)1;

    root2jointVector = attRobot->leftWrist()->jointsFromRootToThis();
    for (unsigned int i =3; i< root2jointVector.size();i++)
        attLeftArm(root2jointVector[i]->rankInConfiguration()) = (char)1;

    attWeightsDouble = attLegs;
    attWeightsLeftLegSupporting = attLegs;
    attWeightsRightLegSupporting = attLegs;
    
    buildWeightVectors();

    attCustomMask.resize(attNumJoints,false);
    attCustomMask.clear();

}

vectorN& ChppGikMaskFactory::leftArmMask()
{
    return attLeftArm;
}

vectorN& ChppGikMaskFactory::customMask(CjrlJoint* inJoint, unsigned int rankOfFirstActivatedJoint)
{
    unsigned int i;
    attCustomMask.clear();
    subrange(attCustomMask,0,6)=scalar_vector<double>(6,1);
    if (inJoint)
    {
        std::vector<CjrlJoint*> root2jointVector = inJoint->jointsFromRootToThis();

        for ( i= rankOfFirstActivatedJoint; i< root2jointVector.size();i++)
            attCustomMask(root2jointVector[i]->rankInConfiguration()) = (char)1;
    }
    return attCustomMask;
}

vectorN& ChppGikMaskFactory::rightArmMask()
{
    return attRightArm;
}

void ChppGikMaskFactory::buildWeightVectors()
{
    //(WARNING! supposing that waist == root, and legs are simple kinematic chains (i.e not subtrees))

    vectorN supportedMassVec(attNumJoints-6+1);
    supportedMassVec.clear();

    //determin the legs joint vectors
    std::vector<CjrlJoint*> lLeg = 
      attRobot->leftAnkle()->jointsFromRootToThis();

    std::vector<CjrlJoint*> rLeg = 
      attRobot->rightAnkle()->jointsFromRootToThis();


    //right leg supporting
    CjrlJoint* inJoint = *(rLeg.begin()+1);
    supportedMass(supportedMassVec, attRobot->rootJoint(), inJoint);

    supportedMassVec(0) -= attRobot->rootJoint()->linkedBody()->mass();
    unsigned int rankJoint, rankParent;
    std::vector<CjrlJoint*>::iterator iter;

    for( iter = rLeg.begin()+1; iter != rLeg.end(); iter++)
    {
        rankJoint = rankInSupportedMassVector(*iter);
        rankParent = rankInSupportedMassVector(*(iter-1));
        supportedMassVec(rankJoint) += (*(iter-1))->linkedBody()->mass() + supportedMassVec(rankParent);
    }
    subrange(attWeightsRightLegSupporting,6,attNumJoints) = subrange(supportedMassVec,1,attNumJoints-6+1);

    double tempTotalMass = attRobot->mass()-attRobot->leftAnkle()->
      linkedBody()->mass();
    for (unsigned int i = 6; i< attWeightsRightLegSupporting.size(); i++)
        attWeightsRightLegSupporting(i) = tempTotalMass/attWeightsRightLegSupporting(i);

    supportedMassVec.clear();

    //left leg supporting
    supportedMass(supportedMassVec, attRobot->rootJoint(), *(lLeg.begin()+1));
    supportedMassVec(0) -= attRobot->rootJoint()->linkedBody()->mass();
    for( iter = lLeg.begin()+1; iter != lLeg.end(); iter++)
    {
        rankJoint = rankInSupportedMassVector(*iter);
        rankParent = rankInSupportedMassVector(*(iter-1));
        supportedMassVec(rankJoint) += (*(iter-1))->linkedBody()->mass() + supportedMassVec(rankParent);
    }
    
    subrange(attWeightsLeftLegSupporting,6,attNumJoints) = subrange(supportedMassVec,1,attNumJoints-6+1);

    for (unsigned int i = 6; i< attWeightsLeftLegSupporting.size(); i++)
        attWeightsLeftLegSupporting(i) = tempTotalMass/attWeightsLeftLegSupporting(i);

    //     //Double support
    //     attWeightsDouble = zero_vector<double>(attNumJoints);
    //     for (unsigned int i = 0; i<attNumJoints;i++)
    //     {
    //         if (attLegs(i) > 0.5)
    //             attWeightsDouble(i) = 1;
    //         else
    //             attWeightsDouble(i) = attWeightsLeftLegSupporting(i);//or right, same
    //     }
    attWeightsDouble = attWeightsLeftLegSupporting;
    for( iter = rLeg.begin()+1; iter != rLeg.end(); iter++)
        attWeightsDouble((*iter)->rankInConfiguration()) = pow(attWeightsRightLegSupporting((*iter)->rankInConfiguration()),4);
    for( iter = lLeg.begin()+1; iter != lLeg.end(); iter++)
        attWeightsDouble((*iter)->rankInConfiguration()) = pow(attWeightsLeftLegSupporting((*iter)->rankInConfiguration()),4);
}

void ChppGikMaskFactory::supportedMass(vectorN& massVec, const CjrlJoint* inJoint, CjrlJoint* excludedChild)
{
    unsigned int rankJoint, rankChildJoint;
    rankJoint = rankInSupportedMassVector(inJoint);
    //std::cout << "rankjoint      "<<rankJoint <<"\n";
    massVec(rankJoint) += inJoint->linkedBody()->mass();

    for (unsigned int i = 0; i<inJoint->countChildJoints(); i++)
    {
        const CjrlJoint* childJoint = inJoint->childJoint(i);
        rankChildJoint = rankInSupportedMassVector(childJoint);
        //std::cout << "rankchildjoint "<< rankChildJoint <<"\n";

        if (childJoint == excludedChild)
        {
            //std::cout << "excluded \n";
            continue;
        }

        supportedMass(massVec, childJoint, 0);
        massVec(rankJoint) += massVec(rankChildJoint);
    }
}

unsigned int ChppGikMaskFactory::rankInSupportedMassVector(const CjrlJoint* inJoint)
{
    if (inJoint == attRobot->rootJoint())
        return 0;
    else
        return inJoint->rankInConfiguration()-5;
}

bool ChppGikMaskFactory::containsJoint(CjrlJoint* inJoint,std::vector<CjrlJoint*>& inVec)
{
    std::vector<CjrlJoint*>::iterator iter;
    for( iter = inVec.begin(); iter != inVec.end(); iter++)
        if ((*iter) == inJoint)
            return true;
    return false;
}

vectorN& ChppGikMaskFactory::maskForJoint(CjrlJoint* inJoint)
{
    if (inJoint == attRobot->rootJoint())
        return attLegs;

    unsigned int rank = inJoint->rankInConfiguration();

    if (attLegs(rank) == 1)
        return attLegs;

    if (attChestAndHead(rank) == 1)
        return attChestAndHead;
    else
        return attChestAndArms;

}


bool ChppGikMaskFactory::activateJointInMask(CjrlJoint* inJoint, vectorN& inoutMask)
{
    if (inoutMask.size() != attNumJoints)
    {
        std::cout <<"ChppGikMaskFactory::activateJointInMask() input mask incorrectly sized\n";
        return false;
    }

    ChppGikTools::combineMasks(maskForJoint(inJoint), inoutMask, inoutMask);
    return true;
}

vectorN& ChppGikMaskFactory::legsMask()
{
    return attLegs;
}

vectorN& ChppGikMaskFactory::upperBodyMask()
{
    return attUpperBody;
}

vectorN& ChppGikMaskFactory::truncMask()
{
    return attChestAndHead;
}

vectorN& ChppGikMaskFactory::wholeBodyMask()
{
    return attWholeBody;
}

vectorN& ChppGikMaskFactory::weightsDoubleSupport()
{
    return attWeightsDouble;
}

vectorN& ChppGikMaskFactory::weightsLeftLegSupporting()
{
    return attWeightsLeftLegSupporting;
}

vectorN& ChppGikMaskFactory::weightsRightLegSupporting()
{
    return attWeightsRightLegSupporting;
}


ChppGikMaskFactory::~ChppGikMaskFactory()
{}
