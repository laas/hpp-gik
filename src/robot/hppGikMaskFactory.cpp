#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "robot/hppGikMaskFactory.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikMaskFactory::ChppGikMaskFactory(CjrlHumanoidDynamicRobot* inRobot)
{
    attRobot = inRobot;
    attNumJoints = attRobot->numberDof()-6;

    //prepare joint masks

    attLegs.resize(attNumJoints,false);
    attLegs.clear();

    attUpperBody.resize(attNumJoints,false);
    attUpperBody.clear();

    attWholeBody.resize(attNumJoints,false);
    attWholeBody.clear();

    attChestAndArms.resize(attNumJoints,false);
    attChestAndArms.clear();

    attChestAndHead.resize(attNumJoints,false);
    attChestAndHead.clear();


    std::vector<CjrlJoint*> root2jointVector = (attRobot->leftFoot())->jointsFromRootToThis();


    for (unsigned int i =1; i< root2jointVector.size();i++)
        attLegs(root2jointVector[i]->rankInConfiguration()-6) = (char)1;



    root2jointVector = attRobot->rightFoot()->jointsFromRootToThis();

    for (unsigned int i =1; i< root2jointVector.size();i++)
        attLegs(root2jointVector[i]->rankInConfiguration()-6) = (char)1;


    root2jointVector = attRobot->leftWrist()->jointsFromRootToThis();

    for (unsigned int i =1; i< root2jointVector.size();i++)
    {
        attChestAndArms(root2jointVector[i]->rankInConfiguration()-6) = (char)1;
        attUpperBody(root2jointVector[i]->rankInConfiguration()-6) = (char)1;
    }


    root2jointVector = attRobot->rightWrist()->jointsFromRootToThis();

    for (unsigned int i =1; i< root2jointVector.size();i++)
    {
        attChestAndArms(root2jointVector[i]->rankInConfiguration()-6) = (char)1;
        attUpperBody(root2jointVector[i]->rankInConfiguration()-6) = (char)1;
    }


    root2jointVector = attRobot->gazeJoint()->jointsFromRootToThis();

    for (unsigned int i =1; i< root2jointVector.size();i++)
    {
        attChestAndHead(root2jointVector[i]->rankInConfiguration()-6) = (char)1;
        attUpperBody(root2jointVector[i]->rankInConfiguration()-6) = (char)1;
    }



    ChppGikTools::combineMasks( attLegs,attUpperBody,attWholeBody);


    attWeightsDouble.resize(attNumJoints,false);
    attWeightsLeftLegSupporting.resize(attNumJoints,false);
    attWeightsRightLegSupporting.resize(attNumJoints,false);


    buildWeightVectors();

}

void ChppGikMaskFactory::buildWeightVectors()
{
    //(WARNING! supposing that waist == root, and legs are simple kinematic chains (i.e not subtrees))
    attWeightsLeftLegSupporting.clear();

    attWeightsRightLegSupporting.clear();


    ublas::vector<double> supportedMassVec(attNumJoints+1);
    supportedMassVec.clear();

    //determin the legs joint vectors
    std::vector<CjrlJoint*> lLeg = attRobot->leftFoot()->jointsFromRootToThis();

    std::vector<CjrlJoint*> rLeg = attRobot->rightFoot()->jointsFromRootToThis();


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
    attWeightsRightLegSupporting = ublas::subrange(supportedMassVec,1,attNumJoints+1);

    double tempTotalMass = attRobot->mass()-attRobot->leftFoot()->linkedBody()->mass();
    for (unsigned int i = 0; i< attWeightsRightLegSupporting.size(); i++)
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
    attWeightsLeftLegSupporting = ublas::subrange(supportedMassVec,1,attNumJoints+1);

    for (unsigned int i = 0; i< attWeightsLeftLegSupporting.size(); i++)
        attWeightsLeftLegSupporting(i) = tempTotalMass/attWeightsLeftLegSupporting(i);

//     //Double support
//     attWeightsDouble = ublas::zero_vector<double>(attNumJoints);
//     for (unsigned int i = 0; i<attNumJoints;i++)
//     {
//         if (attLegs(i) > 0.5)
//             attWeightsDouble(i) = 1;
//         else
//             attWeightsDouble(i) = attWeightsLeftLegSupporting(i);//or right, same
//     }
    attWeightsDouble = attWeightsLeftLegSupporting;
    for( iter = rLeg.begin()+1; iter != rLeg.end(); iter++)
        attWeightsDouble((*iter)->rankInConfiguration()-6) = pow(attWeightsRightLegSupporting((*iter)->rankInConfiguration()-6),4);
    for( iter = lLeg.begin()+1; iter != lLeg.end(); iter++)
        attWeightsDouble((*iter)->rankInConfiguration()-6) = pow(attWeightsLeftLegSupporting((*iter)->rankInConfiguration()-6),4);
    //std::cout << attWeightsDouble << std::endl;
    
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
    
    unsigned int rank = inJoint->rankInConfiguration() -6;    

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
