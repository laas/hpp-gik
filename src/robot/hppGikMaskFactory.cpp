#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "robot/hppGikMaskFactory.h"
#include "hppGikTools.h"

using namespace boost::numeric::ublas;

ChppGikMaskFactory::ChppGikMaskFactory ( CjrlHumanoidDynamicRobot* inRobot )
{
    attRobot = inRobot;
    attNumJoints = attRobot->numberDof();

    //prepare joint masks

    attLegs.resize ( attNumJoints,false );
    attLegs.clear();
    subrange ( attLegs,0,6 ) = scalar_vector<double> ( 6,1 );

    attUpperBody = attLegs;
    attWholeBody = attLegs;
    attRightArm = attLegs;
    attLeftArm = attLegs;


    std::vector<CjrlJoint*> gaze2waist = attRobot->jointsBetween ( *attRobot->gazeJoint(),*attRobot->waist() );

    std::vector<CjrlJoint*> joints = attRobot->jointsBetween ( *attRobot->rightAnkle(),*attRobot->leftAnkle() );
    for ( unsigned int i =0; i< joints.size();i++ )
        if ( joints[i] != attRobot->rootJoint() )
            attLegs ( joints[i]->rankInConfiguration() ) = ( char ) 1;

    joints = attRobot->jointsBetween ( *attRobot->gazeJoint(),*attRobot->leftWrist() );
    for ( unsigned int i =0; i< joints.size();i++ )
        if ( joints[i] != attRobot->rootJoint() )
            attLeftArm ( joints[i]->rankInConfiguration() ) = ( char ) 1;
    for ( unsigned int i =0; i< gaze2waist.size();i++ )
        if ( gaze2waist[i] != attRobot->rootJoint() )
            attLeftArm ( gaze2waist[i]->rankInConfiguration() ) = ( char ) 0;

    joints = attRobot->jointsBetween ( *attRobot->gazeJoint(),*attRobot->rightWrist() );
    for ( unsigned int i =0; i< joints.size();i++ )
        if ( joints[i] != attRobot->rootJoint() )
            attRightArm ( joints[i]->rankInConfiguration() ) = ( char ) 1;
    for ( unsigned int i =0; i< gaze2waist.size();i++ )
        if ( gaze2waist[i] != attRobot->rootJoint() )
            attRightArm ( gaze2waist[i]->rankInConfiguration() ) = ( char ) 0;

    joints = attRobot->jointsBetween ( *attRobot->leftWrist(),*attRobot->rightWrist() );
    for ( unsigned int i =0; i< joints.size();i++ )
        if ( joints[i] != attRobot->rootJoint() )
            attUpperBody ( joints[i]->rankInConfiguration() ) = ( char ) 1;
    for ( unsigned int i =0; i< gaze2waist.size();i++ )
        if ( gaze2waist[i] != attRobot->rootJoint() )
            attUpperBody ( gaze2waist[i]->rankInConfiguration() ) = ( char ) 1;
/*
    std::cout << attLegs << std::endl;
    std::cout << attLeftArm << std::endl;
    std::cout << attRightArm << std::endl;
    std::cout << attUpperBody << std::endl;
*/

    ChppGikTools::combineMasks ( attLegs,attUpperBody,attWholeBody );

    attMaskForJoint.resize(attRobot->numberDof());
    attWeightsDouble = attLegs;
    attWeightsLeftLegSupporting = attLegs;
    attWeightsRightLegSupporting = attLegs;

    algoWeights ( attRobot->leftAnkle(), attWeightsLeftLegSupporting );
    algoWeights ( attRobot->rightAnkle(), attWeightsRightLegSupporting );

    attWeightsDouble = attWeightsLeftLegSupporting;
    for (unsigned int i = 6; i<attRobot->numberDof();i++)
        if (attLegs(i) == 1)
            attWeightsDouble(i) = 1;
    
    attCustomMask.resize ( attNumJoints,false );
    attCustomMask.clear();
    /*
    std::cout << attWeightsDouble << std::endl;*/
}

const vectorN& ChppGikMaskFactory::leftArmMask()
{
    return attLeftArm;
}

const vectorN& ChppGikMaskFactory::rightArmMask()
{
    return attRightArm;
}

const vectorN& ChppGikMaskFactory::customMask ( CjrlJoint* inJoint, unsigned int rankOfFirstActivatedJoint )
{
    unsigned int i;
    attCustomMask.clear();
    subrange ( attCustomMask,0,6 ) =scalar_vector<double> ( 6,1 );
    if ( inJoint )
    {
        std::vector<CjrlJoint*> root2jointVector = inJoint->jointsFromRootToThis();

        for ( i= rankOfFirstActivatedJoint; i< root2jointVector.size();i++ )
            attCustomMask ( root2jointVector[i]->rankInConfiguration() ) = ( char ) 1;
    }
    return attCustomMask;
}


void ChppGikMaskFactory::algoWeights ( CjrlJoint* supportJoint, vectorN& outWeights )
{
    vectorN supportedMassVec ( attNumJoints-6+1 );
    supportedMassVec.clear();

    std::vector<CjrlJoint*> backbone = attRobot->jointsBetween ( *supportJoint,  *attRobot->rootJoint() );
    if ( backbone.size() >1 )
    {
        supportedMass ( supportedMassVec, attRobot->rootJoint(), backbone[backbone.size()-2] );
        //std::cout << supportedMassVec << std::endl;
        supportedMassVec ( rankInSupportedMassVector ( attRobot->rootJoint() ) ) -= attRobot->rootJoint()->linkedBody()->mass();//remove root mass from backbone
        //std::cout << supportedMassVec << std::endl;
        for ( unsigned int i = 1; i< backbone.size()-1;i++ )
            if ( backbone[i]->countChildJoints() > 1 )
            {
                //Sum subtree for this node except for branch belonging to backbone
                supportedMass ( supportedMassVec, backbone[i], backbone[i-1] );
                //remove node mass
                supportedMassVec ( rankInSupportedMassVector ( backbone[i] ) ) -= backbone[i]->linkedBody()->mass();
                //forward subtree mass to child in backbone
                supportedMassVec (rankInSupportedMassVector ( backbone[i-1] ) )= supportedMassVec (rankInSupportedMassVector ( backbone[i] ));
                //cancel this node supported mass
                supportedMassVec (rankInSupportedMassVector ( backbone[i] ) )= 0;
          //      std::cout << supportedMassVec << std::endl;
            }
        for ( int i = backbone.size()-2; i>-1;i-- )
        {
            supportedMassVec ( rankInSupportedMassVector ( backbone[i] ) ) += supportedMassVec(rankInSupportedMassVector ( backbone[i+1] )) + backbone[i+1]->linkedBody()->mass();//subtrees and backbone links masses here ok
            //std::cout << supportedMassVec << std::endl;
        }
    }
    else
    {
        supportedMass ( supportedMassVec, attRobot->rootJoint(), 0 );
    }
    //remove link mass from backbone
    supportedMassVec ( rankInSupportedMassVector ( backbone[0] ) ) -= backbone[0]->linkedBody()->mass();
    subrange ( outWeights,6,attNumJoints ) = subrange ( supportedMassVec,1,attNumJoints-6+1 );
    for (unsigned int i = 6; i< outWeights.size(); i++)
    {
        if (outWeights(i)!=0)
            outWeights(i) = attRobot->mass()/outWeights(i);
    }
    
    
    //std::cout << "                "<<outWeights << std::endl;
}

void ChppGikMaskFactory::supportedMass ( vectorN& massVec, const CjrlJoint* inJoint, CjrlJoint* excludedChild )
{
    unsigned int rankJoint, rankChildJoint;
    rankJoint = rankInSupportedMassVector ( inJoint );
    //std::cout << "rankjoint      "<<rankJoint <<"\n";
    massVec ( rankJoint ) += inJoint->linkedBody()->mass();

    for ( unsigned int i = 0; i<inJoint->countChildJoints(); i++ )
    {
        const CjrlJoint* childJoint = inJoint->childJoint ( i );
        rankChildJoint = rankInSupportedMassVector ( childJoint );
        //std::cout << "rankchildjoint "<< rankChildJoint <<"\n";

        if ( childJoint == excludedChild )
        {
            //std::cout << "excluded \n";
            continue;
        }

        supportedMass ( massVec, childJoint, 0 );
        massVec ( rankJoint ) += massVec ( rankChildJoint );
    }
}

unsigned int ChppGikMaskFactory::rankInSupportedMassVector ( const CjrlJoint* inJoint )
{
    if ( inJoint == attRobot->rootJoint() )
        return 0;
    else
        return inJoint->rankInConfiguration()-5;
}

bool ChppGikMaskFactory::containsJoint ( CjrlJoint* inJoint,std::vector<CjrlJoint*>& inVec )
{
    std::vector<CjrlJoint*>::iterator iter;
    for ( iter = inVec.begin(); iter != inVec.end(); iter++ )
        if ( ( *iter ) == inJoint )
            return true;
    return false;
}

const vectorN& ChppGikMaskFactory::maskForJointsBetween(CjrlJoint* inStartJoint,CjrlJoint* inEndJoint)
{
    attMaskForJoint.clear();
    std::vector<CjrlJoint*> vecJoints = attRobot->jointsBetween(*inStartJoint, *inEndJoint);
    unsigned int i,j;
    for (i=0;i<vecJoints.size();i++)
        for (j=vecJoints[i]->rankInConfiguration();j<vecJoints[i]->rankInConfiguration()+vecJoints[i]->numberDof();j++)
            attMaskForJoint(j) = 1;

    return attMaskForJoint;
    //return attWholeBody;
}

const vectorN& ChppGikMaskFactory::legsMask()
{
    return attLegs;
}

const vectorN& ChppGikMaskFactory::upperBodyMask()
{
    return attUpperBody;
}

const vectorN& ChppGikMaskFactory::wholeBodyMask()
{
    return attWholeBody;
}

const vectorN& ChppGikMaskFactory::weightsDoubleSupport()
{
    return attWeightsDouble;
}

const vectorN& ChppGikMaskFactory::weightsLeftLegSupporting()
{
    return attWeightsLeftLegSupporting;
}

const vectorN& ChppGikMaskFactory::weightsRightLegSupporting()
{
    return attWeightsRightLegSupporting;
}


ChppGikMaskFactory::~ChppGikMaskFactory()
{}

