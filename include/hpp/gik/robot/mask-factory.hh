#ifndef HPP_GIK_MASK_FACTORY_H
#define HPP_GIK_MASK_FACTORY_H

#include <vector>
#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"
#include "abstract-robot-dynamics/humanoid-dynamic-robot.hh"


/**
\brief Build some joint masks and weighting vectors for the GIK solver. Some returned data are hardcoded for hrp2 robot.
\ingroup robot
 */
class ChppGikMaskFactory
{
public:
    /**
    \brief Constructor
    */
    ChppGikMaskFactory(CjrlHumanoidDynamicRobot* inRobot);

    /**
    \brief Get the joint mask for the given joint
     */
    const vectorN& maskForJointsBetween(CjrlJoint* inStartJoint,CjrlJoint* inEndJoint);
    /**
    \brief Get the joint mask for the legs
     */
    const vectorN& legsMask();
    /**
    \brief Get the joint mask for the upper body
    */
    const vectorN& upperBodyMask();
    /**
    \brief Get the joint mask for the left arm
    */
    const vectorN& leftArmMask();
    /**
    \brief Get the joint mask for the right arm
     */
    const vectorN& rightArmMask();
    /**
    \brief Get the joint mask for joints betwwen root and given joint, starting from joint with rank rankOfFirstActivatedJoint
     */
    const vectorN& customMask(CjrlJoint* inJoint, unsigned int rankOfFirstActivatedJoint);
    /**
    \brief Get the joint mask for the whole body
     */
    const vectorN& wholeBodyMask();
    /**
    \brief Get the contribution weights of joints for the double support case
     */
    const vectorN& weightsDoubleSupport();
    /**
    \brief Get the contribution weights when only the left leg is supporting
     */
    const vectorN& weightsLeftLegSupporting();
    /**
    \brief Get the contribution weights when only the right leg is supporting
     */
    const vectorN& weightsRightLegSupporting();
    
    /**
    \brief Destructor
     */
    ~ChppGikMaskFactory();

private:
            
    bool containsJoint(CjrlJoint* inJoint,std::vector<CjrlJoint*>& inVec);
    
    unsigned int rankInSupportedMassVector(const CjrlJoint* inJoint);
    
    void algoWeights(CjrlJoint* supportJoint, vectorN& outWeights );
    
    void supportedMass(vectorN& massVec, const CjrlJoint* inJoint, CjrlJoint* excludedChild);

    CjrlHumanoidDynamicRobot* attRobot;
    
    unsigned int attNumJoints;
    vectorN attUpperBody;
    vectorN attLegs;
    vectorN attMaskForJoint;
    vectorN attLeftArm;
    vectorN attRightArm;
    vectorN attWholeBody;
    vectorN attWeightsDouble;
    vectorN attCustomMask;
    vectorN attWeightsLeftLegSupporting;
    vectorN attWeightsRightLegSupporting;

};

#endif
