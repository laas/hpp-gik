#ifndef HPP_GIK_MASK_FACTORY_H
#define HPP_GIK_MASK_FACTORY_H

#include <vector>
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlHumanoidDynamicRobot.h"


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
    vectorN& maskForJoint(CjrlJoint* inJoint);
    /**
    \brief Activate a joint in the given mask
    \return false in inoutMask incorrectly sized
     */
    bool activateJointInMask(CjrlJoint* inJoint, vectorN& inoutMask);
    /**
    \brief Get the joint mask for the legs
     */
    vectorN& legsMask();
    /**
    \brief Get the joint mask for the upper body
    */
    vectorN& upperBodyMask();
    /**
    \brief Get the joint mask for the left arm
    */
    vectorN& leftArmMask();
    /**
    \brief Get the joint mask for the right arm
     */
    vectorN& rightArmMask();
    /**
    \brief Get the joint mask for joints betwwen root and given joint, starting from joint with rank rankOfFirstActivatedJoint
     */
    vectorN& customMask(CjrlJoint* inJoint, unsigned int rankOfFirstActivatedJoint);
    /**
    \brief Get the joint mask for the trunc (chest + head + legs)
     */
    vectorN& truncMask();
    /**
    \brief Get the joint mask for the whole body
     */
    vectorN& wholeBodyMask();
    /**
    \brief Get the contribution weights of joints for the double support case
     */
    vectorN& weightsDoubleSupport();
    /**
    \brief Get the contribution weights when only the left leg is supporting
     */
    vectorN& weightsLeftLegSupporting();
    /**
    \brief Get the contribution weights when only the right leg is supporting
     */
    vectorN& weightsRightLegSupporting();
    
    /**
    \brief Destructor
     */
    ~ChppGikMaskFactory();

private:
    
    void buildWeightVectors();
            
    bool containsJoint(CjrlJoint* inJoint,std::vector<CjrlJoint*>& inVec);
    
    unsigned int rankInSupportedMassVector(const CjrlJoint* inJoint);
    
    void supportedMass(vectorN& massVec, const CjrlJoint* inJoint, CjrlJoint* excludedChild);

    CjrlHumanoidDynamicRobot* attRobot;
    
    unsigned int attNumJoints;
    vectorN attUpperBody;
    vectorN attLegs;
    vectorN attLeftArm;
    vectorN attRightArm;
    vectorN attChestAndArms;
    vectorN attChestAndHead;
    vectorN attWholeBody;
    vectorN attWeightsDouble;
    vectorN attCustomMask;
    vectorN attWeightsLeftLegSupporting;
    vectorN attWeightsRightLegSupporting;

};

#endif
