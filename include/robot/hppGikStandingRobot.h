#ifndef HPP_GIK_STANDING_ROBOT_H
#define HPP_GIK_STANDING_ROBOT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlHumanoidDynamicRobot.h"
#include "robot/hppGikFootprintRelated.h"
#include "robot/hppGikMaskFactory.h"
#include "robot/hppGik2DShape.h"


/**
\brief This is a wrapper for a jrlHumanoidDynamicRobot, its support polygon and related information
*/
class ChppGikStandingRobot
{
public:
    /**
    \brief constructor. The passed robot should be in half sitting configuration. The world frame has its center on the ground on the middle of ankle projections, x axis pointing ahead, y axis to the left foot of the robot and z axis up.
    */
    ChppGikStandingRobot(CjrlHumanoidDynamicRobot* inRobot);

    /**
    \brief Get the jrl robot
    */
    CjrlHumanoidDynamicRobot* robot() const;

    /**
    \brief Get the current support polygon
    */
    ChppGikSupportPolygon* supportPolygon();

    /**
    \brief Test position of right foot against left
    */
    bool rightFootAhead();

    /**
    \brief Test position of left foot against right
    */
    bool leftFootAhead();

    /**
    \brief get the 2DShape representing the robot's waist seen from above
    */
    const ChppGik2DShape& waistShape()const;

    /**
    \brief get the 2DShape representing the robot's left foot seen from above. HRP2 feet dimensions hard coded
     */
    const ChppGik2DShape& leftFootShape()const;

    /**
    \brief get the 2DShape representing the robot's right foot seen from above. HRP2 feet dimensions hard coded
     */
    const ChppGik2DShape& rightFootShape()const;

    /**
    \name HalfSitting configuration related
    {@
    */

    /**
    \brief Get (dx, dy, dz) defined by (dy = YCOM - Yrfoot) and (dx = RfootR'(XCOM - Xrfoot)) in right foot frame when the robot is in half sitting stance
    */
    vector3d& halfsittingRelativeCOM();

    /**
    \brief Get the halfsitting configuration
     */
    vectorN& halfsittingConfiguration();

    /**
    \brief Get the distance separating the feet in halfsitting stance
     */
    double halfsittingFeetDistance();

    /**
    \brief Get the haight of the waist in halfsitting stance
     */
    double halfsittingWaistHeight();

    /**
    \brief Get the transformation of the waist in halfsitting stance, in world frame
     */
    matrix4d& halfsittingWaistTransformation();

    /**
    \brief Get the transformation of the left wrist in halfsitting stance, in world frame
     */
    matrix4d& halfsittingLeftWristTransformation();

    /**
    \brief Get the transformation of the right wrist in halfsitting stance, in world frame
     */
    matrix4d& halfsittingRightWristTransformation();

    /**
    \brief Get the transformation of the right foot in halfsitting stance, in world frame
     */
    matrix4d& halfsittingRightFootTransformation();

    /**
    \brief Get the transformation of the left foot in halfsitting stance, in world frame
     */
    matrix4d& halfsittingLeftFootTransformation();

    /**
    \brief Get the transformation of the head in halfsitting stance, in world frame
     */
    matrix4d& halfsittingHeadTransformation();

    /**
    \brief apply the static half sitting configuration to the robot
    */
    void staticHalfsitting();

    /**
    @}
     */


    /**
    \brief Get the associated mask factory
     */
    ChppGikMaskFactory* maskFactory();

    /**
    \brief apply a static configuration on the robot
    */
    bool staticState(const vectorN& inConfig);
    
    /**
    \brief update velocity and acceleration of joints in robot
    */
    void updateJointVA(double inSamplingPeriod);
    /**
    \brief Simulate the application of a configuration to the robot (uses the Finite Difference control scheme). The ZMP resulting from the motion is computed in world frame and in waist frame. This method also computes the planned ZMP position in waist frame based on the position in world frame.
    \return false if the provided configuration does not meet requirements of method FiniteDifferenceStateUpdate() in jrl robot.
    */
    void updateDynamics(double inSamplingPeriod, const vector3d& inZMPworPla, vector3d& outZMPworObs, vector3d& outZMPwstObs, vector3d& outZMPwstPla);


    /**
    \brief compute the transformation of the root body based on the desired transformation of a body in the world and return the corresponding robot configuration.
    */
    vectorN computeConfigurationWrtFreeFlyer(CjrlJoint* inJoint, matrix4d& inFreeFlyerInWorld);

    /**
    \brief 
    */
    const ChppGik2DShape& supportPolygonShape();
    
    /**
    \brief 
    */
    void computeFeet2DConvexHull(std::vector<const ChppGikLinkedVertex*>& outVertices);
    
    /**
    \brief Destructor
    */
    ~ChppGikStandingRobot();

private:
    CjrlHumanoidDynamicRobot* attRobot;
    ChppGikMaskFactory* attMaskFactory;
    ChppGikSupportPolygon* attCurrentSupportPolygon;
    vectorN attSupportPolygonConfig;
    vector3d attRelativeCOM;
    vectorN attHalfSittingConfig;
    matrix4d attLWrist, attRWrist, attHead, attWaist, attLFoot, attRFoot;
    ChppGik2DShape attWaistShape,attLeftFootShape, attRightFootShape;
    vector3d tempVec;
    matrix4d tempInv, tempM4;
    vectorN attPreviousVelocity;
    vectorN attPreviousConfiguration, attAcceleration, attVelocity;
    ChppGik2DShape attSPShape;
    std::vector<ChppGikLinkedVertex> attElements;
};

#endif
