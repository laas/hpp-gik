#ifndef HPP_GIK_STANDING_ROBOT_H
#define HPP_GIK_STANDING_ROBOT_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlHumanoidDynamicRobot.h"
#include "hpp/gik/robot/foot-print-related.hh"
#include "hpp/gik/robot/mask-factory.hh"
#include "hpp/gik/robot/shape-2d.hh"


/**
A rigid body
*/
class ChppGikBody
{
public:
    matrix3d R;
    vector3d w, dw;
    vector3d p, v, dv;
    
    matrix3d pastR;
    vector3d pastw;
    vector3d pastp, pastv;
};

/**
\brief This is a wrapper for a jrlHumanoidDynamicRobot, its support polygon and related information.  Some returned data are hardcoded for hrp2 robot.
\ingroup robot
*/
class ChppGikStandingRobot
{
public:
    /**
    \brief constructor. The passed robot should be in half sitting configuration. The world frame has its center on the ground on the middle of ankle projections, x axis pointing ahead, y axis to the left foot of the robot and z axis up.
    */
    ChppGikStandingRobot(CjrlHumanoidDynamicRobot& inRobot, ChppGik2DShape& inLeftFootprintShape, ChppGik2DShape& inRightFootprintShape);

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
    return a unitary vector expressed in the waist joint's local frame, coinciding with the reference frame (world) Z axis
    */
    const vector3d& halfsittingLocalWaistVertical();
            
    /**
    \brief Get the distance separating the feet in halfsitting stance
     */
    double halfsittingFeetDistance();

    /**
    \brief Get the haight of the waist in halfsitting stance
     */
    double halfsittingWaistHeight();

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
    \brief compute the transformation of the root body based on the desired transformation of a body in the world and return the corresponding robot configuration.
     */
    vectorN computeConfigurationWrtFreeFlyer(CjrlJoint* inJoint, matrix4d& inFreeFlyerInWorld);

    /**
    \name Robot State Update
    @{
     */
    /**
    \brief Apply a static configuration on the robot
    */
    bool staticState(const vectorN& inConfig);

    /**
    \brief Update robot kinematics(cartesian positions, velocities and accelerations of bodies + center of mass) and dynamics(ZMP) based on given root and other joints configurations.
    */
    void updateRobot(const matrix4d& inRootPose, const vectorN& inJoints, double inTimeStep);
    /**
    @}
    */

    /**
    \brief 
    */
    const ChppGik2DShape& supportPolygonShape();

    /**
    \brief 
     */
    bool isPointInsideSupportPolygon(double inX, double inY,double safetyMargin = 0.01);

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
    matrix4d attWaist, attLFoot, attRFoot;
    ChppGik2DShape attLeftFootShape, attRightFootShape;
    vector3d tempVec;
    vector3d attWaistVertical;
    matrix4d tempInv, tempM4;
    vectorN attPreviousConfiguration, attConfiguration, attVelocity, attPreviousVelocity, attAcceleration;
    ChppGik2DShape attSPShape;
    std::vector<ChppGikLinkedVertex> attElements;
    double attAnklePos;

    vector3d FD_tmp,FD_tmp2,FD_tmp3,FD_w;
    matrix3d FD_Ro,FD_Roo,FD_Rt;

};

#endif
