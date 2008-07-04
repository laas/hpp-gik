#ifndef HPP_GIK_BASIC_EXAMPLE_H
#define HPP_GIK_BASIC_EXAMPLE_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "gikTask/jrlGikObjectConstructor.h"

#include "constraints/hppGikPlaneConstraint.h"
#include "constraints/hppGikPositionConstraint.h"
#include "constraints/hppGikRotationConstraint.h"
#include "constraints/hppGikTransformationConstraint.h"
#include "constraints/hppGikParallelConstraint.h"
#include "constraints/hppGikGazeConstraint.h"
#include "constraints/hppGikComConstraint.h"

#include "robot/hppGikStandingRobot.h"


/**
 \brief This object inplements a simple motion planning algorithm using a GikSolverHumanoid object
 */
class ChppGikBasicExample
{
public:
    /**
        \brief Constructor
     */
    ChppGikBasicExample();

    /**
        \brief Destructor
     */
    ~ChppGikBasicExample();

    /**
        \brief Call the planning method
     */
    void planExample();


private:

    /**
        \brief Create a humanoid robot
     */
    void createStandingRobot();

    /**
        \brief A pointer to the created humanoid robot
    */
    CjrlHumanoidDynamicRobot* attRobot;

    /**
        \brief A constructor for constraints
    */
    CjrlGikObjectConstructor<ChppGikPlaneConstraint, ChppGikParallelConstraint, ChppGikRotationConstraint, ChppGikPositionConstraint, ChppGikTransformationConstraint, ChppGikPointingConstraint, ChppGikGazeConstraint, ChppGikComConstraint> attGikFactory;
    
    /**
        \brief A pointer to an object of class ChppGikStandingRobot built from attRobot
    */
    ChppGikStandingRobot* attStandingRobot;

    /**
        \brief The sampling period used to compute the dynamics in the robot
    */
    double attSamplingPeriod;

};

#endif
