#include "constraints/hppGikGazeConstraint.h"

ChppGikGazeConstraint::ChppGikGazeConstraint(CjrlHumanoidDynamicRobot& inRobot, const vector3d& inTargetWorldPoint): ChppGikPointingConstraint(inRobot,*(inRobot.gazeJoint()),inRobot.gazeOrigin(),inRobot.gazeDirection(),inTargetWorldPoint)
{
    if (attLocalVector(0) ==0 && attLocalVector(1)==0 && attLocalVector(2)==0)
    {
        //std::cout << "ChppGikGazeConstraint constructor: Warning: gaze direction was not set. Using default value [1;0;0]\n" << std::endl;
        attLocalVector(0) =1;
        attLocalVectorVector3[0] =1;
    }
}

ChppGikGazeConstraint::~ChppGikGazeConstraint()
{}
