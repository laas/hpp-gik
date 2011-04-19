#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/constraint/gaze-constraint.hh"

ChppGikGazeConstraint::ChppGikGazeConstraint(CjrlHumanoidDynamicRobot& inRobot, const vector3d& inTargetWorldPoint): ChppGikPointingConstraint(inRobot,*(inRobot.gazeJoint()),inRobot.gazeOrigin(),inRobot.gazeDirection(),inTargetWorldPoint)
{
    if (attLocalVector(0) ==0 && attLocalVector(1)==0 && attLocalVector(2)==0)
    {
        std::cout << "ChppGikGazeConstraint constructor: Warning: gaze direction was not set. Using default value [0;1;0]\n" << std::endl;
        attLocalVector(0) =0;
        attLocalVector(1) =1;
        attLocalVector(2) =0;
        attLocalVectorVector3[0] =0;
        attLocalVectorVector3[1] =1;
        attLocalVectorVector3[2] =0;
    }
}

ChppGikGazeConstraint::~ChppGikGazeConstraint()
{}
