#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "motionplanners/elements/hppGikWalkElement.h"
#include "hppGikTools.h"


using namespace boost::numeric::ublas;

ChppGikWalkElement::ChppGikWalkElement(CjrlHumanoidDynamicRobot* inRobot, double inSamplingPeriod,double inStartTime, const std::vector<ChppGikStepTarget*>& inAbsoluteSteps):ChppGikLocomotionElement( inRobot, inStartTime, 0, inSamplingPeriod)
{
    attZMPstart = 0.1;
    attZMPend = 0.1;
    attFootFlight = 1.2;
    attStepDuration = attZMPstart+attZMPend+attFootFlight;
    attDuration = attStepDuration*inAbsoluteSteps.size();
    attEndTime = attDuration + attStartTime;
    attModifiedEnd = attEndTime;

    for (unsigned int i=0;i<inAbsoluteSteps.size();i++)
        attSteps.push_back(new ChppGikStepElement(attHumanoidRobot, attStartTime+attStepDuration*i, &(inAbsoluteSteps[i]->footprint()), inAbsoluteSteps[i]->isForRight(), attSamplingPeriod , 0.5,  attZMPend, attZMPstart, attFootFlight));
    
    for (unsigned int i=0;i<inAbsoluteSteps.size();i++)
        attStepTargets.push_back(new ChppGikStepTarget(*(inAbsoluteSteps[i])));
}


ChppGikWalkElement::~ChppGikWalkElement()
{
    for (unsigned int i=0;i<attSteps.size();i++)
    {
        delete attSteps[i];
        delete attStepTargets[i];
    }
}

bool ChppGikWalkElement::plan(ChppGikSupportPolygon& supportPolygon, vector3d& ZMP)
{
    attPlanSuccess = false;

    attZMPmotion.resize(3,1);
    for (unsigned int i = 0 ; i<3; i++)
        attZMPmotion(i,0) = ZMP[i];

    if (!supportPolygon.isPointInsideSafeZone(ZMP[0], ZMP[1]))
    {
        std::cout << "ChppGikZMPshiftElement::plan() bad initial ZMP\n";
        return attPlanSuccess;
    }

    if (!supportPolygon.isDoubleSupport())
    {
        std::cout << "ChppGikZMPshiftElement::plan() bad initial Supoport Polygon\n";
        return attPlanSuccess;
    }

    vector3d supportZMP, finalZMP;
    supportZMP[2] = finalZMP[2] = ZMP[2];

    for (unsigned int i=0;i<attSteps.size();i++)
    {
        attPlanSuccess = attSteps[i]->plan(supportPolygon, ZMP);
        if (!attPlanSuccess)
        {
            std::cout << "Failed to plan "<< i+1 <<"-th step in WalkElement\n";
            return false;
        }
    }
    
    for (unsigned int i=0;i<attSteps.size();i++)
    {
        ChppGikTools::overlapConcat(attZMPmotion, attSteps[i]->ZMPmotion(),  1);
    }

    return true;
}

CjrlGikMotionConstraint* ChppGikWalkElement::clone() const
{
    return new ChppGikWalkElement(attHumanoidRobot,attSamplingPeriod,attStartTime,attStepTargets);
}


CjrlGikStateConstraint* ChppGikWalkElement::stateConstraintAtTime(double inTime)
{
    if (!attPlanSuccess)
        return 0;
    CjrlGikStateConstraint* retC = 0;
    
    for (unsigned int i=0;i<attSteps.size();i++)
    {
        retC = attSteps[i]->stateConstraintAtTime( inTime );
        if (retC)
            break;
    }
    
    return retC;
}


CjrlJoint* ChppGikWalkElement::supportFootAtTime(double inTime)
{
    if (!attPlanSuccess)
        return 0;
    CjrlJoint* retJ = 0;
    
    for (unsigned int i=0;i<attSteps.size();i++)
    {
        retJ = attSteps[i]->supportFootAtTime( inTime );
        if (retJ)
            break;
    }
    return retJ;
}


void ChppGikWalkElement::preProlongate(double inPreProlongation)
{
    attPreProlongation = (inPreProlongation>0.0)?inPreProlongation:0.0;
    attModifiedStart = attStartTime - attPreProlongation;
    if (!attSteps.empty())
        attSteps[0]->preProlongate( attPreProlongation );
    attPlanSuccess = false;
}


void ChppGikWalkElement::postProlongate(double inPostProlongation)
{
    attPostProlongation = (inPostProlongation>0.0)?inPostProlongation:0.0;
    attModifiedEnd = attEndTime + attPostProlongation;
    if (!attSteps.empty())
        attSteps[attSteps.size()-1]->postProlongate( attPostProlongation );
    attPlanSuccess = false;
}
