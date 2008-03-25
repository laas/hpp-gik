#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "hppGikTools.h"
#include "motionplanners/elements/hppGikZMPshiftElement.h"

using namespace ublas;

ChppGikZMPshiftElement::ChppGikZMPshiftElement( CjrlHumanoidDynamicRobot* inRobot, const vector3d& targetZMP, double inStartTime, double inDuration, double inSamplingPeriod):ChppGikLocomotionElement( inRobot, inStartTime, inDuration, inSamplingPeriod)
{
    attTargetZMP = targetZMP;
    
    vector3d zer;
    zer[0] = zer[1] = zer[2] = 0;
    attConstraint = new ChppGikTransformationConstraint(*attRobot, *(attRobot->rightFoot()), zer, attRobot->rightFoot()->currentTransformation());
    
    attMotionConstraint = this;
}

ChppGikZMPshiftElement::~ChppGikZMPshiftElement()
{
    delete attConstraint;
}


CjrlGikMotionConstraint* ChppGikZMPshiftElement::clone() const
{
    ChppGikZMPshiftElement* el =  new ChppGikZMPshiftElement( attRobot, attTargetZMP, attStartTime, attDuration, attSamplingPeriod);
    
    el->postProlongate( attPostProlongation );
    el->preProlongate( attPreProlongation );
    
    return el;
}


CjrlGikStateConstraint* ChppGikZMPshiftElement::stateConstraintAtTime(double inTime)
{
    if (!attPlanSuccess)
        return 0;
    
    if ((inTime < attModifiedStart + attEps) || (inTime > attModifiedEnd + attEps))
        return 0;
    
    unsigned int i = ChppGikTools::timetoRank( attModifiedStart, inTime, attSamplingPeriod );
    if (i==1)
        attConstraint->targetTransformation( attConstrainedFoot->currentTransformation() );

    return attConstraint;
}


CjrlJoint* ChppGikZMPshiftElement::supportFootAtTime(double inTime)
{
    if (!attPlanSuccess)
        return 0;
    if ((inTime > attModifiedStart + attEps) && (inTime < attModifiedEnd + attEps))
        return attSupportFoot;
    return 0;
}

bool ChppGikZMPshiftElement::plan(ChppGikSupportPolygon& supportPolygon, vector3d& ZMP)
{
    attPlanSuccess = false;


    if (!supportPolygon.isPointInside(ZMP[0], ZMP[1]))
    {
        std::cout << "ChppGikZMPshiftElement::plan() bad initial ZMP\n";
        return false;
    }

    if (!supportPolygon.isPointInside(attTargetZMP(0), attTargetZMP(1)))
    {
        std::cout << "ChppGikZMPshiftElement::plan() bad ZMP target\n";
        return false;
    }

    attPlanSuccess = true;
    
    if (supportPolygon.isRightLegSupporting())
    {
        attSupportFoot = attRobot->rightFoot();
        attConstrainedFoot = attRobot->leftFoot();
    }
    else
    {
        attSupportFoot = attRobot->leftFoot();
        attConstrainedFoot = attRobot->rightFoot();
    }

    attConstraint->joint(attConstrainedFoot);

    vectorN initialZMPU(3);
    vectorN finalZMPU(3);
    ChppGikTools::Vector3toUblas(ZMP,initialZMPU);
    ChppGikTools::Vector3toUblas(attTargetZMP,finalZMPU);
    matrixNxP data;
    ChppGikTools::multiLinearInterpolation( attDuration, attSamplingPeriod, initialZMPU, finalZMPU, data);
    
    ChppGikTools::prolongateTimeBased(attPreProlongation, attPostProlongation, attSamplingPeriod, data, attZMPmotion);
    ZMP = attTargetZMP;

    return attPlanSuccess;
}

