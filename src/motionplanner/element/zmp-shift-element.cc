#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "hpp/gik/tools.hh"
#include "hpp/gik/motionplanner/element/zmp-shift-element.hh"

using namespace boost::numeric::ublas;

ChppGikZMPshiftElement::ChppGikZMPshiftElement( CjrlHumanoidDynamicRobot* inRobot, const vector3d& targetZMP, double inStartTime, double inDuration, double inSamplingPeriod):ChppGikLocomotionElement( inRobot, inStartTime, inDuration, inSamplingPeriod)
{
    attTargetZMP = targetZMP;

    vector3d zer;
    zer[0] = zer[1] = zer[2] = 0;
    attConstraint =
      new ChppGikTransformationConstraint(*attRobot,
					  *(attHumanoidRobot->rightAnkle()),
					  zer,
					  attHumanoidRobot->rightAnkle()->
					  currentTransformation());

    attMotionConstraint = this;
    attStartZMPCheck = true;
}

ChppGikZMPshiftElement::~ChppGikZMPshiftElement()
{
    delete attConstraint;
}


CjrlGikMotionConstraint* ChppGikZMPshiftElement::clone() const
{
    ChppGikZMPshiftElement* el =  new ChppGikZMPshiftElement( attHumanoidRobot, attTargetZMP, attStartTime, attDuration, attSamplingPeriod);

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

    //if (attConstraint)
    //std::cout << "time " << inTime << " targetTr " << attConstraint->targetTransformation() << "\n";
    return attConstraint;
}

ChppGikTransformationConstraint* ChppGikZMPshiftElement::footConstraintAtTime ( double inTime )
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

void ChppGikZMPshiftElement::startZMPCheck(bool inChoice)
{
    attStartZMPCheck = inChoice;
}

bool ChppGikZMPshiftElement::plan(ChppGikSupportPolygon& supportPolygon, vector3d& ZMP)
{
    

    //std::cout << "shift ZMP from " << ZMP <<  " to " << attTargetZMP << "\n";
    /*
    attPlanSuccess = false;
    if (attStartZMPCheck)
    {
        if (!supportPolygon.isPointInsideSafeZone(ZMP[0], ZMP[1]))
        {
            std::cout << "ChppGikZMPshiftElement::plan() bad initial ZMP\n";
            return false;
        }
    }

    if (!supportPolygon.isPointInsideSafeZone(attTargetZMP(0), attTargetZMP(1)))
    {
        std::cout << "ChppGikZMPshiftElement::plan() bad ZMP target\n";
        std::cout << attTargetZMP(0)<<"\n";
        std::cout << attTargetZMP(1)<<"\n";
        supportPolygon.print();
        return false;
    }
    */

    attPlanSuccess = true;

    if (supportPolygon.isRightLegSupporting())
    {
        attSupportFoot = attHumanoidRobot->rightAnkle();
        attConstrainedFoot = attHumanoidRobot->leftAnkle();
    }
    else
    {
        attSupportFoot = attHumanoidRobot->leftAnkle();
        attConstrainedFoot = attHumanoidRobot->rightAnkle();
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

