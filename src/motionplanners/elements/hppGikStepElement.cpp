#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "motionplanners/elements/hppGikStepElement.h"
#include "hppGikTools.h"


using namespace ublas;


ChppGikStepElement::ChppGikStepElement(CjrlHumanoidDynamicRobot* inRobot,double inStartTime, const ChppGikFootprint* inFootprint, bool isRightFoot, double inSamplingPeriod, double inFinalZMPCoefficient, double inEndShiftTime, double inStartZMPShiftTime, double inFootMotionDuration, double inStepHeight):ChppGikLocomotionElement( inRobot, inStartTime, inFootMotionDuration+inStartZMPShiftTime+inEndShiftTime, inSamplingPeriod)
{

    attUseZMPcoefficient = true;
    //thresholding endZMPcoef
    if (attFinalZMPCoef > 1)
        attFinalZMPCoef = 1;
    if (attFinalZMPCoef < 0)
        attFinalZMPCoef = 0;

    attFinalZMPCoef = inFinalZMPCoefficient;

    init(inFootMotionDuration, isRightFoot, inStepHeight, inFootprint, inStartZMPShiftTime, inEndShiftTime);
}

ChppGikStepElement::ChppGikStepElement(CjrlHumanoidDynamicRobot* inRobot, const ChppGikFootprint* inFootprint, double inStartTime, bool isRightFoot, double rightfoot2TargetZMPX, double rightfoot2TargetZMPY, double inSamplingPeriod, double inEndShiftTime, double inStartZMPShiftTime, double inFootMotionDuration, double inStepHeight):ChppGikLocomotionElement( inRobot, inStartTime, inFootMotionDuration+inStartZMPShiftTime+inEndShiftTime, inSamplingPeriod)
{
    attUseZMPcoefficient = false;
    attRfoot2TargetZMPX = rightfoot2TargetZMPX;
    attRfoot2TargetZMPY = rightfoot2TargetZMPY;

    init(inFootMotionDuration, isRightFoot, inStepHeight, inFootprint, inStartZMPShiftTime, inEndShiftTime);
}

void ChppGikStepElement::init(double inFootMotionDuration, bool isRight, double inHeight, const ChppGikFootprint* inFootprint,double inStartShiftTime, double inEndShiftTime)
{
    vector3d dum;
    attShift1 = new ChppGikZMPshiftElement( attRobot, dum, attStartTime, inStartShiftTime, attSamplingPeriod);
    attShift2 = new ChppGikZMPshiftElement( attRobot, dum, attStartTime + inStartShiftTime + inFootMotionDuration, inEndShiftTime, attSamplingPeriod);

    attFootDisplace = new ChppGikFootDisplaceElement( attRobot, attStartTime + inStartShiftTime, inFootprint, isRight, inFootMotionDuration, attSamplingPeriod, inHeight);

    if (isRight)
    {
        attSupportFoot = attRobot->leftFoot();
        attConstrainedFoot = attRobot->rightFoot();
    }
    else
    {
        attSupportFoot = attRobot->rightFoot();
        attConstrainedFoot = attRobot->leftFoot();
    }
}

ChppGikStepElement::~ChppGikStepElement()
{
    delete attShift1;
    delete attShift2;
    delete attFootDisplace;
}

bool ChppGikStepElement::isRight()
{
    return attFootDisplace->isRight();
}

bool ChppGikStepElement::plan(ChppGikSupportPolygon& supportPolygon, vector3d& ZMP)
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

    const ChppGikFootprint *supportFootprint = 0;
    if (attFootDisplace->isRight())
        supportFootprint = supportPolygon.leftFootprint();
    else
        supportFootprint = supportPolygon.rightFootprint();

    {
        supportZMP[0] = supportFootprint->x();
        supportZMP[1] = supportFootprint->y();
        attShift1->targetZMP( supportZMP );
        attPlanSuccess = attShift1->plan(supportPolygon, ZMP);
    }

    if (!attPlanSuccess)
    {
        std::cout << "Step Failed 0\n";
        return false;
    }

    attPlanSuccess = attFootDisplace->plan(supportPolygon, ZMP);

    if (!attPlanSuccess)
    {
        std::cout << "Step Failed 1\n";
        return false;
    }

    {
        if (attUseZMPcoefficient)
        {
            finalZMP[0] = (1 - attFinalZMPCoef)*supportFootprint->x() + attFinalZMPCoef * attFootDisplace->targetFootprint()->x();
            finalZMP[1] =  (1 - attFinalZMPCoef)*supportFootprint->y() + attFinalZMPCoef * attFootDisplace->targetFootprint()->y();
        }
        else
        {
            const ChppGikFootprint* rfp = supportPolygon.rightFootprint();
            finalZMP[0] = rfp->x() + cos(rfp->th())*attRfoot2TargetZMPX - sin(rfp->th())*attRfoot2TargetZMPY;
            finalZMP[1] = rfp->y() + sin(rfp->th())*attRfoot2TargetZMPX + cos(rfp->th())*attRfoot2TargetZMPY;
        }

        attShift2->targetZMP( finalZMP );
        attPlanSuccess = attShift2->plan(supportPolygon, ZMP);
    }

    if (!attPlanSuccess)
        std::cout << "Step Failed 2\n";

    ChppGikTools::overlapConcat(attZMPmotion, attShift1->ZMPmotion(),  1);
    ChppGikTools::overlapConcat(attZMPmotion, attFootDisplace->ZMPmotion(),  1);
    ChppGikTools::overlapConcat(attZMPmotion, attShift2->ZMPmotion(),  1);

    return attPlanSuccess;
}

CjrlGikMotionConstraint* ChppGikStepElement::clone() const
{
    ChppGikStepElement* el = 0;
    if (attUseZMPcoefficient)
    {
        el = new ChppGikStepElement(attRobot,attStartTime, attFootDisplace->targetFootprint(), attFootDisplace->isRight(), attSamplingPeriod, attFinalZMPCoef, attShift2->duration(), attShift1->duration(), attFootDisplace->duration(), attFootDisplace->height());
    }
    else
    {
        el = new ChppGikStepElement(attRobot, attFootDisplace->targetFootprint(), attStartTime, attFootDisplace->isRight(), attRfoot2TargetZMPX, attRfoot2TargetZMPY, attSamplingPeriod, attShift2->duration(),  attShift1->duration(), attFootDisplace->duration(), attFootDisplace->height());
    }

    el->postProlongate( attPostProlongation );
    el->preProlongate( attPreProlongation );
    return el;
}


CjrlGikStateConstraint* ChppGikStepElement::stateConstraintAtTime(double inTime)
{
    if (!attPlanSuccess)
        return 0;
    CjrlGikStateConstraint* retC = 0;
    retC = attShift1->stateConstraintAtTime( inTime );
    if (!retC)
    {
        retC = attFootDisplace->stateConstraintAtTime( inTime );
        if (!retC)
        {
            retC = attShift2->stateConstraintAtTime( inTime );
        }
    }

    return retC;
}


CjrlJoint* ChppGikStepElement::supportFootAtTime(double inTime)
{
    if (!attPlanSuccess)
        return 0;
    CjrlJoint* retJ = 0;
    retJ = attShift1->supportFootAtTime( inTime );
    if (!retJ)
    {
        retJ = attFootDisplace->supportFootAtTime( inTime );
        if (!retJ)
            retJ = attShift2->supportFootAtTime( inTime );
    }

    return retJ;
}


void ChppGikStepElement::preProlongate(double inPreProlongation)
{
    attPreProlongation = (inPreProlongation>0.0)?inPreProlongation:0.0;
    attModifiedStart = attStartTime - attPreProlongation;
    attShift1->preProlongate( attPreProlongation );
    attPlanSuccess = false;
}


void ChppGikStepElement::postProlongate(double inPostProlongation)
{
    attPostProlongation = (inPostProlongation>0.0)?inPostProlongation:0.0;
    attModifiedEnd = attEndTime + attPostProlongation;
    attShift2->postProlongate( attPostProlongation );
    attPlanSuccess = false;
}
