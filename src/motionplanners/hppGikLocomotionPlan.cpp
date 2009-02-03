#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "motionplanners/hppGikLocomotionPlan.h"
#include "motionplanners/elements/hppGikStepElement.h"

#include "hppGikTools.h"


ChppGikLocomotionPlan::ChppGikLocomotionPlan(ChppGikMotionPlan* inAssociatedMotionPlan, ChppGikStandingRobot* inStandingRobot, double inSamplingPeriod)
{
    attRobot = inStandingRobot->robot();
    attExtraEndTime = 0.0;
    attStartTime = attModifiedStartTime = 0.0;
    attEndTime = attModifiedEndTime = 0.0;
    attSamplingPeriod = inSamplingPeriod;
    attEps = attSamplingPeriod/2;

    attAssociatedMotionPlan = inAssociatedMotionPlan;
    attStandingRobot = inStandingRobot;

    attPreviewController = new ChppGikPreviewController( inSamplingPeriod );

    
    attFootMotionPlanRow = 0;
    attComMotionPlanRow = 0;
    attNoElementRow = 0;
    attNoElementCase = 0;
    attComMotion = 0;
    
    attPlanSuccess = false;
}

ChppGikLocomotionPlan::~ChppGikLocomotionPlan()
{
    clearElements();
    delete attPreviewController;
}

void ChppGikLocomotionPlan::clearElements() //i.e a different problem is coming
{
    clearSolverMess();

    for (unsigned int i=0;i<attElements.size();i++)
        delete attElements[i];

    attElements.clear();

    attPlanSuccess = false;
    attEndTime = attStartTime = attModifiedEndTime = attModifiedStartTime = 0.0;
}

void ChppGikLocomotionPlan::clearSolverMess() // might be the same problem resolved
{
    if (attFootMotionPlanRow)
        for (unsigned int i=0;i<attElements.size();i++)
            attFootMotionPlanRow->removeMotion(attElements[i]);
    if (attComMotion)
    {
        attComMotionPlanRow->removeMotion( attComMotion );
        delete attComMotion;
        attComMotion = 0;
    }
    if (attNoElementCase)
    {
        attNoElementRow->removeMotion( attNoElementCase );
        delete attNoElementCase;
        attNoElementCase = 0;
    }
    attFootMotionPlanRow = 0;
}

void ChppGikLocomotionPlan::extraEndTime(double inDuration)
{
    attExtraEndTime = (inDuration<0.0)?0.0:inDuration;
    attPlanSuccess = false;
}

double ChppGikLocomotionPlan::extraEndTime()
{
    return attExtraEndTime;
}

bool ChppGikLocomotionPlan::addElement(ChppGikLocomotionElement * inElement)
{
    double epsilon = attSamplingPeriod/2;

    if (inElement->startTime() < attStartTime - epsilon)
    {
        std::cout << "ChppGikLocomotionPlan::addElement() Invalid start time element \n";
        return false;
    }

    if (attElements.empty())
    {
        attElements.push_back(inElement);
        attEndTime = inElement->endTime();
        return true;
    }

    if (inElement->startTime() >= (*(attElements.end()-1))->endTime() - epsilon)
    {
        attElements.push_back(inElement);
        attEndTime = inElement->endTime();
        return true;
    }

    if (inElement->endTime() < (*(attElements.begin()))->startTime() + epsilon)
    {
        attElements.insert(attElements.begin(),inElement);
        return true;
    }

    bool insertionSuccess = false;

    std::vector<ChppGikLocomotionElement*>::iterator iter;
    for (iter = attElements.begin(); iter != attElements.end()-1; iter++)
    {
        if (((*iter)->endTime() - epsilon < inElement->startTime()) && ((*(iter+1))->startTime() + epsilon > inElement->endTime()))
        {
            attElements.insert(iter,inElement);
            insertionSuccess  = true;
            attPlanSuccess = false;
            break;
        }
    }
    return insertionSuccess;
}


double ChppGikLocomotionPlan::endTime()
{
    return attEndTime;
}


bool ChppGikLocomotionPlan::solve()
{
    attPlanSuccess = false;

    clearSolverMess();

    //get previewTime
    double prevT = attPreviewController->previewTime();

    bool retVal;
    if (attElements.empty())
    {
        attModifiedStartTime = attStartTime;
        attModifiedEndTime = attEndTime + attExtraEndTime;
        attNoElementCase = new ChppGikNoLocomotion( attRobot, attRobot->rightFoot(), attModifiedStartTime, attModifiedEndTime, attStandingRobot->maskFactory()->legsMask(),0);
        attNoElementRow = attAssociatedMotionPlan->addMotion( attNoElementCase );
    }
    else
    {
        attModifiedStartTime = attElements[0]->startTime() - prevT;
        attModifiedEndTime = attEndTime + attExtraEndTime;
        if (attModifiedStartTime > attStartTime)
            attModifiedStartTime = attStartTime;
        attElements[0]->preProlongate(attElements[0]->startTime() - attModifiedStartTime);
        //Extra time due to preview controller
        attExtraZMPEndTime = prevT;
        //build motions
        retVal = planElementsZMP();
        if (!retVal)
        {
            std::cout << "Failed to plan locomotion\n";
            return false;
        }
        //filter ZMP motion to improve stability
        matrixNxP filteringResult;
        ChppGikTools::multiFilter(attSamplingPeriod, attPlannedZMP, filteringResult);

        if (0)
            ChppGikTools::dumpMatrix("filterdebug",  attPlannedZMP, 0.0, attSamplingPeriod);


        attPlannedZMP = filteringResult;
        matrixNxP resultTrajCOMXY;
        retVal = attPreviewController->ZMPtoCOM(attPlannedZMP,resultTrajCOMXY);
        if (!retVal)
        {
            std::cout << "Failed to plan center of mass motion\n";
            return false;
        }
        attComMotion = new ChppGikComMotion( attRobot, attModifiedStartTime, attSamplingPeriod, attStandingRobot->maskFactory()->legsMask(),1);
        attComMotion->setSamples( resultTrajCOMXY );
        attComMotionPlanRow = attAssociatedMotionPlan->addMotion(attComMotion);
        for (unsigned int i=0;i<attElements.size();i++)
            attFootMotionPlanRow =  attAssociatedMotionPlan->addMotion( attElements[i]);
    }

    attPlanSuccess = true;
    return true;
}



bool ChppGikLocomotionPlan::planElementsZMP()
{
    bool retVal = true;
    double gapTime;

    ChppGikSupportPolygon supportPolygon(*(attStandingRobot->supportPolygon()));
    vector3d ZMP = attRobot->positionCenterOfMass();
    ZMP[2] = 0.0;
    attPlannedZMP.resize(3,1);
    for (unsigned int i=0;i<3;i++)
        attPlannedZMP(i,0) = ZMP[i];

    std::vector<ChppGikLocomotionElement*>::iterator iter;
    for (iter = attElements.begin(); iter != attElements.end()-1; iter++)
    {
        gapTime = (*(iter+1))->startTime() - (*iter)->endTime();
        if (gapTime>0.0)
            (*iter)->postProlongate(gapTime);
        retVal = (*iter)->plan(supportPolygon, ZMP);

        if (!retVal)
        {
            std::cout << "Failed 0\n";
            return false;
        }

        if (!supportPolygon.isPointInside( ZMP[0],ZMP[1]))
        {
            std::cout << "Planned ZMP out of support polygon. Aborting Locomotion planning\n";
            return false;
        }

        ChppGikTools::overlapConcat(attPlannedZMP, (*iter)->ZMPmotion(),  1);
    }

    (*iter)->postProlongate(attExtraEndTime);
    retVal = (*iter)->plan(supportPolygon,ZMP);

    if (!retVal)
    {
        std::cout << "Failed 1\n";
        return false;
    }

    ChppGikTools::overlapConcat(attPlannedZMP, (*iter)->ZMPmotion(),  1);

    prolongateZMP(attExtraZMPEndTime);
    return true;
}



void ChppGikLocomotionPlan::prolongateZMP(double inDuration)
{

    unsigned int sizeNewChunk = (unsigned int)round(inDuration/attSamplingPeriod)-1;
    unsigned int previousSize = attPlannedZMP.size2();
    unsigned int newSize =  previousSize + sizeNewChunk;
    vectorN paddingZMP = boost_ublas::column(attPlannedZMP,previousSize-1);
    attPlannedZMP.resize(3,newSize,true);
    for (unsigned int i=previousSize; i<newSize; i++)
        boost_ublas::column(attPlannedZMP,i) = paddingZMP;
}

CjrlJoint* ChppGikLocomotionPlan::supportFootJoint(double inTime)
{
    if (!attPlanSuccess)
        return 0;
    CjrlJoint* joint = 0;
    if (attElements.empty())
    {
        if (inTime > attNoElementCase->endTime() + attEps || inTime < attNoElementCase->startTime() + attEps)
            return 0;
        return attNoElementCase->supportFoot();
    }
    else
    {
        std::vector<ChppGikLocomotionElement*>::iterator iter;
        for (iter = attElements.begin(); iter != attElements.end(); iter++)
        {
            joint = (*iter)->supportFootAtTime(inTime);
            if (joint)
                return joint;
        }
        return 0;
    }
}

bool ChppGikLocomotionPlan::getZMPAtTime(double inTime, vectorN& outZMP)
{
    if (!attPlanSuccess)
        return false;

    if (attElements.empty())
    {
        if (inTime > attNoElementCase->endTime() + attEps || inTime < attNoElementCase->startTime() + attEps)
            return false;
        outZMP = attNoElementCase->ZMP();
        return true;
    }
    else
    {
        unsigned int i = ChppGikTools::timetoRank( attModifiedStartTime, inTime, attSamplingPeriod);
        if (i>0 && i<attPlannedZMP.size2())
        {
            outZMP = column(attPlannedZMP, i );
            return true;
        }
        return false;
    }

}

const ChppGikLocomotionElement* ChppGikLocomotionPlan::activeElement(double inTime) const
{
    double epsilon = attSamplingPeriod/2;

    const ChppGikLocomotionElement* returnedPointer = 0;

    std::vector<ChppGikLocomotionElement*>::const_iterator iter;
    for (iter = attElements.begin(); iter != attElements.end(); iter++)
        if (((*iter)->startTime() + epsilon - attPreviewController->previewTime() < inTime) && (inTime < (*(iter))->endTime()) + 0.4) //^^;
        {
            returnedPointer = *iter;
            break;
        }

    return returnedPointer;
}

bool ChppGikLocomotionPlan::getWeightsAtTime(double inTime, vectorN& outWeights)
{
    double epsilon = attSamplingPeriod/2;

    outWeights = attStandingRobot->maskFactory()->weightsDoubleSupport();

    std::vector<ChppGikLocomotionElement*>::iterator iter;
    for (iter = attElements.begin(); iter != attElements.end(); iter++)
    {
        ChppGikStepElement* st = dynamic_cast<ChppGikStepElement*>(*iter);
        if (st)
            if (st->isRight())
                outWeights = attStandingRobot->maskFactory()->weightsLeftLegSupporting();
            else
                outWeights = attStandingRobot->maskFactory()->weightsRightLegSupporting();

        if (((*iter)->startTime() + epsilon < inTime) && (inTime < (*(iter))->endTime()))
            break;

    }
    return true;
}

