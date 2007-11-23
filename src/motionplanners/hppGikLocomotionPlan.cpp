#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "motionplanners/hppGikLocomotionPlan.h"
#include "constraints/hppGikComConstraint.h"
#include "hppGikTools.h"


ChppGikLocomotionPlan::ChppGikLocomotionPlan(ChppGikMotionPlan* inAssociatedMotionPlan, ChppGikStandingRobot* inStandingRobot, double inStartTime, double inSamplingPeriod)
{
    attExtraEndTime = 1.0;
    attStartTime = inStartTime;
    attTasksEndTime = attStartTime;
    attSamplingPeriod = inSamplingPeriod;

    attAssociatedMotionPlan = inAssociatedMotionPlan;
    attStandingRobot = inStandingRobot;

    attSupportPolygonMotion = new ChppGikSupportPolygonMotion(inStartTime);
    attFootMotion = new ChppGikMotionConstraint(inSamplingPeriod);
    attComMotion = new ChppGikMotionConstraint(inSamplingPeriod);
    attPreviewController = new ChppGikPreviewController(inSamplingPeriod);
    addtoMotionPlan();
}

ChppGikLocomotionPlan::~ChppGikLocomotionPlan()
{
    attFootMotionPlanRow->removeMotionConstraint( attFootMotion );
    attComMotionPlanRow->removeMotionConstraint( attComMotion );
    
    delete attSupportPolygonMotion;
    delete attPreviewController;
    delete attComMotion;
    delete attFootMotion;
}

void ChppGikLocomotionPlan::clearTasks()
{
    attTasks.clear();
    attTasksEndTime = attStartTime;
    attExtraEndTime = 1.0;
}

void ChppGikLocomotionPlan::extendEnd(double inDuration)
{
    attExtraEndTime += inDuration;
}

bool ChppGikLocomotionPlan::reset(double inStartTime)
{
    //Clear previous support polygon vector
    attSupportPolygonMotion->reset(inStartTime);

    //Feet and Com Motions
    attFootMotion->clear();
    attComMotion->clear();
    attFootMotion->startTime(inStartTime);
    attComMotion->startTime(inStartTime);

    //ZMP
    //the robot is assumed to be still at first, so the projection of the center of mass on the flat ground gives the zmp
    attPlannedZMP.resize(3,1,false);
    vectorN initialZMP(3);
    ChppGikTools::Vector3toUblas(attStandingRobot->robot()->positionCenterOfMass(),initialZMP);
    attPlannedZMP(0,0) = initialZMP(0);
    attPlannedZMP(1,0) = initialZMP(1);
    attPlannedZMP(2,0) = 0.0;

    //check for double support
    if (!attStandingRobot->supportPolygon()->isDoubleSupport())
    {
        std::cout << "ChppGikLocomotionPlan::reset : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }
    attStandingRobot->supportPolygon()->rfootTransformation(attStandingRobot->robot()->rightFoot()->currentTransformation());
    attStandingRobot->supportPolygon()->lfootTransformation(attStandingRobot->robot()->leftFoot()->currentTransformation());
    //Add first support polygon to the support polygon motion
    attSupportPolygonMotion->pushbackSupportPolygon(attStandingRobot->supportPolygon(),0.0);

    //Non support foot: initial state constraint (default to right)
    ublas::zero_vector<double> localP(3);
    vector3d v3lP;
    ChppGikTools::UblastoVector3( localP, v3lP);
    ChppGikTransformationConstraint* footConstraint = new ChppGikTransformationConstraint(*attStandingRobot->robot(), *(attStandingRobot->robot()->rightFoot()), v3lP, attStandingRobot->robot()->rightFoot()->currentTransformation());
    attFootMotion->pushbackStateConstraint((CjrlGikTransformationConstraint*)footConstraint);
    delete footConstraint;

    return true;
}

const ChppGikLocomotionElement* ChppGikLocomotionPlan::activeTask(double inTime) const
{
    double epsilon = attSamplingPeriod/2;
    
    const ChppGikLocomotionElement* returnedPointer = 0;
    
    std::vector<ChppGikLocomotionElement*>::const_iterator iter;
    for (iter = attTasks.begin(); iter != attTasks.end(); iter++)
        if (((*iter)->startTime() - epsilon - attPreviewController->previewTime() < inTime) && (inTime < (*(iter))->endTime()) + 0.4)
        {
            returnedPointer = *iter;
            break;
        }

        return returnedPointer;
}

bool ChppGikLocomotionPlan::weightsAtTime(double inTime, vectorN& outWeights)
{
    double epsilon = attSamplingPeriod/2;

    outWeights = attStandingRobot->maskFactory()->weightsDoubleSupport();

    std::vector<ChppGikLocomotionElement*>::iterator iter;
    for (iter = attTasks.begin(); iter != attTasks.end(); iter++)
    {
        ChppGikStepElement* st = dynamic_cast<ChppGikStepElement*>(*iter);
        if (st)
            if (st->isRight())
                outWeights = attStandingRobot->maskFactory()->weightsLeftLegSupporting();
            else
                outWeights = attStandingRobot->maskFactory()->weightsRightLegSupporting();

        if (((*iter)->startTime() - epsilon < inTime) && (inTime < (*(iter))->endTime()))
            break;

    }
    return true;
}

bool ChppGikLocomotionPlan::addTask(ChppGikLocomotionElement * inTask)
{
    double epsilon = attSamplingPeriod/2;

    if (inTask->startTime() < attStartTime - epsilon)
    {
        std::cout << "ChppGikLocomotionPlan::addTask() Invalid start time task \n";
        return false;
    }

    if (attTasks.empty())
    {
        attTasks.push_back(inTask);
        attTasksEndTime = inTask->endTime();
        return true;
    }

    if (inTask->startTime() >= (*(attTasks.end()-1))->endTime() - epsilon)
    {
        attTasks.push_back(inTask);
        attTasksEndTime = inTask->endTime();
        return true;
    }

    if (inTask->endTime() < (*(attTasks.begin()))->startTime() + epsilon)
    {
        attTasks.insert(attTasks.begin(),inTask);
        return true;
    }

    bool insertionSuccess = false;

    std::vector<ChppGikLocomotionElement*>::iterator iter;
    for (iter = attTasks.begin(); iter != attTasks.end()-1; iter++)
    {
        if (((*iter)->endTime() - epsilon < inTask->startTime()) && ((*(iter+1))->startTime() + epsilon > inTask->endTime()))
        {
            attTasks.insert(iter,inTask);
            insertionSuccess  = true;
            break;
        }
    }
    return insertionSuccess;
}

double ChppGikLocomotionPlan::endTime()
{
    return attTasksEndTime;//+attExtraEndTime;
}

ChppGikSupportPolygonMotion* ChppGikLocomotionPlan::supportPolygonMotion()
{
    return attSupportPolygonMotion;
}

CjrlJoint* ChppGikLocomotionPlan::supportFootJoint(double inTime)
{
    double epsilon = attSamplingPeriod/2;
    if ((inTime < attFootMotion->startTime()-epsilon) || (inTime > attFootMotion->endTime()+epsilon))
    {
        std::cout << "ChppGikLocomotionPlan::supportFootJoint() invalid time, return NULL pointer\n";
        return NULL;
    }

    CjrlJoint* joint = 0;
    CjrlGikStateConstraint* scstr = attFootMotion->stateConstraintAtTime(inTime);

    ChppGikTransformationConstraint* cstr = dynamic_cast<ChppGikTransformationConstraint*>(scstr);

    if (cstr->joint() == attStandingRobot->robot()->leftFoot())
        joint =  attStandingRobot->robot()->rightFoot();
    else
        joint = attStandingRobot->robot()->leftFoot();


    return joint;
}

bool ChppGikLocomotionPlan::solveOneStage()
{
    //previewTime of planned motion
    double prevT = attPreviewController->previewTime();

    bool retVal;
    if (attTasks.empty())
    {
        retVal = reset(attStartTime);
        if (!retVal)
        {
            std::cout << "Failed to reset locomotionplan 0\n";
            return false;
        }
    }
    else
    {
        double moddedStartTime = attTasks[0]->startTime() - prevT;
        if (moddedStartTime >0)
            moddedStartTime = attStartTime;
        retVal = reset(moddedStartTime);
        if (!retVal)
        {
            std::cout << "Failed to reset locomotionplan 1\n";
            return false;
        }
        retVal = prolongate(attTasks[0]->startTime() - moddedStartTime);
        if (!retVal)
        {
            std::cout << "Failed to reset locomotionplan 2\n";
            return false;
        }
    }

    //Extra time due to preview controller
    attExtraZMPEndTime = prevT;
    //build motions
    retVal = buildMotions();
    if (!retVal)
    {
        std::cout << "Failed to plan locomotion\n";
        return false;
    }

    //filter ZMP motion to improve stability
    matrixNxP filteringResult;
    ChppGikTools::multiFilter(attSamplingPeriod, attPlannedZMP, filteringResult);
    //     ChppGikTools::dumpMatrix("filterdebug",  attPlannedZMP, 0.0, attSamplingPeriod);
    attPlannedZMP = filteringResult;


    matrixNxP resultTrajCOMXY;
    retVal = attPreviewController->ZMPtoCOM(attPlannedZMP,resultTrajCOMXY);
    if (!retVal)
    {
        std::cout << "Failed to plan center of mass motion\n";
        return false;
    }

    //fill in com motion constraint
    ChppGikComConstraint* comC = new ChppGikComConstraint(*(attStandingRobot->robot()),0,0);
    for (unsigned int i=0; i<attFootMotion->numberStateConstraints(); i++)
    {
        comC->targetXY(resultTrajCOMXY(0,i), resultTrajCOMXY(1,i));
        attComMotion->pushbackStateConstraint(comC);
    }

    delete comC;
    return true;
}



bool ChppGikLocomotionPlan::buildMotions()
{
    bool retVal = true;
    double gapTime;
    std::vector<ChppGikLocomotionElement*>::iterator iter;

    if (attTasks.size()>0)
    {
        for (iter = attTasks.begin(); iter != attTasks.end()-1; iter++)
        {
            retVal = (*iter)->planMotions(*attSupportPolygonMotion,*attFootMotion,attPlannedZMP, attSamplingPeriod);
            if (!retVal)
                return false;
            gapTime = (*(iter+1))->startTime() - (*iter)->endTime();
            if (gapTime>0.0)
            {
                retVal = prolongate(gapTime);
                if (!retVal)
                    return false;
            }
        }

        retVal = (*iter)->planMotions(*attSupportPolygonMotion,*attFootMotion,attPlannedZMP, attSamplingPeriod);
        if (!retVal)
            return false;
    }
    retVal = prolongate(attExtraEndTime);
    if (!retVal)
        return false;

    prolongateZMP(attExtraZMPEndTime);
    return retVal;
}


bool ChppGikLocomotionPlan::solveSecondStage(const matrixNxP& inObservedZMP)
{

    /*
        //previewTime of planned motion
        double prevT = attPreviewController->previewTime();
     
     
        //Compute Delta ZMP
        matrixNxP CorrectedZMP.resize(3,inObservedZMP.size2());
        CorrectedZMP = 2*subrange(attPlannedZMP,0,3,0,inObservedZMP.size2()) - inObservedZMP;
        
        //extend zmp (beginning and end)
        unsigned int newSize = inObservedZMP.size2()+2*attPreviewController->numberPreviewSamples();
        attPlannedZMP.resize(3,newSize);
        
        for (unsigned int i=0; i<attPreviewController->numberPreviewSamples;i++)
        {
            column(attPlannedZMP,i) = column(CorrectedZMP,0);
            column(attPlannedZMP,newSize-i-1) = column(CorrectedZMP,inObservedZMP.size2()-1);
        }
     
        //compute new COM
        matrixNxP resultTrajCOMXY;
        bool retVal = attPreviewController->ZMPtoCOM(attPlannedZMP,resultTrajCOMXY);
        if (!retVal)
            return false;
        
        //reset com motion constraint
        attComMotion->clear();
        attComMotion->startTime(attStartTime-2*attPreviewController->previewTime());
        
        //store com constraints
        ChppGikComConstraint* comC = new ChppGikComConstraint(*attRobot);
        for (unsigned int i=0; i<attFootMotion->numberStateConstraints(); i++)
        {
            comC->targetXY(resultTrajCOMXY(0,i), resultTrajCOMXY(1,i));
            attComMotion->pushbackStateConstraint(comC);
        }
        delete comC;
     
        //pad foot motion's beginning with start constraint
        for (unsigned int i=0; i<attPreviewController->numberPreviewSamples;i++)
        {
            attFootMotion->pushfrontStateConstraint( );
        }
     
     
        ChppGikTools::dumpMatrix("debug.zmpref2",  attPlannedZMP, moddedStartTime, attSamplingPeriod);
     
        
     
        */

    return false;
}


void ChppGikLocomotionPlan::addtoMotionPlan()
{
    unsigned int priority_0 = 0;
    unsigned int priority_1 = 1;
    attComMotionPlanRow =  attAssociatedMotionPlan->addMotionConstraint(attComMotion, priority_1);
    attFootMotionPlanRow =  attAssociatedMotionPlan->addMotionConstraint(attFootMotion, priority_0);
}

bool ChppGikLocomotionPlan::prolongate(double inDuration)
{
    const ChppGikSupportPolygon* lastSupportPolygon = attSupportPolygonMotion->lastSupportPolygon();
    if (!(lastSupportPolygon->isDoubleSupport()))
    {
        std::cout << "ChppGikLocomotionPlan::prolongate() Double support prerequisite not met.\n";
        return false;
    }

    unsigned int sizeNewChunk = (unsigned int)round(inDuration/attSamplingPeriod);
    unsigned int previousSize = attPlannedZMP.size2();
    unsigned int newSize =  previousSize + sizeNewChunk;

    //Last ZMP
    vectorN paddingZMP = ublas::column(attPlannedZMP,previousSize-1);
    attPlannedZMP.resize(3,newSize,true);
    //Last foot constraint
    CjrlGikStateConstraint* footConstraint = attFootMotion->stateConstraintAtRank(attFootMotion->numberStateConstraints()-1);
    //Fill motions
    for (unsigned int i=previousSize; i<newSize; i++)
    {
        ublas::column(attPlannedZMP,i) = paddingZMP;
        attFootMotion->pushbackStateConstraint(footConstraint);
    }
    //Extend support polygon motion's last element
    attSupportPolygonMotion->extendLastSupportPolygonDuration(inDuration);

    return true;
}

void ChppGikLocomotionPlan::prolongateZMP(double inDuration)
{

    unsigned int sizeNewChunk = (unsigned int)round(inDuration/attSamplingPeriod);
    unsigned int previousSize = attPlannedZMP.size2();
    unsigned int newSize =  previousSize + sizeNewChunk;

    //Last ZMP
    vectorN paddingZMP = ublas::column(attPlannedZMP,previousSize-1);
    attPlannedZMP.resize(3,newSize,true);

    //Fill motions
    for (unsigned int i=previousSize; i<newSize; i++)
        ublas::column(attPlannedZMP,i) = paddingZMP;
}

const matrixNxP& ChppGikLocomotionPlan::plannedZMPmotion()
{
    return attPlannedZMP;
}
