#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "tasks/hppGikGenericTaskElements.h"
#include "hppGikTools.h"


using namespace ublas;


ChppGikPrioritizedStateConstraint::ChppGikPrioritizedStateConstraint(CjrlGikStateConstraint* inStateConstraint, unsigned int inPriority)
{
    attStateConstraint = inStateConstraint;
    attPriority = inPriority;
}
CjrlGikStateConstraint* ChppGikPrioritizedStateConstraint::stateConstraint()
{
    return attStateConstraint;
}
unsigned int ChppGikPrioritizedStateConstraint::priority()
{
    return attPriority;
}
ChppGikPrioritizedStateConstraint::~ChppGikPrioritizedStateConstraint()
{}
/****************************************************************/
ChppGikPrioritizedMotionConstraint::ChppGikPrioritizedMotionConstraint(CjrlGikMotionConstraint* inMotionConstraint, unsigned int inPriority)
{
    attMotionConstraint = inMotionConstraint;
    attPriority = inPriority;
}
CjrlGikMotionConstraint* ChppGikPrioritizedMotionConstraint::motionConstraint()
{
    return attMotionConstraint;
}
unsigned int ChppGikPrioritizedMotionConstraint::priority()
{
    return attPriority;
}
ChppGikPrioritizedMotionConstraint::~ChppGikPrioritizedMotionConstraint()
{}
/****************************************************************/
ChppGikSingleMotionElement::ChppGikSingleMotionElement(ChppGikSingleMotionElementConstraint* inTargetConstraint, unsigned int inPriority, double inStartTime, double inDuration)
{
    attPriority = inPriority+1; //(+1 to avoid confict with the locomotion motions)
    attStartTime = inStartTime;
    attDuration = inDuration;
    attEndTime = inDuration + inStartTime;
    attTargetConstraint = dynamic_cast<ChppGikSingleMotionElementConstraint*>(inTargetConstraint->clone());
    attPlannedMotion = 0;
    attMotionRow = 0;
}

unsigned int ChppGikSingleMotionElement::priority ()
{
    return attPriority;
}

double ChppGikSingleMotionElement::startTime()
{
    return attStartTime;
}

void ChppGikSingleMotionElement::startTime(double inStartTime)
{
    attStartTime = inStartTime;
    attDuration = attEndTime - attStartTime;
    if (attDuration <= 0)
        attDuration = 0;
}

void ChppGikSingleMotionElement::duration(double inDuration)
{
    attEndTime = attStartTime+inDuration;
    attDuration = inDuration;
}

double ChppGikSingleMotionElement::duration()
{
    return attDuration;
}

ChppGikSingleMotionElementConstraint* ChppGikSingleMotionElement::targetConstraint()
{
    return attTargetConstraint;
}

ChppGikSingleMotionElement::~ChppGikSingleMotionElement()
{
    dereferMotion();
    delete attTargetConstraint;
    delete attPlannedMotion;
}

ChppGikMotionConstraint* ChppGikSingleMotionElement::motion()
{
    return attPlannedMotion;
}

bool ChppGikSingleMotionElement::planMotion(double inSamplingPeriod)
{
    if (duration()<inSamplingPeriod)
        return false;

    dereferMotion();
    
    delete attPlannedMotion;
    attPlannedMotion = 0;
    attPlannedMotion = new ChppGikMotionConstraint(inSamplingPeriod, attStartTime);

    return attTargetConstraint->minimumJerkInterpolation(attPlannedMotion, inSamplingPeriod, attDuration);

}

bool ChppGikSingleMotionElement::referMotion(ChppGikMotionPlan* inMotionPlan)
{
    if (!attMotionRow && attPlannedMotion)
    {
        attMotionRow = inMotionPlan->addMotionConstraint( attPlannedMotion, attPriority);
        return true;
    }
    else
        return false;
}

bool ChppGikSingleMotionElement::dereferMotion()
{
    bool ret=false;
    if (attMotionRow)
        ret = attMotionRow->removeMotionConstraint(attPlannedMotion);
    attMotionRow = 0;
    return ret;

}
/**********************************************************/

ChppGikZMPshiftElement::ChppGikZMPshiftElement( double inNewZMPx, double inNewZMPy, double inStartTime, double inDuration)
{
    attTargetZMP.resize(3,false);
    attTargetZMP(0) = inNewZMPx;
    attTargetZMP(1) = inNewZMPy;
    attTargetZMP(2) = 0.0;
    attStartTime = inStartTime;
    attDuration = inDuration;
}

double ChppGikZMPshiftElement::startTime()
{
    return attStartTime;
}

double ChppGikZMPshiftElement::duration()
{
    return attDuration;
}

ChppGikZMPshiftElement::~ChppGikZMPshiftElement()
{}

bool ChppGikZMPshiftElement::planMotions(ChppGikSupportPolygonMotion& outSupportPolygonMotion, ChppGikMotionConstraint& outFootMotion, matrixNxP& outZMPmotion, double inSamplingPeriod)
{
    if (outSupportPolygonMotion.empty() || outZMPmotion.size2() ==0 || outFootMotion.empty())
    {
        std::cout << "ChppGikFootDisplaceElement::planMotions() at least one element has to populate every argument motion\n";
        return false;
    }

    const ChppGikSupportPolygon* lastSupportPolygon = outSupportPolygonMotion.lastSupportPolygon();
    if (!(lastSupportPolygon->isDoubleSupport()))
    {
        std::cout << "ChppGikLocomotionPlan::shiftZMP Double support prerequisite not met.\n";
        return false;
    }
    //std::cout <<  attDuration << std::endl;
    //ZMP interpolation
    unsigned int sizeNewChunk = (unsigned int)round(attDuration/inSamplingPeriod);
    unsigned int previousSize = outZMPmotion.size2();
    unsigned int newSize =  previousSize + sizeNewChunk;

    outZMPmotion.resize(3,newSize,true);

    vectorN lastZMP = column(outZMPmotion,previousSize-1);


    /*

    // Version of minimum jerk ZMP interpolation //
    matrixNxP tempZMPTraj;
    tempZMPTraj.resize(2,sizeNewChunk,false);
    vectorN tempInterpolationResult;
    tempInterpolationResult.resize(sizeNewChunk+1,false);

    for (unsigned int i = 0; i<2; i++)
    {
        bool OK = ChppGikTools::minJerkCurve(attDuration,inSamplingPeriod,lastZMP(i),0,0,attTargetZMP(i),tempInterpolationResult);
        row(tempZMPTraj,i) = subrange(tempInterpolationResult,1,sizeNewChunk+1);
    }
    subrange(outZMPmotion,0,2,previousSize, newSize) = tempZMPTraj;

    */

    ///*
    matrixNxP interpolationResult;
    ChppGikTools::multiLinearInterpolation( attDuration, inSamplingPeriod, lastZMP, attTargetZMP, interpolationResult);
    subrange(outZMPmotion,0,3,previousSize, newSize) = subrange(interpolationResult,0,3,1,sizeNewChunk+1);
    //*/



    //Fill in foot motion constraint
    // get last foot constraint
    CjrlGikStateConstraint* footConstraint = outFootMotion.stateConstraintAtRank(outFootMotion.numberStateConstraints()-1);
    for (unsigned int i=previousSize; i<newSize; i++)
        outFootMotion.pushbackStateConstraint(footConstraint);

    //Add unchenged interval of support polygon to the support polygon motion object
    outSupportPolygonMotion.extendLastSupportPolygonDuration(attDuration);

    return true;
}


/**********************************************************/
/**********************************************************/

ChppGikFootDisplaceElement::ChppGikFootDisplaceElement(double inStartTime, ChppGikFootprint* inTargetFootprint, bool inIsRight, double inDuration, double inHeight)
{
    attTargetFootprint = new ChppGikFootprint(*inTargetFootprint);
    attIsRight = inIsRight;
    attDuration = inDuration;
    attHeight = inHeight;
    //threshold stepping motion time
    if (attDuration < 0.5)
        attDuration = 0.5;
    tempRot.resize(3,3,false);
    attStartTime = inStartTime;
}

bool ChppGikFootDisplaceElement::isRight() const
{
    return attIsRight;
}

void ChppGikFootDisplaceElement::selectFoot(bool inRight)
{
    attIsRight = inRight;
}


double ChppGikFootDisplaceElement::startTime()
{
    return attStartTime;
}

double ChppGikFootDisplaceElement::duration()
{
    return attDuration;
}

void ChppGikFootDisplaceElement::duration(double inTime)
{
    attDuration = inTime;
}

double ChppGikFootDisplaceElement::height() const
{
    return attHeight;
}

void ChppGikFootDisplaceElement::height(double inHeight)
{
    attHeight = inHeight;
}

const ChppGikFootprint& ChppGikFootDisplaceElement::targetFootprint() const
{
    return *attTargetFootprint;
}

void ChppGikFootDisplaceElement::targetFootprint(ChppGikFootprint* inTarget)
{
    attTargetFootprint->x(inTarget->x());
    attTargetFootprint->y(inTarget->y());
    attTargetFootprint->th(inTarget->th());
}

bool ChppGikFootDisplaceElement::planMotions(ChppGikSupportPolygonMotion& outSupportPolygonMotion, ChppGikMotionConstraint& outFootMotion, matrixNxP& outZMPmotion, double inSamplingPeriod)
{
    if (outSupportPolygonMotion.empty() || outZMPmotion.size2() ==0 || outFootMotion.empty())
    {
        std::cout << "ChppGikFootDisplaceElement::planMotions() at least one element has to populate every argument motion\n";
        return false;
    }
    //std::cout <<  attDuration << std::endl;
    unsigned int sizeNewChunk = (unsigned int)round(attDuration/inSamplingPeriod);
    unsigned int previousSize = outZMPmotion.size2();
    unsigned int newSize =  previousSize + sizeNewChunk;

    //determin support and non support feet related variables
    const ChppGikSupportPolygon *lastSupportPolygon= outSupportPolygonMotion.lastSupportPolygon();
    ChppGikSupportPolygon *newSingleSupportPolygon;
    ChppGikSupportPolygon *newDoubleSupportPolygon;
    CjrlJoint* supportFootJoint, *nonsupportFootJoint;
    const ChppGikFootprint *nonsupportFootprint, *supportFootprint;

    //const ChppGikFootprint* targetFootprint = &(attTargetFootprint);

    ChppGikTransformationConstraint* footConstraint = dynamic_cast<ChppGikTransformationConstraint*>( outFootMotion.stateConstraintAtRank(0)->clone());

    CjrlHumanoidDynamicRobot& robot = footConstraint->robot();
    
    matrix4d startFootTransformation;
    matrix4d endFootTransformation;
    matrixNxP uH(4,4);
    uH.clear();
    uH(3,3)=1;
    matrixNxP uR(3,3);
    ChppGikTools::RotFromYaw(attTargetFootprint->th(),uR);
    subrange(uH,0,3,0,3) = uR;
    uH(0,3) = attTargetFootprint->x();
    uH(1,3) = attTargetFootprint->y();
    uH(2,3) = robot.footHeight();
    ChppGikTools::UblastoMatrix4(uH,endFootTransformation);
    
    if (attIsRight)
    {
        nonsupportFootprint = lastSupportPolygon->rightFootprint();
        supportFootprint = lastSupportPolygon->leftFootprint();
        supportFootJoint = robot.leftFoot();
        nonsupportFootJoint = robot.rightFoot();
        newDoubleSupportPolygon = new ChppGikSupportPolygon(*supportFootprint, *attTargetFootprint);
        newDoubleSupportPolygon->lfootTransformation(*(lastSupportPolygon->lfootTransformation()));
        newDoubleSupportPolygon->rfootTransformation(endFootTransformation);
        newSingleSupportPolygon = new ChppGikSupportPolygon(*supportFootprint,!attIsRight);
        newSingleSupportPolygon->lfootTransformation(*(lastSupportPolygon->lfootTransformation()));
        startFootTransformation = *(lastSupportPolygon->rfootTransformation());
    }
    else
    {
        nonsupportFootprint = lastSupportPolygon->leftFootprint();
        supportFootprint = lastSupportPolygon->rightFootprint();
        supportFootJoint = robot.rightFoot();
        nonsupportFootJoint = robot.leftFoot();
        newDoubleSupportPolygon = new ChppGikSupportPolygon(*attTargetFootprint, *supportFootprint);
        newDoubleSupportPolygon->rfootTransformation(*(lastSupportPolygon->rfootTransformation()));
        newDoubleSupportPolygon->lfootTransformation(endFootTransformation);
        newSingleSupportPolygon = new ChppGikSupportPolygon(*supportFootprint,attIsRight);
        newSingleSupportPolygon->rfootTransformation(*(lastSupportPolygon->rfootTransformation()));
        startFootTransformation = *(lastSupportPolygon->lfootTransformation());
    }

    //Update support polygon motion object
    outSupportPolygonMotion.pushbackSupportPolygon(newSingleSupportPolygon, attDuration);
    outSupportPolygonMotion.pushbackSupportPolygon(newDoubleSupportPolygon, 0.0);
    delete newSingleSupportPolygon;
    delete newDoubleSupportPolygon;
    
    //We plan the foot motion so that it reaches the desired x y and yaw before it touches the ground by numberZbufferSamples*inSamplingPeriod seconds
    //interpolation related variables
    unsigned int nsamples = sizeNewChunk+1;
    matrixNxP data(6,nsamples);
    vectorN tempInterpolationResult(nsamples);

    vectorN tempEulerr(3);
    vectorN tempTrans(3);
    matrixNxP tempRota(3,3);
    ChppGikTools::HtoRT(startFootTransformation,tempRota,tempTrans);
    ChppGikTools::RottoEulerZYX(tempRota,tempEulerr);

    vectorN startXYZRPY(6);
    vectorN endXYZRPY(6);
    subrange(startXYZRPY,0,2) = subrange(tempTrans,0,2);
    subrange(startXYZRPY,3,6) = tempEulerr;
    endXYZRPY(0) = attTargetFootprint->x();
    endXYZRPY(1) = attTargetFootprint->y();
    endXYZRPY(2) = 0.0;
    endXYZRPY(3) = 0.0;
    endXYZRPY(4) = 0.0;
    endXYZRPY(5) = attTargetFootprint->th();

    if (inSamplingPeriod <= 6e-3) // small enough
    {
        unsigned int numberZbufferSamples = 15; //arbitrary
        double flightTime = attDuration - numberZbufferSamples*inSamplingPeriod;
        unsigned int padStart = nsamples - numberZbufferSamples;//we are sure this is a strictly positive integer because we threshold the minimum stepping time in the ChppStep constructor
        matrixNxP vectomat(1,padStart);
        //interpolate
        for (unsigned int i = 0; i< 6 ; i++)
        {
            if (i==2)
                continue;
            
            bool contn = ChppGikTools::minJerkCurve(flightTime,inSamplingPeriod,startXYZRPY(i),0.0,0.0,endXYZRPY(i),tempInterpolationResult);

            row(vectomat,0) = subrange(tempInterpolationResult,0,padStart);
            subrange(data,i,i+1,0,padStart) = vectomat;

            if (!contn)
                return false;
        }
        //pad last Samples  with the reached final values
        vectorN endValues = column(data,padStart-1);
        for (unsigned int i = padStart; i< nsamples ; i++)
            column(data,i) = endValues;

    }
    else // the sampling period is too gross, (might be for debugging, so don't do the previous trick)
    {
        double flightTime = attDuration;
        for (unsigned int i = 0; i< 6 ; i++)
        {
            if (i==2)
                continue;
            
            bool contn = ChppGikTools::minJerkCurve(flightTime,inSamplingPeriod,startXYZRPY(i),0.0,0.0,endXYZRPY(i),tempInterpolationResult);
            row(data,i) = tempInterpolationResult;
            if (!contn)
                return false;
        }
    }

    //now plan z
    double lowZ = tempTrans(2);// robot.footHeight();
    double highZ = lowZ+attHeight;
    bool simpleConcat;
    unsigned int halfnsamples;
    double flightTime;
    if (nsamples % 2 == 0)
    {
        simpleConcat = true;
        halfnsamples = nsamples/2;
    }
    else
    {
        simpleConcat = false;
        halfnsamples = (nsamples+1)/2;
    }
    flightTime = inSamplingPeriod*(halfnsamples-1);
    tempInterpolationResult.resize(halfnsamples,false);
    matrixNxP vectomat(1,halfnsamples);
    //foot z up
    bool ok = ChppGikTools::minJerkCurve(flightTime,inSamplingPeriod,lowZ,0.0,0.0,highZ,tempInterpolationResult);
    row(vectomat,0) = tempInterpolationResult;
    subrange(data,2,3,0,halfnsamples) = vectomat;
    if (!ok)
        return false;

    //foot z down
    ChppGikTools::minJerkCurve(flightTime,inSamplingPeriod,highZ,0.0,0.0,lowZ,tempInterpolationResult);
    if (simpleConcat)
    {
        row(vectomat,0) = tempInterpolationResult;
        subrange(data,2,3,halfnsamples,nsamples) = vectomat;
    }
    else
    {
        row(vectomat,0) = tempInterpolationResult;
        subrange(data,2,3,halfnsamples-1,nsamples) = vectomat;
    }

    //End of interpolation

    //Storing new planned data
    //Foot motion constaint
    footConstraint->joint( nonsupportFootJoint );
    vectorN vecXYZRPY(6);
    for (unsigned int i = 1; i< nsamples; i++)
    {
        vecXYZRPY = column(data,i);
        fillFootConstraint(footConstraint, vecXYZRPY);
        outFootMotion.pushbackStateConstraint((CjrlGikTransformationConstraint*)footConstraint);
    }
    delete footConstraint;
    
    

    outZMPmotion.resize(3,newSize,true);

    vectorN paddingZMP = column(outZMPmotion,previousSize-1);
    for (unsigned int i = previousSize; i< newSize; i++)
        column(outZMPmotion,i)= paddingZMP;

    return true;
}

void ChppGikFootDisplaceElement::fillFootConstraint( ChppGikTransformationConstraint* inConstraint, vectorN& inVectorXYZRPY)
{
    if (inVectorXYZRPY.size() !=6)
        std::cout << "ChppGikLocomotionPlan::fillFootConstraint(): Shouldn't happen\n";

    vectorN rpy = ublas::subrange(inVectorXYZRPY,3,6);
    ChppGikTools::EulerZYXtoRot(rpy, tempRot);

    vectorN xyz = ublas::subrange(inVectorXYZRPY,0,3);

    inConstraint->worldTargetU(xyz);
    inConstraint->targetOrientationU(tempRot);
}

ChppGikFootDisplaceElement::~ChppGikFootDisplaceElement()
{
    delete attTargetFootprint;
}


/**********************************************************/
/**********************************************************/
ChppGikStepElement::ChppGikStepElement(double inStartTime, ChppGikFootprint* inTargetFootprint, bool isRightFoot, double inFinalZMPCoefficient, double inEndShiftTime, double inStartZMPShiftTime, double inFootMotionDuration, double inStepHeight):ChppGikFootDisplaceElement(inStartTime, inTargetFootprint, isRightFoot, inFootMotionDuration, inStepHeight)
{
    attStartShiftTime = inStartZMPShiftTime;
    attEndShiftTime = inEndShiftTime;
    attFinalZMPCoef = inFinalZMPCoefficient;
    //thresholding endZMPcoef
    if (attFinalZMPCoef > 1)
        attFinalZMPCoef = 1;
    if (attFinalZMPCoef < 0)
        attFinalZMPCoef = 0;
    attUseZMPcoefficient = true;
}

ChppGikStepElement::ChppGikStepElement(ChppGikFootprint* inTargetFootprint, double inStartTime, bool isRightFoot, double rightfoot2TargetZMPX, double rightfoot2TargetZMPY, double inEndShiftTime, double inStartZMPShiftTime, double inFootMotionDuration, double inStepHeight):ChppGikFootDisplaceElement(inStartTime, inTargetFootprint, isRightFoot, inFootMotionDuration, inStepHeight)
{
    attUseZMPcoefficient = false;
    attStartShiftTime = inStartZMPShiftTime;
    attEndShiftTime = inEndShiftTime;
    attRfoot2TargetZMPX = rightfoot2TargetZMPX;
    attRfoot2TargetZMPY = rightfoot2TargetZMPY;
}

double ChppGikStepElement::startShiftTime() const
{
    return attStartShiftTime;
}

void ChppGikStepElement::startShiftTime(double inTime)
{
    attStartShiftTime = inTime;
}

double ChppGikStepElement::endShiftTime() const
{
    return attEndShiftTime;
}

void ChppGikStepElement::endShiftTime(double inTime)
{
    attEndShiftTime = inTime;
}

double ChppGikStepElement::finalZMPCoef() const
{
    return attFinalZMPCoef;
}

void ChppGikStepElement::finalZMPCoef(double inCoef)
{
    attFinalZMPCoef = inCoef;
}

bool ChppGikStepElement::planMotions(ChppGikSupportPolygonMotion& outSupportPolygonMotion, ChppGikMotionConstraint& outFootMotion, matrixNxP& outZMPmotion, double inSamplingPeriod)
{
    if (outSupportPolygonMotion.empty() || outZMPmotion.size2() ==0 || outFootMotion.empty())
    {
        std::cout << "ChppGikFootDisplaceElement::planMotions() at least one element has to populate every argument motion\n";
        return false;
    }

    const ChppGikSupportPolygon* lastSupportPolygon = outSupportPolygonMotion.lastSupportPolygon();
    if (!(lastSupportPolygon->isDoubleSupport()))
    {
        std::cout << "ChppGikStepElement::planMotions() Double support prerequisite not met.\n";
        return false;
    }

    bool checkOK;
    double targetZMPx, targetZMPy;

    //determin support footprint
    const ChppGikFootprint *supportFootprint;
    if (attIsRight)
        supportFootprint = lastSupportPolygon->leftFootprint();
    else
        supportFootprint = lastSupportPolygon->rightFootprint();

    //Processing the entered ChppStep object
    //Phase1: bring zmp under support foot if required
    if (attStartShiftTime > 0)
    {

        targetZMPx = supportFootprint->x();
        targetZMPy = supportFootprint->y();

        ChppGikZMPshiftElement* startZMPshiftTask = new ChppGikZMPshiftElement(targetZMPx, targetZMPy, attStartTime, attStartShiftTime);
        checkOK = startZMPshiftTask->planMotions(outSupportPolygonMotion, outFootMotion, outZMPmotion, inSamplingPeriod);
        
        delete startZMPshiftTask;
    }
    if (!checkOK)
    {
        std::cout << "ChppGikStepElement::planMotions() Failed at phase 1\n";
        return false;
    }

    //Phase2: stepping
    ChppGikFootDisplaceElement* footDisplaceTask = new ChppGikFootDisplaceElement(attStartTime + attStartShiftTime, attTargetFootprint, attIsRight, attDuration, attHeight);
    checkOK = footDisplaceTask->planMotions(outSupportPolygonMotion, outFootMotion, outZMPmotion, inSamplingPeriod);

    delete footDisplaceTask;

    if (!checkOK)
    {
        std::cout << "ChppGikStepElement::planMotions() Failed at phase 2\n";
        return false;
    }

    //Phase3: bring zmp between support foot and moved foot if required
    if (attEndShiftTime > 0)
    {
        if (attUseZMPcoefficient)
        {
            targetZMPx = (1 -attFinalZMPCoef)*supportFootprint->x() + attFinalZMPCoef*attTargetFootprint->x();
            targetZMPy =  (1 -attFinalZMPCoef)*supportFootprint->y() + attFinalZMPCoef*attTargetFootprint->y();
        }
        else
        {
            lastSupportPolygon = outSupportPolygonMotion.lastSupportPolygon();
            const ChppGikFootprint* rfp = lastSupportPolygon->rightFootprint();
            
            targetZMPx = rfp->x() + cos(rfp->th())*attRfoot2TargetZMPX - sin(rfp->th())*attRfoot2TargetZMPY;
            
            targetZMPy = rfp->y() + sin(rfp->th())*attRfoot2TargetZMPX + cos(rfp->th())*attRfoot2TargetZMPY;
            
            if (!lastSupportPolygon->isPointInside( targetZMPx, targetZMPy)) 
            {
                std::cout << "Error: tried to plan a zmp out of the support polygon\n";
                std::cout << "Target ZMP: " << targetZMPx <<" , "<< targetZMPy <<" \n";
                lastSupportPolygon->print();
                return false;
            }
        }

        ChppGikZMPshiftElement* endZMPshiftTask = new ChppGikZMPshiftElement(targetZMPx, targetZMPy, attStartTime + attStartShiftTime + attDuration, attEndShiftTime);
        checkOK = endZMPshiftTask->planMotions(outSupportPolygonMotion, outFootMotion, outZMPmotion, inSamplingPeriod);
        
        delete endZMPshiftTask;
    }

    if (!checkOK)
    {
        std::cout << "ChppGikStepElement::planMotions() Failed at phase 3\n";
        return false;
    }

    return true;
}

double ChppGikStepElement::startTime()
{
    return attStartTime;
}

double ChppGikStepElement::duration()
{
    return attEndShiftTime + attStartShiftTime + attDuration;
}

ChppGikStepElement::~ChppGikStepElement()
{}





