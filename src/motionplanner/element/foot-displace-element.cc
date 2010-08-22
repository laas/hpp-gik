#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "hpp/gik/motionplanner/element/foot-displace-element.hh"
#include "hpp/gik/tools.hh"

#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J
using namespace boost::numeric::ublas;

ChppGikFootDisplaceElement::ChppGikFootDisplaceElement(ChppGikStandingRobot* inSRobot, double inStartTime, const ChppGikFootprint * inTargetFootprint, bool isRight, double inDuration, double inSamplingPeriod, double inHeight):ChppGikLocomotionElement( inSRobot, inStartTime, inDuration, inSamplingPeriod)
{
    attTargetFootprint = new ChppGikFootprint(*inTargetFootprint);
    attIsRight = isRight;
    attHeight = inHeight;

    if (attDuration < 0.5)
        attDuration = 0.5;

    if (attIsRight)
    {
        attSupportFoot = attHumanoidRobot->leftAnkle();
        attConstrainedFoot = attHumanoidRobot->rightAnkle();
    }
    else
    {
        attSupportFoot = attHumanoidRobot->rightAnkle();
        attConstrainedFoot = attHumanoidRobot->leftAnkle();
    }

    vector3d zer;
    zer[0] = zer[1] = zer[2] = 0;
    matrix4d m;
    attConstraint = new ChppGikTransformationConstraint(*attRobot, *attConstrainedFoot, zer, m);
    attVectorizedTarget.resize(6);
    attMotionConstraint = this;
}

ChppGikFootDisplaceElement::~ChppGikFootDisplaceElement()
{
    delete attTargetFootprint;
    delete attConstraint;
}

bool ChppGikFootDisplaceElement::plan(ChppGikSupportPolygon& supportPolygon, vector3d& ZMP)
{

    attPlanSuccess = false;

    if (attIsRight)
    {
        if (!supportPolygon.isLeftLegSupporting())
        {
            std::cout << "ChppGikFootDisplaceElement::plan() bad initial Supoport Polygon\n";
            return false;
        }
/*
        if (!supportPolygon.leftFootprint()->isPointInsideSafeZone(ZMP[0], ZMP[1]))
        {
            std::cout << "ChppGikFootDisplaceElement::plan() bad initial ZMP\n";
            return false;
        }
        */
    }
    else
    {
        if (!supportPolygon.isRightLegSupporting())
        {
            std::cout << "ChppGikZMPshiftElement::plan() bad initial Supoport Polygon\n";
            return false;
        }
        /*
        if (!supportPolygon.rightFootprint()->isPointInsideSafeZone(ZMP[0], ZMP[1]))
        {
            std::cout << "ChppGikZMPshiftElement::plan() bad initial ZMP\n";
            return false;
        }
        */
    }

    unsigned int nsamples = ChppGikTools::timetoRank( attModifiedStart,attModifiedEnd,attSamplingPeriod)+1;
    attZMPmotion.resize(3,nsamples);
    for (unsigned int i=0; i<3;i++)
        row(attZMPmotion,i) = scalar_vector<double>(nsamples,ZMP[i]);
    supportPolygon.applyStep( attTargetFootprint, attIsRight );
    
    attPlanSuccess = true;
    return true;
}

bool ChppGikFootDisplaceElement::planFeet()
{
    attPlanSuccess = false;

    unsigned int nsamples = (unsigned int)round(attDuration/attSamplingPeriod)+1;

    matrix4d m  = attConstrainedFoot->currentTransformation();
    attConstraint->targetTransformation(m);
    attConstraint->computeVectorizedTarget();
    vectorN startXYZRPY = ((ChppGikVectorizableConstraint*)attConstraint)->vectorizedTarget();
    
	ChppGikSupportPolygon* cursp = attStandingRobot->supportPolygon();
	if (!cursp->isDoubleSupport())
	{
		std::cout << "No double support polygon found when planning a step" << std::endl;
		return false;
	}
	
	double rotangle;
	matrix4d curM;
	
	 if (attIsRight)
	{
        rotangle =  attTargetFootprint->th() - cursp->rightFootprint()->th();
		

        matrix4d rightfootT = attHumanoidRobot->rightAnkle()->currentTransformation();
        vector3d rffwd = rightfootT*attStandingRobot->rightFootLocalForwardVector();
        vector3d rfsd = rightfootT*attStandingRobot->rightFootLocalSideVector();
        vector3d rfup = rightfootT*attStandingRobot->rightFootLocalUpVector();
        for ( unsigned int i=0;i<3;i++ )
        {
            M4_IJ ( curM,i,0 ) = rffwd[i];
            M4_IJ ( curM,i,1 ) = rfsd[i];
            M4_IJ ( curM,i,2 ) = rfup[i];
            M4_IJ ( curM,i,3 ) = M4_IJ ( rightfootT,i,3 );
        }
	}
    else
	{
		matrix4d leftfootT = attHumanoidRobot->leftAnkle()->currentTransformation();
        vector3d lffwd = leftfootT*attStandingRobot->leftFootLocalForwardVector();
        vector3d lfsd = leftfootT*attStandingRobot->leftFootLocalSideVector();
        vector3d lfup = leftfootT*attStandingRobot->leftFootLocalUpVector();
        for ( unsigned int i=0;i<3;i++ )
        {
            M4_IJ ( curM,i,0 ) = lffwd[i];
            M4_IJ ( curM,i,1 ) = lfsd[i];
            M4_IJ ( curM,i,2 ) = lfup[i];
            M4_IJ ( curM,i,3 ) = M4_IJ ( leftfootT,i,3 );
        }
		rotangle =  attTargetFootprint->th() - cursp->leftFootprint()->th();
	}
	matrix4d prerot;
	M4_IJ(prerot, 0,0 ) = M4_IJ(prerot, 1,1 ) = cos (rotangle);
	M4_IJ(prerot, 1,0 ) = sin (rotangle);
	M4_IJ(prerot, 0,1 ) = -M4_IJ(prerot, 1,0 );
	
	matrix4d targetM;
	MAL_S4x4_C_eq_A_by_B(targetM,curM,prerot);
	M4_IJ(targetM, 0,3 ) = attTargetFootprint->x();
	M4_IJ(targetM, 1,3 ) = attTargetFootprint->y();
	M4_IJ(targetM, 2,3 ) = attTargetFootprint->th();
    
    attConstraint->targetTransformation(targetM);
    attConstraint->computeVectorizedTarget();
    vectorN endXYZRPY = ((ChppGikVectorizableConstraint*)attConstraint)->vectorizedTarget();

    matrixNxP data(6,nsamples);
    data.clear();
    vectorN tempInterpolationResult(nsamples);

    
    
    if (attSamplingPeriod <= 11e-3) // small enough
    {
        unsigned int numberZbufferSamples = 8; //arbitrary
        if (attSamplingPeriod <= 6e-3)
            numberZbufferSamples = 15; //arbitrary

        double flightTime = attDuration - numberZbufferSamples*attSamplingPeriod;
        unsigned int padStart = nsamples - numberZbufferSamples;//we are sure this is a strictly positive integer because we threshold the minimum stepping time in the ChppStep constructor
        matrixNxP vectomat(1,padStart);
        //interpolate
        for (unsigned int i = 0; i< 6 ; i++)
        {
            if (i==2)
                continue;

            attPlanSuccess = ChppGikTools::minJerkCurve(flightTime,attSamplingPeriod,startXYZRPY(i),0.0,0.0,endXYZRPY(i),tempInterpolationResult);
            if (!attPlanSuccess)
                return attPlanSuccess;
            row(vectomat,0) = subrange(tempInterpolationResult,0,padStart);
            subrange(data,i,i+1,0,padStart) = vectomat;

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

            attPlanSuccess = ChppGikTools::minJerkCurve(flightTime,attSamplingPeriod,startXYZRPY(i),0.0,0.0,endXYZRPY(i),tempInterpolationResult);
            if (!attPlanSuccess)
                return attPlanSuccess;
            row(data,i) = tempInterpolationResult;
        }
    }

    //now plan z
    double lowZ = startXYZRPY(2);
    double highZ = lowZ + attHeight;
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
    flightTime = attSamplingPeriod*(halfnsamples-1);
    tempInterpolationResult.resize(halfnsamples,false);
    matrixNxP vectomat(1,halfnsamples);
    //foot z up
    attPlanSuccess = ChppGikTools::minJerkCurve(flightTime,attSamplingPeriod,lowZ,0.0,0.0,highZ,tempInterpolationResult);
    
    if (!attPlanSuccess)
        return attPlanSuccess;
    
    row(vectomat,0) = tempInterpolationResult;
    subrange(data,2,3,0,halfnsamples) = vectomat;

    //foot z down
    ChppGikTools::minJerkCurve(flightTime,attSamplingPeriod,highZ,0.0,0.0,lowZ,tempInterpolationResult);
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

    ChppGikTools::prolongateTimeBased(attPreProlongation, attPostProlongation, attSamplingPeriod, data,  attFootMotion);
    
    return attPlanSuccess;
}

CjrlGikMotionConstraint* ChppGikFootDisplaceElement::clone() const
{
    ChppGikFootDisplaceElement* fde = new ChppGikFootDisplaceElement(attStandingRobot, attStartTime, attTargetFootprint, attIsRight, attDuration, attSamplingPeriod, attHeight);

    fde->postProlongate( attPostProlongation );
    fde->preProlongate( attPreProlongation );
    return fde;
}


CjrlGikStateConstraint* ChppGikFootDisplaceElement::stateConstraintAtTime(double inTime)
{
    if (!attPlanSuccess)
        return 0;

    unsigned int i = ChppGikTools::timetoRank(attModifiedStart,inTime,attSamplingPeriod);

    if (i == 1)
        if (!planFeet())
        {
            std::cout << "failed to plan feet in footdisplace element\n";
            return 0;
        }

    if ((i > 0) && (i < attFootMotion.size2()) )
    {
        attVectorizedTarget = column(attFootMotion,i);
        attConstraint->vectorizedTarget( attVectorizedTarget );
        return attConstraint;
    }
    else
        return 0;
}

ChppGikTransformationConstraint* ChppGikFootDisplaceElement::footConstraintAtTime ( double inTime )
{
    if (!attPlanSuccess)
        return 0;

    unsigned int i = ChppGikTools::timetoRank(attModifiedStart,inTime,attSamplingPeriod);

    if (i == 1)
        if (!planFeet())
    {
        std::cout << "failed to plan feet in footdisplace element\n";
        return 0;
    }

    if ((i > 0) && (i < attFootMotion.size2()) )
    {
        attVectorizedTarget = column(attFootMotion,i);
        attConstraint->vectorizedTarget( attVectorizedTarget );
        return attConstraint;
    }
    else
        return 0;
}

CjrlJoint* ChppGikFootDisplaceElement::supportFootAtTime(double inTime)
{
    if (!attPlanSuccess)
        return 0;
    if ((inTime > attModifiedStart + attEps) && (inTime < attModifiedEnd + attEps))
        return attSupportFoot;
    return 0;
}
