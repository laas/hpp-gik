
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "motionplanners/hppGikLocomotionPlanner.h"
#include "hppGikTools.h"

#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J
#define M3_IJ MAL_S3x3_MATRIX_ACCESS_I_J

using namespace ublas;

ChppGikLocomotionPlanner::ChppGikLocomotionPlanner(ChppGikStandingRobot* inStandingRobot)
{
    attStandingRobot = inStandingRobot;
    attRobot = attStandingRobot->robot();
    attGikSolver = new ChppGikSolver(attStandingRobot->robot());
    attGikMaxIter = 5;
    attSolveDefaultMaxIter = 100;
    attNvalthresh = 1e-5;

    attMinDist = 0.18;
    attMaxDist = 0.30;
    attMaxComDist = 0.05;
    attMinRelLeftY = 0.05;

    attGikValueStep = 0.05;
    attMinValProgress = 1e-6;
    attMinWaistProgress = 1e-3;

    attMinRelTheta = -M_PI/24;
    attMaxRelTheta = M_PI/3;//M_PI/3;

    attMinPosTheta = M_PI/4;
    attMaxPosTheta = 3*M_PI/4;

    attCheckForPos = true;
    attCheckForRot = true;
    attCheckForDist = true;

    attDetailedMotion = new ChppRobotMotion(inStandingRobot->robot(), 0.0, 1);
    attRefinedMotion = new ChppRobotMotion(inStandingRobot->robot(), 0.0, 1);

    vectorN activated =  attStandingRobot->maskFactory()->wholeBodyMask();
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    vectorN attWeightedWholeBody = weights;

    for (unsigned int i=0;i<attWeightedWholeBody.size();i++)
        attWeightedWholeBody(i) *= activated(i);

    attWeights = attStandingRobot->maskFactory()->legsMask();
    //attWeights =  attStandingRobot->maskFactory()->wholeBodyMask();

    matrix4d m;
    vector3d lp, pn, pp;
    lp[0] = lp[1] = lp[2] = 0;
    pp = lp;
    pp[2] = attRobot->footHeight();
    pn = lp;
    pn[2] = 1;

    attFootParallel = new ChppGikParallelConstraint(*attRobot, *(attRobot->leftFoot() ), pn, pn );
    attFootPlane = new ChppGikPlaneConstraint(*attRobot, *(attRobot->leftFoot() ), lp, pp, pn );

    attFootPosition = new ChppGikPositionConstraint(*attRobot, *(attRobot->leftFoot() ), lp, lp );

    attFootTransformation = new ChppGikTransformationConstraint(*attRobot, *(attRobot->leftFoot() ), lp, m );

    matrix3d rm;
    attFootRot = new ChppGikRotationConstraint(*attRobot, *(attRobot->leftFoot()), rm );

    attWaistPosition = new ChppGikPositionConstraint(*attRobot, *(attRobot->waist() ), lp, lp );
    pp[2] = attStandingRobot->halfsittingWaistHeight();

    attWaistPlane = new ChppGikPlaneConstraint(*attRobot, *(attRobot->waist() ), lp, pp, pn );

    attWaistParallel = new ChppGikParallelConstraint(*attRobot, *(attRobot->waist() ), pn, pn );

    attWaistTransformation = new ChppGikTransformationConstraint(*attRobot, *(attRobot->waist() ), lp, m );

    pn[0] = 0.0;
    pn[1] = 1.0;
    pn[2] = 0.0;

    pp[0] = 0.0;
    pp[1] = 0.0;
    pp[2] = attStandingRobot->halfsittingWaistHeight();

    attWaistPointing = new ChppGikPointingConstraint(*attRobot, *(attRobot->waist()), lp, pn, pp );

    attComConstraint = new ChppGikComConstraint(*attRobot, 0.0, 0.0);

    attLPConstraint = new ChppGikLPConstraint(*attRobot);

    attLP2Constraint = new ChppGikLP2Constraint(*attRobot);

    attFinalConfiguration.resize(attRobot->numberDof(), false);

    //Arbitrary
    supportFoot(attRobot->rightFoot());

    attLocomotionConstraints.push_back(attLPConstraint);
    attLocomotionConstraints.push_back(attLP2Constraint);
    attLocomotionConstraints.push_back(attWaistParallel);
    attLocomotionConstraints.push_back(attWaistPlane);
    attLocomotionConstraints.push_back(attFootParallel);
    attLocomotionConstraints.push_back(attFootPlane);

    attJrlWatched.push_back(attLPConstraint);
    attJrlWatched.push_back(attLP2Constraint);
    attJrlWatched.push_back(attWaistParallel);
    attJrlWatched.push_back(attWaistPlane);
    attJrlWatched.push_back(attFootParallel);
    attJrlWatched.push_back(attFootPlane);

    attDeltaWaistState.resize(6,false);
    attPreviousConfiguration.resize(attRobot->numberDof(),false);
}

ChppGikLocomotionPlanner::~ChppGikLocomotionPlanner()
{
    clearConstraints();
    clearFeet();

    delete attGikSolver;
    delete attDetailedMotion, attRefinedMotion;
    delete attFootParallel, attFootPlane, attFootTransformation, attFootPosition, attFootRot;
    delete attWaistPosition, attWaistParallel, attWaistPlane, attWaistTransformation, attWaistPointing;
    delete attComConstraint, attLPConstraint, attLP2Constraint;
}


bool ChppGikLocomotionPlanner::automaticSupportFoot()
{
    if (attConstraints.empty())
        return false;

    vectorN bConf = attRobot->currentConfiguration();
    supportFoot(attRobot->rightFoot());//right
    attDeltaWaistState.clear();
    bool ls = littleSolve(false);
    double normR = norm_2(attDeltaWaistState);

    attRobot->applyConfiguration( bConf );
    supportFoot(attRobot->leftFoot());//left
    attDeltaWaistState.clear();
    ls = littleSolve(false);
    double normL = norm_2(attDeltaWaistState);

    if (normL > normR)
    {
        attSupportFoot = attRobot->rightFoot();
        attNonSupportFoot = attRobot->leftFoot();
    }
    else
    {
        attSupportFoot = attRobot->leftFoot();
        attNonSupportFoot = attRobot->rightFoot();
    }

    return true;
}

void ChppGikLocomotionPlanner::supportFoot(const CjrlJoint* inJoint)
{
    if (inJoint == attRobot->rightFoot())
    {
        attNonSupportFoot = attRobot->leftFoot();
        attSupportFoot = attRobot->rightFoot();
    }
    else
    {
        attNonSupportFoot = attRobot->rightFoot();
        attSupportFoot = attRobot->leftFoot();
    }

    updateSupportFootRelated();

}

void ChppGikLocomotionPlanner::updateSupportFootRelated()
{
    attRobot->clearFixedJoints();
    attRobot->addFixedJoint(attSupportFoot);

    attFootParallel->joint( attNonSupportFoot );
    attFootPosition->joint( attNonSupportFoot );
    attFootPlane->joint( attNonSupportFoot );
    attFootRot->joint(attNonSupportFoot);
    attFootTransformation->joint( attNonSupportFoot );
    attLPConstraint->movingFoot( attNonSupportFoot );
    attLP2Constraint->movingFoot( attNonSupportFoot );
}

void ChppGikLocomotionPlanner::switchFeetRoles()
{
    //modifies attSupportFoot, attNonSupportFoot and all depending constraints
    attRobot->applyConfiguration( attPreviousConfiguration );

    //exchange feet roles and try again
    attDummyFoot = attSupportFoot;
    attSupportFoot = attNonSupportFoot;
    attNonSupportFoot = attDummyFoot;

    updateSupportFootRelated();

    std::cout << "Changing foot\n";
}

bool ChppGikLocomotionPlanner::solveConstraints()
{
    attJrlAll.clear();
    for (unsigned int i = 0; i<attLocomotionConstraints.size(); i++)
        attJrlAll.push_back(attLocomotionConstraints[i]);

    appendClampedTargets( attJrlAll, attGikValueStep);

    recordCurrentValues();

    return solveGik(attJrlAll, attJrlWatched, false);
}

void ChppGikLocomotionPlanner::recordCurrentValues()
{
    attLastValues.clear();

    for (unsigned int wc =0; wc<attConstraints.size();wc++)
    {
        attConstraints[wc]->computeValue();
        attLastValues.push_back( norm_2(attConstraints[wc]->value()) );
    }

}

void ChppGikLocomotionPlanner::computeProgress()
{

    attDeltaValues.clear();
    for (unsigned int wc =0; wc<attConstraints.size();wc++)
    {
        attConstraints[wc]->computeValue();
        attDeltaValues.push_back(attLastValues[wc] - norm_2(attConstraints[wc]->value()) );
    }

    attDeltaWaistState = subrange(attRobot->currentConfiguration(),0,6) - subrange(attPreviousConfiguration,0,6);
}

bool ChppGikLocomotionPlanner::anyProgress()
{
    for (unsigned int wc =0; wc<attDeltaValues.size();wc++)
    {
        //std::cout << attDeltaValues[wc] << "\n";
        if (attDeltaValues[wc] > attMinValProgress)
            return true;
    }

    /*
    if (norm_2(attDeltaWaistState) > attMinWaistProgress)
        return true;
    */
    return false;
}

bool ChppGikLocomotionPlanner::littleSolve(bool allowFootChange)
{

    bool result = false;
    unsigned int footChanges = 0;
    bool changeFoot;
    
    attPreviousConfiguration = attRobot->currentConfiguration();
    while ((result == false) && footChanges<2)
    {
        changeFoot = false;

        //recordCurrentValues();
        if (solveConstraints())
        {
            if (enforceFeetDomain())
            {
                if (1/*comMovable()*/)
                {
                    computeProgress();
                    
                    attPreviousConfiguration = attRobot->currentConfiguration();
                    
                    if (anyProgress())
                        result = true;
                    else
                    {
                        std::cout << "No progress\n";
                        changeFoot = true;
                    }
                }
                else
                {
                    std::cout << "COM not reaching\n";
                    changeFoot = true;
                }
            }
            else
            {
                std::cout << "Could not enforce feet boundaries\n";
                changeFoot = true;
            }
        }
        else
        {
            std::cout << "Could not ensure waist and foot constraints\n";
            changeFoot = true;
        }

        if (changeFoot)
        {
            switchFeetRoles();
            if (allowFootChange)
                footChanges++;
            else
                footChanges = 2;
        }
    }

    return result;
}


bool ChppGikLocomotionPlanner::bigSolve(unsigned int inMaxIter)
{
    if (!attStandingRobot->supportPolygon()->isDoubleSupport())
    {
        std::cout << "ChppGikLocomotionPlanner::solve() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    if (attConstraints.size() == 0)
    {
        std::cout << "bool ChppGikLocomotionPlanner::solve(): nothing to do.\n";
        return true;
    }

    //previous results
    clearFeet();
    attDetailedMotion->clear();
    attRefinedMotion->clear();

    bool problemSolved = false, littleSolved;

    unsigned int maxIter = (inMaxIter)?inMaxIter:attSolveDefaultMaxIter;

    while (attSolutionFeet.size() <= maxIter)
    {
        problemSolved =  defaultStopConditionMet();
        if (problemSolved == true)
        {
            std::cout << "End. Reason: solved\n";
            break;
        }

        littleSolved = littleSolve();
        if (littleSolved == false)
        {
            std::cout << "The feet are not contributing anymore to the constraints \n";
            break;
        }

        saveState();
    }

    appendLast();

    return problemSolved;
}


bool ChppGikLocomotionPlanner::enforceFeetDomain()
{
    if (!(attStandingRobot->supportPolygon()->isDoubleSupport()))
    {
        std::cout << " ChppGikLocomotionPlanner::enforceFeetDomain(): sp not double\n";
        return false;
    }

    const matrix4d& nsfm = attNonSupportFoot->currentTransformation();
    const matrix4d& sfm = attSupportFoot->currentTransformation();

    ChppGikSupportPolygon* supportPolygon = attStandingRobot->supportPolygon();

    const ChppGikFootprint* SFP = 0;
    ChppGikFootprint* relNSFP = 0;
    vector3d gap;

    if (attSupportFoot == attRobot->rightFoot())
    {
        SFP = supportPolygon->rightFootprint();
        relNSFP = new ChppGikFootprint(*(supportPolygon->leftFootprint()));
    }
    else
    {
        SFP = supportPolygon->leftFootprint();
        relNSFP = new ChppGikFootprint(*(supportPolygon->rightFootprint()));
    }

    bool modifyPosition = false, modifyRotation = false, projectTarget = false;



    double co, si, hyp, theta;

    ChppGikFootprint::makeRelative( SFP, relNSFP );
    hyp = sqrt(relNSFP->y()*relNSFP->y() + relNSFP->x()*relNSFP->x());

    if (attSupportFoot == attRobot->rightFoot())
    {
        if (attCheckForPos)
        {
            co = relNSFP->x() / hyp;
            si = relNSFP->y() / hyp;
            theta = atan2(si, co);

            if (theta < attMinPosTheta )
            {
                theta = attMinPosTheta;
                modifyPosition = true;
            }
            else
                if (theta > attMaxPosTheta )
                {
                    theta = attMaxPosTheta;
                    modifyPosition = true;
                }

            relNSFP->x(hyp*cos(theta)) ;
            relNSFP->y(hyp*sin(theta)) ;
        }

        if (attCheckForRot)
        {
            theta = relNSFP->th();
            if (theta < attMinRelTheta)
            {
                theta = attMinRelTheta;
                modifyRotation = true;
            }
            else
                if (theta > attMaxRelTheta)
                {
                    theta = attMaxRelTheta;
                    modifyRotation = true;
                }

            relNSFP->th(theta);
        }
    }

    else
    {
        if (attCheckForPos)
        {
            co = relNSFP->x() / hyp;
            si = relNSFP->y() / hyp;
            theta = atan2(si, co);

            if (theta > -attMinPosTheta )
            {
                theta = -attMinPosTheta;
                modifyPosition = true;
            }
            else
                if (theta < -attMaxPosTheta )
                {
                    theta = -attMaxPosTheta;
                    modifyPosition = true;
                }

            relNSFP->x(hyp*cos(theta)) ;
            relNSFP->y(hyp*sin(theta)) ;
        }

        if (attCheckForRot)
        {
            theta = relNSFP->th();
            //std::cout <<"theta before "<<theta<<"\n";
            if (theta > -attMinRelTheta)
            {
                theta = -attMinRelTheta;
                modifyRotation = true;
            }
            else
                if (theta < -attMaxRelTheta)
                {
                    theta = -attMaxRelTheta;
                    modifyRotation = true;
                }

            //std::cout <<"theta after "<<theta<<"\n";
            relNSFP->th(theta);
        }
    }

    ChppGikFootprint::makeAbsolute( SFP, relNSFP );


    vector3d footTarget;
    matrix3d footRot;
    footTarget[2] = attRobot->footHeight();

    M3_IJ(footRot,0,0) = M3_IJ(footRot,1,1) = cos(relNSFP->th());
    M3_IJ(footRot,1,0) = sin(relNSFP->th());
    M3_IJ(footRot,0,1) = -sin(relNSFP->th());
    M3_IJ(footRot,0,2) = M3_IJ(footRot,1,2) = 0;
    M3_IJ(footRot,2,2) = 1;

    footTarget[0] = relNSFP->x();
    footTarget[1] = relNSFP->y();


    gap[0] = relNSFP->x() - SFP->x();
    gap[1] = relNSFP->y() - SFP->y();
    gap[2] = 0.0;

    delete relNSFP;

    if (attCheckForDist)
    {
        double dist = sqrt(gap[0]*gap[0] + gap[1]*gap[1]);

        if (dist < 1e-2)
        {
            std::cout << "unhandeled exception\n";
            return false;
        }


        double ratio = 1;

        if (dist > attMaxDist)
        {
            ratio = (attMaxDist/dist);
            projectTarget = true;
        }

        if (dist < attMinDist)
        {
            ratio = (attMinDist/dist);
            projectTarget = true;
        }

        if (projectTarget)
        {
            footTarget[0] = SFP->x() + ratio*gap[0];
            footTarget[1] = SFP->y() + ratio*gap[1];
        }

    }
    std::vector<CjrlGikStateConstraint*> constraints;

    bool gik2Solved = true;

    if (modifyPosition || projectTarget || modifyRotation)//(projectTarget)//
    {
        //restore previous configuration
        attRobot->applyConfiguration( attPreviousConfiguration );

        attFootPosition->worldTarget( footTarget );

        constraints.push_back(attLPConstraint);
        constraints.push_back(attLP2Constraint);
        constraints.push_back(attWaistParallel);
        constraints.push_back(attWaistPlane);
        constraints.push_back(attFootRot);
        constraints.push_back(attFootPosition);

        appendTargets( constraints );

        std::vector<CjrlGikStateConstraint*> jrlWatched;

        jrlWatched.push_back(attLPConstraint);
        jrlWatched.push_back(attLP2Constraint);
        jrlWatched.push_back(attWaistParallel);
        jrlWatched.push_back(attWaistPlane);
        jrlWatched.push_back(attFootRot);
        jrlWatched.push_back(attFootPosition);

        gik2Solved = solveGik(constraints, jrlWatched, true);

    }

    ///*
    if (modifyPosition)
        std::cout << "modifyPosition \n";
    if (projectTarget)
        std::cout << "projectTarget \n";
    if (modifyRotation)
        std::cout << "modifyRotation \n";
    //*/

    return gik2Solved;
}

bool ChppGikLocomotionPlanner::defaultStopConditionMet()
{
    bool result = false;

    vectorN backupConfig = attRobot->currentConfiguration();

    attGikSolver->weights( attWeightedWholeBody);

    std::vector<CjrlGikStateConstraint*> constraints;

    const matrix4d& mnsp = attNonSupportFoot->currentTransformation();
    attFootTransformation->targetTransformation(mnsp );

    double cx, cy;
    attStandingRobot->supportPolygon()->center( cx, cy);
    attComConstraint->targetXY( cx, cy);

    double itermaxval = 0.2;
    bool doSolve = true;
    double iters  = 2;
    std::vector<ChppGikPlannableConstraint*> planConstraints;
    std::vector<CjrlGikStateConstraint*> jrlConstraints;

    for (unsigned int iter= 0; iter< iters;iter++)
    {
        constraints.clear();
        constraints.push_back(attFootTransformation);
        //constraints.push_back(attWaistParallel);
        constraints.push_back(attComConstraint);

        appendClampedTargets(constraints, itermaxval);

        result = solveGik( constraints, constraints );
    }

    result = true;
    for (unsigned int wc =0; wc<attConstraints.size();wc++)
    {
        attUserConstraints[wc]->computeVectorizedTarget();
        attConstraints[wc]->vectorizedTarget(attUserConstraints[wc]->vectorizedTarget());
        attConstraints[wc]->computeValue();
        if ((norm_2(attConstraints[wc]->value()) > attNvalthresh))
            result = false;
    }

    attFinalConfiguration = attRobot->currentConfiguration();
    attRobot->applyConfiguration( backupConfig );

    attGikSolver->weights( attWeights );

    return result;
}



bool ChppGikLocomotionPlanner::comMovable()
{

    vectorN backupConfig = attRobot->currentConfiguration();

    bool gikSolved = false;
    std::vector<CjrlGikStateConstraint*> modConstraints;

    const matrix4d& mnsf = attNonSupportFoot->currentTransformation();
    attFootTransformation->targetTransformation( mnsf );
    modConstraints.push_back(attFootTransformation);
    //modConstraints.push_back(attWaistPlane);
    modConstraints.push_back(attWaistParallel);


    const matrix4d& ml = attRobot->leftFoot()->currentTransformation();
    attComConstraint->targetXY( M4_IJ(ml,0,3), M4_IJ(ml,1,3) );
    modConstraints.push_back(attComConstraint);

    //solve constraints
    gikSolved = solveGik(modConstraints, modConstraints);

    if (!gikSolved)
    {
        attRobot->applyConfiguration( backupConfig );
        return false;
    }

    const matrix4d& mr = attRobot->rightFoot()->currentTransformation();
    attComConstraint->targetXY( M4_IJ(mr,0,3), M4_IJ(mr,1,3) );
    modConstraints.pop_back();
    modConstraints.push_back(attComConstraint);

    gikSolved = solveGik(modConstraints, modConstraints);
    attRobot->applyConfiguration( backupConfig );

    return gikSolved;
}


std::vector<ChppGikFoot*> ChppGikLocomotionPlanner::bigSolution()
{
    return attRawFeet;
}

void ChppGikLocomotionPlanner::appendLast()
{
    attSolutionFeet.push_back(attRawFeet[attRawFeet.size()-1]);
    attDetailedMotion->appendSample( attFinalConfiguration, attDummyVec,attDummyVec,attDummyVec,attDummyVec);
    attRefinedMotion->appendSample( attFinalConfiguration, attDummyVec,attDummyVec,attDummyVec,attDummyVec);
}

void ChppGikLocomotionPlanner::saveState()
{
    const ChppRobotMotionSample* previousLastSample = attDetailedMotion->lastSample();

    ChppGikSupportPolygon* curSP = attStandingRobot->supportPolygon();

    if (attNonSupportFoot == attRobot->rightFoot())
        attRawFeet.push_back(new ChppGikFoot(*(curSP->rightFootprint()), true));
    else
        attRawFeet.push_back(new ChppGikFoot(*(curSP->leftFootprint()), false));

    attDetailedMotion->appendSample( attRobot->currentConfiguration(),attDummyVec,attDummyVec,attDummyVec,attDummyVec);

    unsigned int last = attRawFeet.size() - 1;
    if (last > 0)
    {
        if (attRawFeet[last]->isRight() != attRawFeet[last-1]->isRight())
        {
            attSolutionFeet.push_back(attRawFeet[last-1]);
            attRefinedMotion->appendSample(*previousLastSample);
        }
    }
}

bool ChppGikLocomotionPlanner::solveGik(std::vector<CjrlGikStateConstraint*>& inConstraints, std::vector<CjrlGikStateConstraint*>& watchedConstraints, bool verbose)
{
    bool gikOK, gikSolved = false;
    unsigned int nGikIter = attGikMaxIter;
    vector3d p;

    std::vector<double> dampers;
    //     for (unsigned int i = 0; i< inConstraints.size();i++)
    //     {dampers.push_back(0.001);
    //     }

    while (nGikIter > 0 && ! gikSolved)
    {
        attRobot->fixedJoint(0).computeJacobianJointWrtConfig();


        //compute constraints jacobians and values
        for (unsigned int i = 0; i< inConstraints.size();i++)
        {
            inConstraints[i]->computeValue();
            inConstraints[i]->computeJacobian();
        }

        //Account for joint limits: modifies the weights according to current configuration
        attGikSolver->accountForJointLimits();


        gikOK = attGikSolver->gradientStep( inConstraints );//, dampers );

        //attDetailedMotion->appendSample( attRobot->currentConfiguration(), p,p,p,p);
        if (!gikOK)
            return false;


        gikSolved = true;
        for (unsigned int wc =0; wc<watchedConstraints.size();wc++)
        {
            watchedConstraints[wc]->computeValue();
            if ((norm_2(watchedConstraints[wc]->value()) > attNvalthresh))
                gikSolved = false;
        }
        if (gikSolved)
            return gikSolved;

        nGikIter--;
    }

    if (verbose == true)
        for (unsigned int wc =0; wc<watchedConstraints.size();wc++)
        {
            watchedConstraints[wc]->computeValue();
            std::cout << watchedConstraints[wc]->value() << "\n";
        }

    return false;

}

void ChppGikLocomotionPlanner::clearFeet()
{
    //delete previous solution
    for (unsigned int i=0; i<attRawFeet.size();i++)
        delete attRawFeet[i];
    attRawFeet.clear();
    attSolutionFeet.clear();
}

void ChppGikLocomotionPlanner::clearConstraints()
{
    //delete previous constraint copies
    for (unsigned int i=0; i<attConstraints.size();i++)
        delete attConstraints[i];
    attConstraints.clear();
    attUserConstraints.clear();
}


void ChppGikLocomotionPlanner::constraints(const std::vector<ChppGikPlannableConstraint*>& inConstraints)
{
    clearConstraints();
    //copy constraints
    for (unsigned int i=0; i<inConstraints.size();i++)
    {
        CjrlGikStateConstraint* cstr = inConstraints[i]->clone();
        ChppGikPlannableConstraint* pc = dynamic_cast<ChppGikPlannableConstraint*>(cstr);
        attConstraints.push_back(pc);
    }
    //copy pointers to originals
    for (unsigned int i=0; i<inConstraints.size();i++)
    {
        attUserConstraints.push_back(inConstraints[i]);
        attUserConstraints[i]->computeVectorizedTarget();
    }
}

void ChppGikLocomotionPlanner::weights(vectorN& inWeights)
{
    attWeights = inWeights;
    attGikSolver->weights( attWeights );
}

void ChppGikLocomotionPlanner::appendTargets(std::vector<CjrlGikStateConstraint*>& outResult)
{
    for (unsigned int i=0; i<attConstraints.size();i++)
        outResult.push_back(attConstraints[i]);
}

void ChppGikLocomotionPlanner::appendClampedTargets(std::vector<CjrlGikStateConstraint*>& outResult, double maxNorm)
{
    double lambda;

    for (unsigned int i=0; i<attConstraints.size();i++)
    {
        attConstraints[i]->computeVectorizedState();
        const vectorN& currentState = attConstraints[i]->vectorizedState();
        attUserConstraints[i]->computeVectorizedTarget();
        vectorN interpolatedVec(attUserConstraints[i]->vectorizedTarget());

        lambda = 1;
        while (1)
        {
            interpolatedVec = (1-lambda)*subrange(currentState,0,interpolatedVec.size()) + lambda*(attUserConstraints[i]->vectorizedTarget());
            attConstraints[i]->vectorizedTarget(interpolatedVec);
            attConstraints[i]->computeValue();
            const vectorN& val = attConstraints[i]->value();

            if (norm_2(val) <= maxNorm)
                break;

            lambda = lambda/2;
        }
        outResult.push_back(attConstraints[i]);
    }
}

void ChppGikLocomotionPlanner::dumpBigSolutionTo(const char* inFilename)
{
    if (attRawFeet.size() == 0)
    {
        std::cout << "nothing to dump\n";
        return;
    }

    matrixNxP footprints(4, attRawFeet.size());
    for (unsigned int i=0; i<attRawFeet.size();i++)
    {
        footprints(0,i) = attRawFeet[i]->footprint().x();
        footprints(1,i) = attRawFeet[i]->footprint().y();
        footprints(2,i) = attRawFeet[i]->footprint().th();
        if (attRawFeet[i]->isRight())
            footprints(3,i) = 1;
        else
            footprints(3,i) = 0;
    }


    ChppGikTools::dumpMatrix( inFilename, footprints, 0.0, 0.0);
    attDetailedMotion->dumpTo(inFilename);

    footprints.resize(4, attSolutionFeet.size(), false);
    unsigned int rightFoot = (attRawFeet[0]->isRight())?0:1;

    for (unsigned int i=0; i<attSolutionFeet.size();i++)
    {
        footprints(0,i) = attSolutionFeet[i]->footprint().x();
        footprints(1,i) = attSolutionFeet[i]->footprint().y();
        footprints(2,i) = attSolutionFeet[i]->footprint().th();
        rightFoot = !rightFoot;
        footprints(3,i) = rightFoot;
    }

    ChppGikTools::dumpMatrix( "refined", footprints, 0.0, 0.0);
    attRefinedMotion->dumpTo("refined");
}
