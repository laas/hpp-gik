#include <time.h>
#include <sys/time.h>
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "boost/numeric/ublas/operation.hpp"
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/lapack/gesvd.hpp>
#include "core/hppGikSolver.h"
#include "hppGikTools.h"

namespace lapack = boost::numeric::bindings::lapack;

using namespace ublas;

ChppGikSolver::ChppGikSolver ( CjrlDynamicRobot* inRobot )
{
    attRobot = inRobot;

    numDof = attRobot->numberDof();
    if (attRobot->countFixedJoints() > 0)
    {
        numJoints = numDof-6;
        Offset = 6;
    }
    else
    {
        numJoints = 6;
        Offset = 0;
    }

    xDefaultDim = 6;
    attSVDThreshold = 0.001;

    LongSize = LongSizeBackup = numJoints;
    scalar_vector<char> temp ( numJoints,1 );
    PIWeights = temp;
    NextPIWeights = temp;
    PIWeightsBackup = temp;
    Weights = temp;


    UsedIndexes.resize ( numJoints,false );
    NextUsedIndexes.resize ( numJoints,false );
    UsedIndexesBackup.resize ( numJoints,false );

    for ( unsigned int i=0; i<numJoints;i++ )
    {
        UsedIndexes ( i ) = i;
        UsedIndexesBackup ( i ) = i;
    }


    identity_matrix<double> tempid ( numJoints );
    IdentityMat.resize ( numJoints,numJoints,false );
    IdentityMat = tempid;

    resizeMatrices ( xDefaultDim );

    CurFullConfig.resize ( numDof,false );

    BaseEuler.resize ( 3,false );

    H0.resize ( 4,4,false );
    Hf.resize ( 4,4,false );
    Hif.resize ( 4,4,false );
    InvHf.resize ( 4,4,false );

    prepareBrakeWindow();

    jobU = 'N';
    jobVt = 'A';

}


void ChppGikSolver::resizeMatrices ( unsigned int inSize)
{
    Residual.resize ( inSize,false );
    PenroseMask.resize ( inSize,false );
    tempS.resize(inSize, false);

    CarvedJacobian.resize ( inSize,LongSize,false );
    HatJacobian.resize ( inSize,LongSize,false );
    WJt.resize ( LongSize,inSize,false );
    JWJt.resize ( inSize,inSize,false );
    Jsharp.resize ( LongSize,inSize,false );
    InverseJWJt.resize ( inSize,inSize,false );

    tempU.resize(inSize,inSize,false);
    tempVt.resize(inSize,inSize,false);
}


void ChppGikSolver::prepareBrakeWindow()
{
    BrakingZone = 0.12;
    WindowStep = 1e-3;

    unsigned int extra = 10;
    unsigned int sizeWindow = ChppGikTools::timetoRank ( 0.0,1.0,WindowStep ) +1 ;

    JointUpperLimitWindow.resize ( sizeWindow,false );

    ChppGikTools::minJerkCurve ( 1.0-WindowStep*extra,WindowStep,1.0,0.0,0.0,0.0,JointUpperLimitWindow );

    subrange ( JointUpperLimitWindow, sizeWindow - extra, sizeWindow ) = zero_vector<double> ( extra );
}


double ChppGikSolver::brakeCoefForJoint ( const double& qVal,const double& lowerLimit, const double& upperLimit, const double& dq )
{

    if ( upperLimit ==lowerLimit )
        return 0;

    double qFraction = ( qVal-lowerLimit ) / ( upperLimit-lowerLimit );


    if ( ( dq>0 ) && ( qFraction > 1 - BrakingZone ) )
    {
        double curve_Fraction = ( qFraction + BrakingZone - 1 ) /BrakingZone;
        unsigned int windowTick = ChppGikTools::timetoRank ( 0.0, curve_Fraction, WindowStep );
        if ( windowTick >= JointUpperLimitWindow.size() )
            return 0;
        return JointUpperLimitWindow ( windowTick );
    }

    if ( ( dq<0 ) && ( qFraction < BrakingZone ) )
    {
        double curve_Fraction = qFraction/BrakingZone;
        unsigned int windowTick = ChppGikTools::timetoRank ( 0.0, curve_Fraction, WindowStep );
        if (windowTick >= JointUpperLimitWindow.size())
            return 0;

        return JointUpperLimitWindow ( JointUpperLimitWindow.size()-1-windowTick );
    }

    return 1;
}

bool ChppGikSolver::weights ( vectorN& inWeights )
{
    if ( inWeights.size() == numJoints )
    {
        Weights = inWeights;
        return true;
    }
    else
        return false;
}


void ChppGikSolver::accountForJointLimits()
{
    LongSizeBackup = 0;
    unsigned int realrank;
    double coefLjoint;
    const vectorN& cfg = attRobot->currentConfiguration();
    double q, ub, lb;

    for ( unsigned int i=0; i<numJoints;i++ )
        if ( Weights ( i ) >1e-2 )
        {
            realrank = Offset + i;

            if (realrank >= 6)
            {
                q = attRobot->currentConfiguration()(realrank);
                lb = attRobot->lowerBoundDof(realrank, cfg);
                ub = attRobot->upperBoundDof(realrank, cfg);
                coefLjoint =  brakeCoefForJoint(q,lb,ub,attRobot->currentVelocity()(realrank));
            }
            else
            {
                coefLjoint = 1.0;
            }

            UsedIndexesBackup ( LongSizeBackup ) = i;

            PIWeightsBackup ( LongSizeBackup ) = Weights ( i ) * coefLjoint;

            LongSizeBackup++;
        }

    //std::cout <<UsedIndexesBackup << "\n";
}

void ChppGikSolver::solveOneConstraint(CjrlGikStateConstraint *inConstraint,
                                       double inSRcoef)
{
    //determin task dimension
    xDim = inConstraint->dimension();

    resizeMatrices ( xDim );

    //store used columns only
    for ( unsigned int col = 0; col<LongSize;col++ )
        noalias (column ( CarvedJacobian,col) )=  column ( inConstraint->jacobian(),UsedIndexes ( col ));

    //update value according to previous tasks
    noalias (Residual) = inConstraint->value();
    noalias ( Residual) -= prod (CarvedJacobian,DeltaQ);

    //slightly faster product to obtain projected jacobian on previous tasks nullspace
    HatJacobian.clear();
    vectorN& ElementMask = inConstraint->influencingDofs();

    for ( unsigned int d = 0; d<xDim;d++ )
    {
        for ( unsigned int col = 0; col<LongSize;col++ )
            if ( ElementMask ( UsedIndexes ( col ) + Offset ) == 1 )
                noalias ( row (HatJacobian,d) ) += CarvedJacobian ( d,col ) * row ( NullSpace,col);
    }

    //pseudo inverse of the projected jacobian
    noalias ( WJt ) = trans ( HatJacobian);

    for ( unsigned int lin=0;lin<LongSize;lin++ )
        row ( WJt,lin ) *= PIWeights ( lin );

    noalias ( JWJt ) = prod ( HatJacobian,  WJt );

    //svd
    if (inSRcoef != 0)
    {
        identity_matrix<double> I ( xDim );
        JWJt = inSRcoef*I + JWJt;
    }
    lapack::gesvd ( jobU, jobVt, JWJt, tempS, tempU, tempVt );
    tempU = trans ( tempVt );

    PenroseMask.clear();

    //Determin task value projection space
    for ( unsigned int i=0;i<xDim;i++ )
    {
        if ( attSVDThreshold > tempS ( i ) )
            row ( tempVt,i ) *= 0;
        else
        {
            PenroseMask ( i ) =  1;
            row ( tempVt,i ) /= tempS ( i );
        }
    }

    if ( PenroseMask ( 0 ) ==0 )
    {
        return;
    }

    //inverse of JwJt
    noalias ( InverseJWJt ) = prod ( tempU,tempVt );

    //PseudoInverse Jsharp
    noalias ( Jsharp ) = prod (WJt , InverseJWJt);

    //Updated deltaQ
    noalias ( DeltaQ ) += prod ( Jsharp, Residual );

    //Updated null space
    noalias (  NullSpace ) -= prod ( Jsharp, HatJacobian );


}

bool ChppGikSolver::gradientStep ( std::vector<CjrlGikStateConstraint*>& inSortedConstraints)
{
    std::vector<double> SRcoefs(inSortedConstraints.size(), 0);
    return gradientStep(inSortedConstraints, SRcoefs);
}

bool ChppGikSolver::gradientStep ( std::vector<CjrlGikStateConstraint*>& inSortedConstraints, std::vector<double>& inSRcoefs )
{
    LongSize = LongSizeBackup;
    subrange ( PIWeights,0,LongSize ) = subrange ( PIWeightsBackup,0,LongSize );
    subrange ( UsedIndexes,0,LongSize ) = subrange ( UsedIndexesBackup,0,LongSize );



    std::vector<CjrlJoint*> supportJoints;
    if (attRobot->countFixedJoints()>0)
    {
        //store the fixed foot's tranformation (Hif)
        FixedJoint = & ( attRobot->fixedJoint ( 0 ) );
        //joints from root to fixed joint (including both)
        supportJoints = FixedJoint->jointsFromRootToThis();
        ChppGikTools::Matrix4toUblas ( FixedJoint->currentTransformation(),
                                       Hif );
    }
    else
    {
        FixedJoint = NULL;
    }

    bool recompute = true;
    std::vector<CjrlGikStateConstraint*>::iterator iter;
    std::vector<double>::iterator iter2;
    while ( recompute )
    {
        NullSpace.resize ( LongSize, LongSize,false );
        NullSpace = subrange ( IdentityMat,0,LongSize,0,LongSize );

        DeltaQ.resize ( LongSize,false );
        DeltaQ.clear();

        //for every subtask
        for ( iter = inSortedConstraints.begin(), iter2 = inSRcoefs.begin();
                iter != inSortedConstraints.end(); iter++,iter2++ )
        {
            solveOneConstraint(*iter, *iter2);
        }

        //compute new dof vector & perform a basic check on joint limits
        CurFullConfig = attRobot->currentConfiguration();

        //update dof config
        unsigned int iC,realIndex, start;
        if (attRobot->countFixedJoints()>0)
        {
            start = 0;
        }
        else
        {
            start = 6;
            iC = 0;
            while(iC < LongSize && UsedIndexes(iC)<3)
            {
                CurFullConfig(UsedIndexes(iC)) += DeltaQ(iC);
                iC++;
            }
            matrixNxP R1(3,3), R2(3,3), R(3,3);
            ChppGikTools::EulerZYXtoRot(CurFullConfig(3), CurFullConfig(4),
                                        CurFullConfig(5), R1);
            vectorN euler(3);
            euler[0] = euler[1] = euler[2] = 0;
            while(iC < LongSize && UsedIndexes(iC)<6)
            {
                euler(UsedIndexes(iC)-3) = DeltaQ(iC);
                iC++;
            }
            ChppGikTools::EulerZYXtoRot(euler, R2);
            R = prod(R2,R1);
            ChppGikTools::RottoEulerZYX(R, euler);
            CurFullConfig(3) = euler(0);
            CurFullConfig(4) = euler(1);
            CurFullConfig(5) = euler(2);
        }
        for ( iC=start; iC< LongSize; iC++ )
        {
            realIndex = Offset+UsedIndexes ( iC );
            CurFullConfig ( realIndex ) += DeltaQ ( iC );
        }

        //joint limit checking
        double ub, lb;
        recompute = false;
        unsigned int NextLongSize = 0;
        for ( iC=0; iC< LongSize; iC++ )
        {
            realIndex = Offset+UsedIndexes ( iC );
            if (realIndex >= 6)
            {
                lb = attRobot->lowerBoundDof(realIndex, CurFullConfig);
                ub = attRobot->upperBoundDof(realIndex, CurFullConfig);
            }
            //check for joint limits
            if ( realIndex >= 6
                    && (CurFullConfig ( realIndex ) < lb +1e-2
                        || CurFullConfig ( realIndex ) > ub-1e-2 ))
            {
                recompute = true;

            }
            else
            {
                NextPIWeights ( NextLongSize ) = PIWeights ( iC );
                NextUsedIndexes ( NextLongSize ) = UsedIndexes ( iC );
                NextLongSize++;
            }
        }
        if ( recompute )
            if ( NextLongSize == 0 )
            {
                std::cout << "gikSolver(): ran out of movable joints. (Check the weights vector ?)\n";
                return false;
            }
            else
            {
                UsedIndexes = NextUsedIndexes;
                LongSize = NextLongSize;
                PIWeights = NextPIWeights;
            }
    }
    if (FixedJoint)
    {
        unsigned int iC;
        //go to waist frame
        for ( iC=0; iC< 6; iC++ )
            CurFullConfig ( iC ) = 0;

        //update joints transformations from root to fixed joint in waist frame
        for ( iC=0; iC< supportJoints.size(); iC++ )
            supportJoints[iC]->updateTransformation ( CurFullConfig );

        //Compute new waist transformation
        ChppGikTools::Matrix4toUblas ( FixedJoint->currentTransformation(),Hf );
        ChppGikTools::invertTransformation ( Hf,InvHf );
        noalias ( H0 ) = prod ( Hif, InvHf );

        //Compute free flyer dofs (apparently the dynamic robot require euler XYZ)
        ChppGikTools::RottoEulerZYX ( subrange ( H0,0,3,0,3 ),BaseEuler );
        for ( iC=0; iC< 3; iC++ )
            CurFullConfig ( iC ) = H0 ( iC,3 );
        for ( iC=3; iC< 6; iC++ )
            CurFullConfig ( iC ) = BaseEuler ( iC-3 );
    }

    attRobot->applyConfiguration ( CurFullConfig );

    return true;
}


ChppGikSolver::~ChppGikSolver()
{}

