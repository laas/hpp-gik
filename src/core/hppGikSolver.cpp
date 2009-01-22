#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "core/hppGikSolver.h"
#include "hppGikTools.h"

using namespace boost::numeric::ublas;

ChppGikSolver::ChppGikSolver(CjrlDynamicRobot& inRobot)
{
    attRobot = &inRobot;

    attNumParams = inRobot.numberDof();
    attSolver = new ChppGikSolverBasic(attNumParams);
    attBounder = new ChppGikBounder(attNumParams);
    for (unsigned int i=0;i<attNumParams;i++)
    {
        attBounder->upperBound(i, attRobot->upperBoundDof(i));
        attBounder->lowerBound(i, attRobot->lowerBoundDof(i));
    }
    unsigned int rootRank = attRobot->rootJoint()->rankInConfiguration();
    unsigned int rootNumDof = attRobot->rootJoint()->numberDof();
    for (unsigned int i=rootRank;i<rootRank+rootNumDof;i++)
    {
        attBounder->upperBound(i, 1e10);
        attBounder->lowerBound(i, -1e10);
    }
    attWeights = attComputationWeights = attActive = scalar_vector<double>(attNumParams,1);
    attSolution = zero_vector<double>(attNumParams);
    attEulerSolution = attSolution;
    BaseEuler.resize( 3,false );
    ElementMask.resize(attNumParams);

    H0.resize ( 4,4,false );
    Hf.resize ( 4,4,false );
    Hif.resize ( 4,4,false );
    InvHf.resize ( 4,4,false );

    rootJoint(*(attRobot->rootJoint()));
}

void ChppGikSolver::rootJoint(CjrlJoint& inRootJoint)
{
    if (RootJoint != &inRootJoint)
    {
        RootJoint = &inRootJoint;
        supportJoints = RootJoint->jointsFromRootToThis();
    }
}

bool ChppGikSolver::weights ( vectorN& inWeights )
{
    if ( inWeights.size() == attNumParams )
    {
        attWeights = inWeights;
        return true;
    }
    else
    {
        std::cout << "ChppGikSolver::weights() incorrect size()"<<std::endl;
        return false;
    }
}

void ChppGikSolver::solve ( std::vector<CjrlGikStateConstraint*>& inSortedConstraints)
{
    std::vector<double> SRcoefs(inSortedConstraints.size(), 0);
    solve(inSortedConstraints, SRcoefs);
}


void ChppGikSolver::prepare(std::vector<CjrlGikStateConstraint*>& inTasks)
{
    for (unsigned int i=0;i<inTasks.size();i++)
    {
        inTasks[i]->jacobianRoot(*RootJoint);
        inTasks[i]->computeJacobian();
        inTasks[i]->computeValue();
    }
}

void ChppGikSolver::solve(std::vector<CjrlGikStateConstraint*>& inSortedConstraints, std::vector<double>& inSRcoefs)
{
    attSolution.clear();
    if (inSortedConstraints.empty())
    {
        std::cout << "ChppGikSolver::solve() nothing to do"<<std::endl;
        return;
    }

    bool recompute = true;
    unsigned int iC;
    double ub, lb;
    std::vector<CjrlGikStateConstraint*>::iterator iter;
    std::vector<double>::iterator iter2;

    attVals = attRobot->currentConfiguration();
    attRate = attRobot->currentVelocity();
    attComputationWeights = attWeights;
    attBounder->modifyWeights( attVals, attRate, attComputationWeights );

    while ( recompute )
    {
        if (!(attSolver->weights( attComputationWeights )))
        {
            std::cout << "ChppGikSolver::solve(): could not move" << std::endl;
            return;
        }

        iter = inSortedConstraints.begin();
        iter2 = inSRcoefs.begin();

        attSolver->setActiveParameters((*iter)->influencingDofs());
        if (inSortedConstraints.size() == 1)
        {
            attSolver->solveTask(*iter, *iter2, false, false);
        }
        else
        {
            attSolver->solveTask(*iter, *iter2, false, true);
            iter++;
            iter2++;
            while (iter != inSortedConstraints.end()-1)
            {
                attSolver->setActiveParameters((*iter)->influencingDofs());
                attSolver->solveTask(*iter, *iter2, true, true);
                iter++;
                iter2++;
            }
            attSolver->setActiveParameters((*iter)->influencingDofs());
            attSolver->solveTask(*iter, *iter2, true, false);
        }

        //update config and check joint limits
        recompute = false;
        CurFullConfig = attRobot->currentConfiguration();
        for ( iC=0; iC< attNumParams; iC++ )
        {
            //update
            if (attComputationWeights(iC) != 0)
            {
                attBounder->getLowerBound( iC, lb);
                attBounder->getUpperBound( iC, ub);
                CurFullConfig(iC) += attSolver->solution()(iC);
                //threshold
                if ((CurFullConfig(iC) < lb +1e-2 || CurFullConfig ( iC ) > ub-1e-2))
                {
                    recompute = true;
                    attComputationWeights(iC) = 0;
                }
            }
        }
    }
    return;
}

const vectorN& ChppGikSolver::solution()
{
    return attSolver->solution();
}

void ChppGikSolver::applySolution()
{
    convertFreeFlyerVelocity();
    CurFullConfig = attRobot->currentConfiguration();
    CurFullConfig.plus_assign(attEulerSolution);
    attRobot->currentConfiguration( CurFullConfig );
}

void ChppGikSolver::convertFreeFlyerVelocity()
{
    attBackupConfig = CurFullConfig = attRobot->currentConfiguration();
    attSolution = attEulerSolution = attSolver->solution();

    unsigned int i;
    bool changeRootPose = false;
    bool changeRootJoint = (RootJoint!=attRobot->rootJoint());
    if (attRobot->rootJoint()->numberDof()!=0)
        for (i=0;i<6;i++)
            if (attWeights(i)!=0)
            {
                changeRootPose = true;
                break;
            }
    if (changeRootJoint)
    {
        ChppGikTools::Matrix4toUblas ( RootJoint->currentTransformation(), Hif );

        //go to waist frame
        for ( i =0; i< 6; i++ )
            CurFullConfig ( i ) = 0;

        for ( i=1; i< supportJoints.size(); i++  )
            CurFullConfig(supportJoints[i]->rankInConfiguration()) += attSolution(supportJoints[i]->rankInConfiguration());

        //update joints transformations from root to fixed joint in waist frame
        for ( i=0; i< supportJoints.size(); i++ )
            supportJoints[i]->updateTransformation ( CurFullConfig );

        //Compute new waist transformation
        ChppGikTools::Matrix4toUblas ( RootJoint->currentTransformation(),Hf );
        ChppGikTools::invertTransformation ( Hf,InvHf );
        noalias ( H0 ) = prod ( Hif, InvHf );
    }

    if (changeRootPose)
    {
        matrixNxP rootDeltaR,rootChange(4,4);
        vectorN omega = subrange(attSolution,3,6);
        ChppGikTools::OmegaToR( omega, rootDeltaR);
        subrange(rootChange,0,3,0,3) = rootDeltaR;
        rootChange(0,3) =  attSolution(0);
        rootChange(1,3) =  attSolution(1);
        rootChange(2,3) =  attSolution(2);
        if (changeRootJoint)
            H0 = prod ( rootChange, H0 );
        else
        {
            ChppGikTools::Matrix4toUblas ( RootJoint->currentTransformation(),Hf );
            noalias ( H0 ) = prod ( rootChange, Hf );
        }
    }

    if (changeRootJoint || changeRootPose)
    {
        //Compute free flyer velocity update
        ChppGikTools::RottoEulerZYX ( subrange ( H0,0,3,0,3 ),BaseEuler );
        for ( i=0; i< 3; i++ )
            attEulerSolution ( i ) = H0 ( i,3 ) - attBackupConfig(i);
        for ( i=3; i< 6; i++ )
            attEulerSolution ( i ) = BaseEuler ( i-3 ) - attBackupConfig(i);
    }

    if (changeRootJoint)
        //Recover joints transformations from root to fixed joint in waist frame
        for ( i=0; i< supportJoints.size(); i++ )
            supportJoints[i]->updateTransformation ( attBackupConfig );

}

ChppGikSolver::~ChppGikSolver()
{
    delete attSolver;
    delete attBounder;
}
