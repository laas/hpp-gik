#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "core/hppGikSolverRobotAttached.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikSolverRobotAttached::ChppGikSolverRobotAttached(CjrlDynamicRobot& inRobot)
{
    attRobot = &inRobot;
    attNumParams = inRobot.numberDof()-6;
    attSolver = new ChppGikSolverBasic(attNumParams);
    attBounder = new ChppGikBounder(attNumParams);
    for (unsigned int i=0;i<attNumParams;i++)
    {
        attBounder->upperBound(i, attRobot->upperBoundDof(i+6));
        attBounder->lowerBound(i, attRobot->lowerBoundDof(i+6));
    }
    attWeights = attComputationWeights = attActive = scalar_vector<double>(attNumParams,1);
    attSolution = zero_vector<double>(inRobot.numberDof());

    BaseEuler.resize ( 3,false );
    ElementMask.resize(attNumParams);
            
    H0.resize ( 4,4,false );
    Hf.resize ( 4,4,false );
    Hif.resize ( 4,4,false );
    InvHf.resize ( 4,4,false );
}

bool ChppGikSolverRobotAttached::weights ( vectorN& inWeights )
{
    if ( inWeights.size() == attNumParams )
    {
        attWeights = inWeights;
        return true;
    }
    else
    {
        std::cout << "ChppGikSolverRobotAttached::weights() incorrect size()\n";
        return false;
    }
}

void ChppGikSolverRobotAttached::computeConstraintDofs(CjrlGikStateConstraint* inConstraint)
{
    ElementMask = subrange(inConstraint->influencingDofs(),6,attRobot->numberDof());
    //Take into account support Leg dofs
    for(unsigned int em = 0; em < supportJointsRanks.size(); em++)
       ElementMask(supportJointsRanks[em]) = 1;
}

bool ChppGikSolverRobotAttached::solve ( std::vector<CjrlGikStateConstraint*>& inSortedConstraints)
{
    std::vector<double> SRcoefs(inSortedConstraints.size(), 0);
    return solve(inSortedConstraints, SRcoefs);
}

bool ChppGikSolverRobotAttached::solve(std::vector<CjrlGikStateConstraint*>& inSortedConstraints, std::vector<double>& inSRcoefs)
{
    if (inSortedConstraints.empty())
    {
        attSolution.clear();
        std::cout << "ChppGikSolverRobotAttached::solve() nothing to do\n";
        return true;
    }

    if (attRobot->countFixedJoints()>0)
    {
        if (FixedJoint != & ( attRobot->fixedJoint ( 0 ) ))
        {
            FixedJoint = & ( attRobot->fixedJoint ( 0 ) );
            supportJoints = FixedJoint->jointsFromRootToThis();
            supportJointsRanks.clear();
            for(unsigned int em = 1; em < supportJoints.size(); em++)
                supportJointsRanks.push_back(supportJoints[em]->rankInConfiguration()-6);
        }
        ChppGikTools::Matrix4toUblas ( FixedJoint->currentTransformation(), Hif );
    }
    else
    {
        std::cout << "ChppGikSolverRobotAttached::solve() fixed joint not found in robot\n";
        return false;
    }

    bool recompute = true;
    unsigned int remaining;
    unsigned int iC;
    unsigned int paramIndex;
    double ub, lb;
    std::vector<CjrlGikStateConstraint*>::iterator iter;
    std::vector<double>::iterator iter2;
    
    attVals = subrange(attRobot->currentConfiguration(),6,attRobot->numberDof());
    attRate = subrange(attRobot->currentVelocity(),6,attRobot->numberDof());
    attComputationWeights = attWeights;
    attBounder->modifyWeights( attVals, attRate, attComputationWeights );
    
    while ( recompute )
    {
        
        attSolver->weights( attComputationWeights );
        if (remaining ==0)
        {
            attSolution = zero_vector<double>(attNumParams);
            return true; //cannot move
        }

        iter = inSortedConstraints.begin();
        iter2 = inSRcoefs.begin();
        
        computeConstraintDofs(*iter);
        attSolver->setActiveParameters(ElementMask);
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
                computeConstraintDofs(*iter);
                attSolver->setActiveParameters(ElementMask);
                attSolver->solveTask(*iter, *iter2, true, true);
                iter++;
                iter2++;
            }
            computeConstraintDofs(*iter);
            attSolver->setActiveParameters(ElementMask);
            attSolver->solveTask(*iter, *iter2, true, false);
        }

        //update dof config
        recompute = false;
        CurFullConfig = attRobot->currentConfiguration();
        for ( iC=6; iC< attNumParams+6; iC++ )
            CurFullConfig(iC) += attSolver->solution()(iC-6);
        //joint limit checking
        for ( iC=6; iC< attNumParams+6; iC++ )
        {
            paramIndex = iC-6;
            lb = attRobot->lowerBoundDof(iC);
            ub = attRobot->upperBoundDof(iC);
            if (CurFullConfig(iC) < lb +1e-2 || CurFullConfig ( iC ) > ub-1e-2)
            {
                recompute = true;
                attComputationWeights(paramIndex) = 0;
            }
        }
    }
    subrange(attSolution,6,attRobot->numberDof()) = attSolver->solution();
    computeFreeFlyerVelocity();
    return true;
}

const vectorN& ChppGikSolverRobotAttached::solution()
{
    return attSolution;
}

void ChppGikSolverRobotAttached::applySolution()
{
    CurFullConfig = attRobot->currentConfiguration();
    CurFullConfig.plus_assign(attSolution);
    attRobot->currentConfiguration( CurFullConfig );
}

void ChppGikSolverRobotAttached::computeFreeFlyerVelocity()
{
    attBackupConfig = CurFullConfig = attRobot->currentConfiguration();
    
    //go to waist frame
    unsigned int iC;
    for ( iC =0; iC< 6; iC++ )
        CurFullConfig ( iC ) = 0;

    for ( iC=1; iC< supportJoints.size(); iC++  )
        CurFullConfig(supportJoints[iC]->rankInConfiguration()) += attSolution(supportJoints[iC]->rankInConfiguration());
    
    //update joints transformations from root to fixed joint in waist frame
    for ( iC=0; iC< supportJoints.size(); iC++ )
        supportJoints[iC]->updateTransformation ( CurFullConfig );
    
    //Compute new waist transformation
    ChppGikTools::Matrix4toUblas ( FixedJoint->currentTransformation(),Hf );
    ChppGikTools::invertTransformation ( Hf,InvHf );
    noalias ( H0 ) = prod ( Hif, InvHf );

    //Compute free flyer velocity update
    ChppGikTools::RottoEulerZYX ( subrange ( H0,0,3,0,3 ),BaseEuler );
    for ( iC=0; iC< 3; iC++ )
        attSolution ( iC ) = H0 ( iC,3 ) - attBackupConfig(iC);
    for ( iC=3; iC< 6; iC++ )
        attSolution ( iC ) = BaseEuler ( iC-3 ) - attBackupConfig(iC);
    
    //update joints transformations from root to fixed joint in waist frame
    for ( iC=0; iC< supportJoints.size(); iC++ )
        supportJoints[iC]->updateTransformation ( attBackupConfig );
}




ChppGikSolverRobotAttached::~ChppGikSolverRobotAttached()
{
    delete attSolver;
    delete attBounder;
}
