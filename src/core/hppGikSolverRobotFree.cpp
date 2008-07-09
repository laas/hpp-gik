#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "core/hppGikSolverRobotFree.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikSolverRobotFree::ChppGikSolverRobotFree(CjrlDynamicRobot& inRobot)
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
    for (unsigned int i=0;i<6;i++)
    {
        attBounder->upperBound(i, 1e10);
        attBounder->lowerBound(i, -1e10);
    }
    attWeights = attComputationWeights = attActive = scalar_vector<double>(attNumParams,1);
    attSolution = zero_vector<double>(attNumParams);
    ElementMask.resize(attNumParams);
}

bool ChppGikSolverRobotFree::weights ( vectorN& inWeights )
{
    if ( inWeights.size() == attNumParams )
    {
        attWeights = inWeights;
        return true;
    }
    else
    {
        std::cout << "ChppGikSolverRobotFree::weights() incorrect size()\n";
        return false;
    }
}

void ChppGikSolverRobotFree::computeConstraintDofs(CjrlGikStateConstraint* inConstraint)
{
    ElementMask = inConstraint->influencingDofs();
}

bool ChppGikSolverRobotFree::solve ( std::vector<CjrlGikStateConstraint*>& inSortedConstraints)
{
    std::vector<double> SRcoefs(inSortedConstraints.size(), 0);
    return solve(inSortedConstraints, SRcoefs);
}

bool ChppGikSolverRobotFree::solve(std::vector<CjrlGikStateConstraint*>& inSortedConstraints, std::vector<double>& inSRcoefs)
{
    attSolution.clear();
    if (inSortedConstraints.empty())
    {
        std::cout << "ChppGikSolverRobotFree::solve() nothing to do\n";
        return true;
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
        attSolver->weights( attComputationWeights );
        attSolver->resetSolution();

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
        for ( iC=0; iC< attNumParams; iC++ )
            CurFullConfig(iC) += attSolver->solution()(iC);
        //joint limit checking
        for ( iC=6; iC< attNumParams; iC++ )
        {
            lb = attRobot->lowerBoundDof(iC);
            ub = attRobot->upperBoundDof(iC);
            if ((attComputationWeights(iC) != 0)&&(CurFullConfig(iC) < lb +1e-2 || CurFullConfig ( iC ) > ub-1e-2))
            {
                recompute = true;
                attComputationWeights(iC) = 0;
            }
        }
    }
    attSolution = attSolver->solution();
    return true;
}

const vectorN& ChppGikSolverRobotFree::solution()
{
    return attSolution;
}

ChppGikSolverRobotFree::~ChppGikSolverRobotFree()
{
    delete attSolver;
    delete attBounder;
}
