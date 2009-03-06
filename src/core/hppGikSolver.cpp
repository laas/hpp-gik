#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "core/hppGikSolver.h"
#include "hppGikTools.h"

#define V3_I  MAL_S3_VECTOR_ACCESS

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
    attSolution = zero_vector<double>(attNumParams-6);
    attFullSolution = zero_vector<double>(attNumParams);
    rootJoint(*(attRobot->rootJoint()));
    
    H0 =  attRobot->rootJoint()->currentTransformation();

    solutionRootConfiguration();
    for(unsigned int i=0; i<3;i++)
    {
        attFullSolution(i) = V3_I(attRetPair.first,i);
        attFullSolution(3+i) = V3_I(attRetPair.second,i);
    }
    
    attChangeRootPose = true;
    attChangeRootJoint = false;
}

void ChppGikSolver::rootJoint(CjrlJoint& inRootJoint)
{
    if (RootJoint != &inRootJoint)
    {
        RootJoint = &inRootJoint;
        supportJoints = RootJoint->jointsFromRootToThis();
    }
    attChangeRootJoint = (RootJoint!=attRobot->rootJoint());
}

bool ChppGikSolver::weights ( vectorN& inWeights )
{
    if ( inWeights.size() == attNumParams )
    {
        attWeights = inWeights;
        attChangeRootPose = false;
        for (unsigned int i=0;i<6;i++)
            if (attWeights(i)!=0)
            {
                attChangeRootPose = true;
                break;
            }
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

void ChppGikSolver::solve(std::vector<CjrlGikStateConstraint*>& inSortedConstraints, const std::vector<double>& inSRcoefs)
{
    attBackupConfig = attRobot->currentConfiguration();
    if (inSortedConstraints.empty())
    {
        std::cout << "ChppGikSolver::solve() nothing to do"<<std::endl;
        CurFullConfig = attRobot->currentConfiguration();
        H0 =  attRobot->rootJoint()->currentTransformation();
        return;
    }

    bool recompute = true;
    unsigned int iC;
    double ub, lb;
    std::vector<CjrlGikStateConstraint*>::iterator iter;
    std::vector<double>::const_iterator iter2;

    attVals = attRobot->currentConfiguration();
    attRate = attRobot->currentVelocity();
    attComputationWeights = attWeights;
    attBounder->modifyWeights( attVals, attRate, attComputationWeights );

    while ( recompute )
    {
        CurFullConfig = attRobot->currentConfiguration();
        
        if (!(attSolver->weights( attComputationWeights )))
        {
            H0 =  attRobot->rootJoint()->currentTransformation();
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
    
    computeRobotSolution();
}


const matrix4d& ChppGikSolver::solutionRootPose()
{
    return H0;
}

const vectorN& ChppGikSolver::solutionJointConfiguration()
{
    attSolution = subrange(CurFullConfig,6,attNumParams);
    return attSolution;
}

const vectorN& ChppGikSolver::solution()
{
    attFullSolution = CurFullConfig;
    solutionRootConfiguration();
    for(unsigned int i=0; i<3;i++)
    {
        attFullSolution(i) = V3_I(attRetPair.first,i);
        attFullSolution(3+i) = V3_I(attRetPair.second,i);
    }
    return attFullSolution;
}


void ChppGikSolver::computeRobotSolution()
{
    CurFullConfig = attBackupConfig;
    CurFullConfig.plus_assign(attSolver->solution());

    
    if (!attChangeRootJoint && !attChangeRootPose)
    {
        H0 =  RootJoint->currentTransformation();
        return;
    }
    
    unsigned int i;
    if (attChangeRootJoint)
    {
        Hif = RootJoint->currentTransformation();
        //Let everything be expressed in robot root frame
        for ( i =0; i< 6; i++ )
            CurFullConfig ( i ) = 0.0;

        //update joints transformations from robot root to jacobian root
        for ( i=0; i< supportJoints.size(); i++ )
            supportJoints[i]->updateTransformation ( CurFullConfig );

        //Compute robot root transformation change due to chain of joints between robot root and jacobian root
        Hf = RootJoint->currentTransformation();
        MAL_S4x4_INVERSE(Hf,InvHf,double);
        H0 = Hif * InvHf;
    }

    if (attChangeRootPose)
    {
        //Compute robot root transformation change due to jacobian root transformation change
        ChppGikTools::Matrix4dFromVec(attSolver->solution(), HRC);
        if (attChangeRootJoint)
            H0 = HRC * H0;
        else
        {
            H0 = RootJoint->currentTransformation();
            H0 = HRC * H0;
        }
    }

    if (attChangeRootJoint)
        //Recover joints transformations from root to fixed joint in waist frame
        for ( i=0; i< supportJoints.size(); i++ )
            supportJoints[i]->updateTransformation ( attBackupConfig );
}

const std::pair<vector3d,vector3d>& ChppGikSolver::solutionRootConfiguration()
{
    ChppGikTools::splitM4(H0, TmpR, attRetPair.first);
    ChppGikTools::M3toEulerZYX( TmpR, attRetPair.second);
    return attRetPair;
}

ChppGikSolver::~ChppGikSolver()
{
    delete attSolver;
    delete attBounder;
}
