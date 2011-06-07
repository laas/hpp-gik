#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"

#include "hpp/gik/core/solver.hh"
#include "hpp/gik/constraint/configuration-constraint.hh"
#include "hpp/gik/core/motion-plan-element.hh"
#include "hpp/gik/tools.hh"

#include <time.h>
#include <sys/time.h>
#ifdef NDEBUG
#define BOOST_DISABLE_ASSERTS
#endif

#define V3_I  MAL_S3_VECTOR_ACCESS

using namespace boost::numeric::ublas;

void ChppGikSolver::solve ( std::vector<CjrlGikStateConstraint*>& inSortedConstraints )
{
    unsigned int k = inSortedConstraints.size();
    std::vector<double> SRcoefs ( k );
    for ( unsigned int i=0;i<k;i++ ) SRcoefs[i] = 0.0;
    solve ( inSortedConstraints, SRcoefs );
}


void ChppGikSolver::prepare ( std::vector<CjrlGikStateConstraint*>& inTasks )
{
    for ( unsigned int i=0;i<inTasks.size();i++ )
    {
        inTasks[i]->jacobianRoot ( *RootJoint );
        inTasks[i]->computeJacobian();
        inTasks[i]->computeValue();
    }
}

ChppGikSolver::ChppGikSolver ( CjrlDynamicRobot& inRobot )
{
    attRobot = &inRobot;
    attNumParams = inRobot.numberDof();
    attSolution = zero_vector<double> ( attNumParams-6 );
    attFullSolution = zero_vector<double> ( attNumParams );
    attBackupConfig.resize ( attNumParams,false );
    CurFullConfig.resize ( attNumParams,false );
    rootJoint ( * ( attRobot->rootJoint() ) );
    H0 =  attRobot->rootJoint()->currentTransformation();
    solutionRootConfiguration();
    for ( unsigned int i=0; i<3;i++ )
    {
        attFullSolution ( i ) = V3_I ( attRetPair.first,i );
        attFullSolution ( 3+i ) = V3_I ( attRetPair.second,i );
    }
    attChangeRootPose = true; LongSize = attNumParams;
    attChangeRootJoint = false;

    attUb.resize ( attNumParams,false );
    attLb.resize ( attNumParams,false );
    attVUb.resize ( attNumParams,false );attVUb.clear();
    attVLb.resize ( attNumParams,false );attVLb.clear();
    attVeloLambda = 0.5;
    double safetycoef = 0.05, safetyzone;
    double ul,ll;
    for ( unsigned int i =0; i<attNumParams;i++ )
    {
        ul = attRobot->upperBoundDof ( i );
        ll = attRobot->lowerBoundDof ( i );
        safetyzone = safetycoef* ( ul-ll );
        attLb ( i ) = safetyzone + ll;
        attUb ( i ) = ul - safetyzone;
        stored_indices.push_back ( i );
    }
    work_deltas.resize ( attNumParams,false );
    attSolver = new ChppGikSolverLNE ( attNumParams );
    attSolver->workindices ( stored_indices );
    attWeights.resize ( attNumParams );
    attActiveDofs.resize ( attNumParams );
    attExtramove.resize ( attNumParams );
    work_freedofs.resize ( attNumParams );
}

ChppGikSolver::~ChppGikSolver()
{
    delete attSolver;
}

void ChppGikSolver::updateVelocityBounds()
{
    unsigned int ri;
    for ( unsigned int i = 0; i<LongSize;i++ )
    {
        ri = stored_indices[ i ];
        attVUb ( ri ) = attVeloLambda* ( attUb ( ri )-attRobot->currentConfiguration() ( ri ) );
        attVLb ( ri ) = attVeloLambda* ( attLb ( ri )-attRobot->currentConfiguration() ( ri ) );
    }
}

void ChppGikSolver::rootJoint ( CjrlJoint& inRootJoint )
{
    if ( RootJoint != &inRootJoint )
    {
        RootJoint = &inRootJoint;
        supportJoints = RootJoint->jointsFromRootToThis();
    }
    attChangeRootJoint = ( RootJoint!=attRobot->rootJoint() );
}

bool ChppGikSolver::weights ( const vectorN& inWeights )
{
    assert ( inWeights.size() ==attNumParams );

    stored_indices.clear();
    LongSize = 0;
    attChangeRootPose = false;
    attActiveDofs.clear();
    unsigned int i;
    for ( i=0;i<6;i++ )
        if ( inWeights ( i ) > 0 )
        {
            attActiveDofs ( i ) = 1;
            attChangeRootPose = true;
            stored_indices.push_back ( i );
            LongSize++;
        }

    for ( i=6;i<attNumParams;i++ )
        if ( inWeights ( i ) > 0 )
        {
            attActiveDofs ( i ) = 1;
            stored_indices.push_back ( i );
            LongSize++;
        }

    noalias ( attWeights ) = inWeights;
    attSolver->workindices ( stored_indices );
    return true;
}

bool ChppGikSolver::enforceBounds ( vectorN& inActiveDofs, vectorN& prev, const vectorN& extramove,unsigned int& outSaturated )
{
    bool updateVal = false;
    std::vector<double> coefs, ranks;
    unsigned int rankwinner = 0; double coefwinner = 2, npoint;
    unsigned int i,ri;
    for ( i=0; i< LongSize; i++ )
    {
        ri = stored_indices [ i ];
        if ( inActiveDofs ( ri ) == 1 )
        {
            npoint = extramove ( ri ) + prev ( ri ) ;
            if ( npoint > attVUb ( ri ) )
            {
                updateVal = true;
                coefs.push_back ( fabs ( ( attVUb ( ri )- prev ( ri ) ) /extramove ( ri ) ) );
                ranks.push_back ( ri );
            }
            else if ( npoint  < attVLb ( ri ) )
            {
                updateVal = true;

                coefs.push_back ( fabs ( ( attVLb ( ri )- prev ( ri ) ) /extramove ( ri ) ) );
                ranks.push_back ( ri );
            }
        }
    }
    if ( updateVal )
    {

        for ( i=0;i<ranks.size();i++ )
            if ( coefs[i] < coefwinner )
            {
                coefwinner = coefs[i];
                rankwinner = ranks[i];
            }

        prev.plus_assign ( coefwinner*extramove );
        inActiveDofs ( rankwinner ) = 0;
        outSaturated = rankwinner;
        return true;
    }
    else
        prev.plus_assign ( extramove );

    return false;
}

void ChppGikSolver::solve ( std::vector<CjrlGikStateConstraint*>& inTasks, const std::vector<double>& inSRcoefs )
{
    unsigned int nTasks = inTasks.size();
    assert ( inSRcoefs.size() == nTasks );

    //backup
    noalias ( attBackupConfig ) = attRobot->currentConfiguration();
    noalias ( CurFullConfig ) = attRobot->currentConfiguration();
    H0 =  attRobot->rootJoint()->currentTransformation();

    if ( inTasks.empty() )
    {
        std::cout << "ChppGikSolver::solve nothing to do"<<std::endl;
        return;
    }

    unsigned int curTask,rank,ri,j;
    CjrlGikStateConstraint* task;
    unsigned int satDof = 0;
    unsigned int iteration;
    bool modified;
    vectorN value;

    updateVelocityBounds();
    noalias ( work_freedofs ) = attActiveDofs;
    LeftSize = LongSize;

    work_deltas.clear();
    attSolver->resettransform();

    for ( curTask = 0;(curTask<nTasks) && (LeftSize>0);curTask++ )
    {
        if ( curTask!=0 )
            attSolver->updatetransform();
        if ( inTasks[curTask]->dimension() > LongSize - attSolver->constraintdimension() )
            solveOver ( inTasks[curTask],inSRcoefs[curTask],work_freedofs );
        else
        {
            task = inTasks[curTask];
            for ( rank = 0; rank<LongSize;rank++ )
            {
                ri = stored_indices[rank];
                for ( j=0;j<task->dimension();j++ ) attSolver->workmatrix() ( ri,j ) = attWeights ( ri ) * task->jacobian() ( j,ri );
            }
            attSolver->transform ( task->dimension() );
            for ( iteration = 0; iteration<LeftSize; iteration++ )
            {
                if ( ( curTask!=0 ) || ( ( curTask=0 ) && ( iteration!=0 ) ) )
                  {
                    value.resize(task->jacobian().size1());
                    axpy_prod ( task->jacobian(),work_deltas,value, true );
                    noalias ( subrange ( attSolver->workvector(),0,task->dimension() ) ) = -value;
                }
                else
                    noalias ( subrange ( attSolver->workvector(),0,task->dimension() ) ) = -task->value();

                attSolver->solve ( inSRcoefs[curTask] );

                for ( unsigned int ws = 0; ws<LongSize;ws++ )
                    attExtramove ( stored_indices[ws] ) = attWeights ( stored_indices[ws] ) *attSolver->solution() ( stored_indices[ws] );

                modified = enforceBounds ( work_freedofs, work_deltas,attExtramove,  satDof );
                if ( modified )
                {
                    LeftSize--;
                    attSolver->saturateparameter ( satDof );
                }
                else
                    break;
            }
        }
    }
    computeRobotSolution();
}


void ChppGikSolver::solveOver ( CjrlGikStateConstraint* overTask, double overCoef, const vectorN& inFreedofs )
{
    bool specialCase = false;
    ChppGikConfigurationConstraint* confc = dynamic_cast<ChppGikConfigurationConstraint*> ( overTask );
    if ( !confc )
    {
        ChppGikMotionPlanElement* cmpConfc = dynamic_cast<ChppGikMotionPlanElement*> ( overTask );
        if ( cmpConfc  && ( cmpConfc->constraints().size() == 1 ) )
        {
            confc =  dynamic_cast<ChppGikConfigurationConstraint*> ( cmpConfc->constraints() [0] );
            if ( confc )
                specialCase = true;
        }
    }
    else
        specialCase = true;

    if ( specialCase )
    {

        attSolver->workvector().clear();
        unsigned int valInd = 0,li;
        for ( li = 0; ( li<attNumParams ) && ( valInd <  overTask->dimension() );li++ )
            if ( overTask->influencingDofs() ( li ) ==1 )
            {
                attSolver->workvector( ) ( li ) = -overTask->value() ( valInd ) - work_deltas ( li );
                valInd++;
            }
            else
                attSolver->workvector() ( li ) = 0;

        attSolver->calculateresidual();
        unsigned int satDof;
        enforceBounds ( work_freedofs, work_deltas,attSolver->solution(),  satDof );
        LeftSize = 0;
    }
    else
    {
        if ( attSolver->workmatrix().size2() <overTask->dimension() )
        {
            attSolver->workmatrix().resize ( attSolver->workmatrix().size1(),overTask->dimension(),false );
            attSolver->workvector().resize ( overTask->dimension(),false );
        }
        unsigned int j,iteration,ws,satDof,ri;
        for ( unsigned int rank = 0; rank<LongSize;rank++ )
        {
            ri = stored_indices[rank];
            for ( j=0;j<overTask->dimension();j++ ) attSolver->workmatrix() ( ri,j ) = attWeights ( ri ) * overTask->jacobian() ( j,ri );
        }
        attSolver->transform ( overTask->dimension() );
        vectorN value;
        for ( iteration = 0; iteration<LeftSize; iteration++ )
        {

            value.resize(overTask->jacobian().size1());
            axpy_prod ( overTask->jacobian(),work_deltas,value,false );
            noalias ( subrange ( attSolver->workvector(),0,overTask->dimension() ) ) = -value;

            attSolver->solve ( overCoef );

            for ( ws = 0; ws<LongSize;ws++ )
                attExtramove ( stored_indices[ws] ) = attWeights ( stored_indices[ws] ) *attSolver->solution() ( stored_indices[ws] );

            bool modified = enforceBounds ( work_freedofs, work_deltas,attExtramove,  satDof );

            if ( modified )
            {
                LeftSize--;
                attSolver->saturateparameter ( satDof );
            }
            else
                break;
        }
    }
}



const matrix4d& ChppGikSolver::solutionRootPose()
{
    return H0;
}

const vectorN& ChppGikSolver::solutionJointConfiguration()
{
    noalias ( attSolution ) = subrange ( CurFullConfig,6,attNumParams );
    return attSolution;
}

const vectorN& ChppGikSolver::solution()
{
    noalias ( attFullSolution ) = CurFullConfig;
    solutionRootConfiguration();
    for ( unsigned int i=0; i<3;i++ )
    {
        attFullSolution ( i ) = V3_I ( attRetPair.first,i );
        attFullSolution ( 3+i ) = V3_I ( attRetPair.second,i );
    }
    return attFullSolution;
}


void ChppGikSolver::computeRobotSolution()
{

    if ( !attChangeRootJoint && !attChangeRootPose )
        return;

    unsigned int i;

    for ( i =0; i< LongSize; i++ )
        CurFullConfig ( stored_indices [ i ] ) += work_deltas ( stored_indices [ i ] );

    if ( attChangeRootJoint )
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
        MAL_S4x4_INVERSE ( Hf,InvHf,double );
        H0 = Hif * InvHf;
    }

    if ( attChangeRootPose )
    {
        //Compute robot root transformation change due to jacobian root transformation change
        ChppGikTools::Matrix4dFromVec ( work_deltas, HRC );
        if ( attChangeRootJoint )
            H0 = HRC * H0;
        else
        {
            H0 = RootJoint->currentTransformation();
            H0 = HRC * H0;
        }
    }

    if ( attChangeRootJoint )
        //Recover joints transformations from root to fixed joint in waist frame
        for ( i=0; i< supportJoints.size(); i++ )
            supportJoints[i]->updateTransformation ( attBackupConfig );
}

const std::pair<vector3d,vector3d>& ChppGikSolver::solutionRootConfiguration()
{
    ChppGikTools::splitM4 ( H0, TmpR, attRetPair.first );
    ChppGikTools::M3toEulerZYX ( TmpR, attRetPair.second );
    return attRetPair;
}






