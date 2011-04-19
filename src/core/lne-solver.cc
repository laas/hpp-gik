#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/core/lne-solver.hh"
#include "hpp/gik/tools.hh"
#include <time.h>
#include <sys/time.h>
#ifdef NDEBUG
#define BOOST_DISABLE_ASSERTS
#endif
#define min(a,b) (a<b)?a:b
#define V3_I  MAL_S3_VECTOR_ACCESS

using namespace boost::numeric::ublas;

void ChppGikSolverLNE::workindices ( const std::vector<unsigned int>& inindices )
{
    resettransform();
    stored_indices = inindices;
    stored_longsize = stored_indices.size();
}


ChppGikSolverLNE::ChppGikSolverLNE ( unsigned int ndof )
{

    attQRThreshold = 1e-5;
    stored_ndof = ndof;
    stored_qr.resize ( stored_ndof,stored_ndof );
    stored_QtM.resize ( stored_ndof,stored_ndof );
    stored_tau.resize ( stored_ndof );
    work_x.resize ( stored_ndof );

    work_dim = 0;
    work_x.clear();
    stored_eqdim = 0;
    work_damped = false;
    stored_indices.clear();
    for ( unsigned int i=0;i<stored_ndof;i++ )
        stored_indices.push_back ( i );
    stored_longsize = stored_indices.size();
    work_matrix.resize ( stored_ndof+stored_ndof,stored_ndof,false );
    work_vector.resize ( stored_ndof,false );
    work_ya.resize ( stored_ndof,false );
    work_tauA.resize ( stored_ndof,false );
    work_z.resize ( stored_ndof+stored_ndof,false );
    work_subs.resize ( stored_ndof );
    unitVec.resize ( stored_ndof );
    stored_dimQtM = 0;
}

void ChppGikSolverLNE::transform ( unsigned int inDim )
{
    stored_dimQtM = inDim;

    if (stored_QtM.size2() < stored_dimQtM)
        stored_QtM.resize(stored_QtM.size1(),stored_dimQtM,false);
    
    noalias ( subrange ( stored_QtM,0,stored_ndof,0,stored_dimQtM ) ) = subrange ( work_matrix,0,stored_ndof,0,stored_dimQtM );

    if ( stored_eqdim != 0 )
        QtM ( stored_qr, stored_tau, stored_eqdim, stored_indices, stored_QtM, stored_dimQtM, stored_longsize );

}

unsigned int ChppGikSolverLNE::constraintdimension()
{
    return stored_eqdim;
}


void ChppGikSolverLNE::loadundermatrix ( double inLambda )
{
    work_length = stored_ndof;
    work_damped = false;
    if ( inLambda>0 )
    {
        work_damped = true;
        work_length += work_dim;
    }

    unsigned int i,j;
    for ( i=stored_eqdim;i<stored_longsize;i++ )
        for ( j=0; j < work_dim; j++ )
            work_matrix ( stored_indices[i],j ) = stored_QtM ( stored_indices[i],j );

    //prepare (Q_N^T * A^T)_lambda
    work_indices = stored_indices;
    if ( work_damped )
    {
        noalias ( subrange ( work_matrix,stored_ndof,work_length,0,work_dim ) ) = zero_matrix<double> ( work_dim,work_dim );
        for ( i=stored_ndof;i<work_length;i++ )
        {
            work_matrix ( i,i-stored_ndof ) = inLambda;
            work_indices.push_back ( i );
        }
    }
    work_nindices = work_indices.size();
}


void ChppGikSolverLNE::solve ( double inLambda )
{
    double damping_factor = inLambda;
    
    work_dim= stored_dimQtM;
    unsigned int expected_dim = stored_eqdim + work_dim;

    
    if ( expected_dim > stored_longsize )
    {
        damping_factor += 0.0;
        if (( work_dim > work_matrix.size2()) || (stored_ndof+work_dim > work_matrix.size1()))
            work_matrix.resize ( stored_ndof+work_dim,work_dim,false );
    }

    loadundermatrix ( damping_factor );

    work_tauA.clear();
    //factorize (Q_N^T * A^T)_lambda -> (Q,R)
    qrtrunc ( work_matrix, work_tauA, work_vector, work_dim,  stored_eqdim, work_indices, work_nindices, work_dim );
    if ( work_dim == 0 )
    {
        work_x.clear();
        return;
    }

    expected_dim = work_dim+stored_eqdim;

    //calculate y = R^{-T} * b
    work_ya.clear();
    trisolveT ( work_matrix,work_indices, stored_eqdim, work_vector, work_dim, work_ya );
    //calculate z = Q^T y
    work_z.clear();
    unsigned int maxcol = min(expected_dim,stored_longsize);
    for ( unsigned int k=stored_eqdim;k<maxcol;k++ )
        work_z ( stored_indices[k] ) = work_ya ( k-stored_eqdim );
    Qb ( work_matrix, stored_eqdim, work_nindices, 0, work_dim, work_indices, work_tauA, work_z );
    //calculate  Q_N z
    Qb ( stored_qr, 0, stored_longsize, 0, stored_eqdim, stored_indices, stored_tau, work_z );
    //calculate  x = x + Q_N z
    noalias ( work_x ) = subrange ( work_z,0,stored_ndof );
}

void ChppGikSolverLNE::calculateresidual ()
{
    noalias ( work_subs ) = subrange ( work_vector,0,stored_ndof );
    noalias ( work_x ) = work_subs;
    Qtb ( stored_qr, stored_tau, stored_eqdim, stored_indices, work_subs, stored_longsize,false );
    for ( unsigned int zi=stored_eqdim;zi<stored_longsize;zi++ ) work_subs ( stored_indices[zi] ) = 0;
    Qb ( stored_qr, 0, stored_longsize, 0, stored_eqdim, stored_indices, stored_tau, work_subs );
    work_x.minus_assign ( work_subs );
}

void ChppGikSolverLNE::saturateparameter ( unsigned int iDof )
{
    if ( stored_eqdim==stored_longsize )
        return;

    unsigned int k;
    if ( stored_eqdim==0 )
    {
        unitVec.clear();
        unitVec ( iDof ) = 1;
    }
    else
    {
        double ip;
        if ( iDof == stored_indices[0] )
            ip = -stored_tau ( 0 );
        else
            ip = -stored_tau ( 0 ) *stored_qr ( iDof,0 );
        unitVec ( stored_indices[0] ) = ip;
        for ( k=1;k<stored_longsize;k++ )
            unitVec ( stored_indices[k] ) = ip*stored_qr ( stored_indices[k],0 );
        unitVec ( iDof ) += 1;
        Qtb ( stored_qr, stored_tau, stored_eqdim, stored_indices, unitVec, stored_longsize,true );
    }
    for ( k=stored_eqdim;k<stored_longsize;k++ ) stored_qr ( stored_indices[k],stored_eqdim ) = unitVec ( stored_indices[k] );

    qtrunc ( stored_qr, stored_tau, stored_longsize, 1, stored_eqdim, stored_indices, work_dim );

    if ( work_dim==1 )
    {
        HupdM ( stored_qr,stored_tau,stored_eqdim,stored_indices, stored_longsize,stored_QtM,stored_dimQtM );
        stored_eqdim++;
        work_dim = 0;
    }
}

void ChppGikSolverLNE::updatetransform()
{
    if ( stored_eqdim==stored_longsize )
        return;
    if ( work_dim!=0 )
    {
        unsigned int ndim=work_dim+stored_eqdim;
        if (stored_qr.size2() < ndim)
            stored_qr.resize(stored_ndof,ndim,true);
        if (work_damped)
        {
            noalias ( subrange ( stored_qr,0,stored_ndof,stored_eqdim,ndim ) ) = subrange ( stored_QtM,0,stored_ndof,0,stored_dimQtM );
            qtrunc ( stored_qr, stored_tau, stored_longsize, stored_dimQtM, stored_eqdim, stored_indices, work_dim );
        }
        else
        {
            ndim = min(ndim,stored_longsize);
            work_dim = ndim-stored_eqdim;
            noalias ( subrange ( stored_qr,stored_eqdim,stored_ndof,stored_eqdim,ndim ) ) = subrange ( work_matrix,stored_eqdim,stored_ndof,0,work_dim );
            noalias ( subrange ( stored_tau,stored_eqdim,work_dim+stored_eqdim ) ) = subrange ( work_tauA,0,work_dim );
        }
        stored_eqdim += work_dim;
        work_dim = 0;
    }
}

void ChppGikSolverLNE::resettransform()
{
    work_dim = 0;
    stored_eqdim = 0;
    work_damped = false;
    stored_dimQtM = 0;
}

const vectorN& ChppGikSolverLNE::solution()
{
    return work_x;
}

void ChppGikSolverLNE::utv ( matrixNxP&  M, unsigned int rowu, unsigned int colu, unsigned int maxcol, unsigned int longsize, double gamma, const std::vector<unsigned int>& indices )
{
    unsigned int k,r;
    double ip;
    for ( k=colu+1;k<maxcol;k++ )
    {
        ip = M ( indices[rowu],k );
        for ( r=rowu+1;r<longsize;r++ )
            ip+= M ( indices[r],colu ) * M ( indices[r],k );
        ip *=gamma;
        M ( indices[rowu],k ) -= ip;
        for ( r=rowu+1;r<longsize;r++ )
            M ( indices[r],k ) -= ip*M ( indices[r],colu );
    }
}

void ChppGikSolverLNE::qrtrunc ( matrixNxP&  M, vectorN& tau, vectorN& outVal, unsigned int& outDim,  unsigned int startrow, const std::vector<unsigned int>& indices, unsigned int longsize, unsigned int m )
{
    unsigned int writecol = 0, writerow = startrow;
    unsigned int row = 0;
    double normx=0,norm2x=0, u1=0;

    while ( writecol< m )
    {
        norm2x = 0;
        writerow = startrow+writecol;
        for ( row=writerow;row<longsize;row++ )
            norm2x += M ( indices[row],writecol ) *M ( indices[row],writecol );
        if ( norm2x<attQRThreshold )
        {
            subrange ( M,0,indices[longsize-1]+1,writecol,m-1 ) = subrange ( M,0,indices[longsize-1]+1,writecol+1,m );
            subrange ( outVal,writecol,m-1 ) = subrange ( outVal,writecol+1,m );
            m--;
            continue;
        }
        normx = sqrt ( norm2x );
        if ( M ( indices[writerow],writecol ) >0 )
        {
            u1 =  M ( indices[writerow],writecol ) + normx;
            tau ( writecol ) = u1/normx;
            M ( indices[writerow],writecol ) = -normx;
        }
        else
        {
            u1 =  M ( indices[writerow],writecol ) - normx;
            tau ( writecol ) = -u1/normx;
            M ( indices[writerow],writecol ) = normx;
        }

        for ( row=writerow+1;row<longsize;row++ )
            M ( indices[row],writecol ) /=  u1;
        utv ( M, writerow, writecol, m, longsize, tau ( writecol ), indices );

        writecol++;
    }
    outDim = writecol;
}

void ChppGikSolverLNE::trisolveT ( const matrixNxP&  inQR, const std::vector<unsigned int>& indices, unsigned int startrow, const vectorN& inb, unsigned int dim, vectorN& outY )
{
    unsigned int row = 0, col = 0;
    outY ( row ) = inb ( row ) /inQR ( indices[startrow],row );
    row++;
    for (;row<dim;row++ )
    {
        outY ( row ) = inb ( row );
        for ( col=0;col<row;col++ )
            outY ( row ) -= inQR ( indices[startrow+col],row ) *outY ( col );
        outY ( row ) /= inQR ( indices[startrow+row],row );
    }
}

void ChppGikSolverLNE::QtM ( const matrixNxP&  M, const vectorN& tau , unsigned int dimQ, const std::vector<unsigned int>& indices, matrixNxP&  outMat,  unsigned int dimOut, unsigned int longsize )
{
    unsigned int outcol,r;
    unsigned int rank=0;
    double ip;
    while ( rank<dimQ )
    {
        for ( outcol = 0; outcol<dimOut;outcol++ )
        {
            ip = outMat ( indices[rank],outcol );
            for ( r = rank+1;r<longsize;r++ )
                ip+=M ( indices[r],rank ) *outMat ( indices[r],outcol );

            ip*=tau ( rank );

            outMat ( indices[rank],outcol ) -= ip;
            for ( r = rank+1;r<longsize;r++ )
                outMat ( indices[r],outcol ) -= ip*M ( indices[r],  rank );
        }
        rank++;
    }
}

void ChppGikSolverLNE::Qtb ( const matrixNxP&  M, const vectorN& tau , unsigned int dimQ, const std::vector<unsigned int>& indices, vectorN&  outX, unsigned int longsize,bool special )
{
    unsigned int r;
    unsigned int rank;
    double ip;
    if ( special )
        rank = 1;
    else
        rank =0;

    while ( rank<dimQ )
    {
        ip = outX ( indices[rank] );
        for ( r = rank+1;r<longsize;r++ )
            ip+=M ( indices[r],rank ) *outX ( indices[r] );

        ip*=tau ( rank );
        outX ( indices[rank] ) -= ip;
        for ( r = rank+1;r<longsize;r++ )
            outX ( indices[r] ) -= ip*M ( indices[r],  rank );

        rank++;
    }
}

void ChppGikSolverLNE::Qb ( const matrixNxP&  M, unsigned int rowOffset, unsigned int longsize, unsigned int columnOffset,  unsigned int dimQ, const std::vector<unsigned int>& indices, const vectorN& tau,  vectorN& outX )
{
    int counter = dimQ-1;
    int col = columnOffset+dimQ-1;
    int row = rowOffset+dimQ-1;
    unsigned int r;
    double ip;

    while ( counter >-1 )
    {
        ip = outX ( indices[row] );
        for ( r=row+1;r<longsize;r++ )
            ip+=M ( indices[r],col ) *outX ( indices[r] );
        ip*=tau ( counter );

        outX ( indices[row] ) -= ip;
        for ( r=row+1;r<longsize;r++ )
            outX ( indices[r] ) -= ip*M ( indices[r],col );

        row--;
        col--;
        counter--;
    }
}

void ChppGikSolverLNE::qtrunc ( matrixNxP&  M, vectorN& tau, unsigned int longsize, unsigned int m,  unsigned int startrow, const std::vector<unsigned int>& indices, unsigned int& work_dim )
{
    unsigned int writecol = startrow, writerow = startrow;
    unsigned int row = 0;
    double normx=0,norm2x=0, u1=0;

    work_dim = 0;

    unsigned int maxcol = startrow+m;
    while ( writecol<maxcol && writecol<longsize)
    {
        norm2x = 0;
        for ( row=writerow;row<longsize;row++ )
            norm2x += M ( indices[row],writecol ) *M ( indices[row],writecol );
        if ( norm2x<attQRThreshold )
        {
            subrange ( M,indices[startrow],indices[longsize-1]+1,writecol,maxcol-1 ) = subrange ( M,indices[startrow],indices[longsize-1]+1,writecol+1,maxcol );
            maxcol--;
            continue;
        }

        normx = sqrt ( norm2x );
        if ( M ( indices[writerow],writecol ) >0 )
        {
            u1 =  M ( indices[writerow],writecol ) + normx;
            tau ( writecol ) = u1/normx;
            M ( indices[writerow],writecol ) = -normx;
        }
        else
        {
            u1 =  M ( indices[writerow],writecol ) - normx;
            tau ( writecol ) = -u1/normx;
            M ( indices[writerow],writecol ) = normx;
        }

        for ( row=writerow+1;row<longsize;row++ )
            M ( indices[row],writecol ) /= u1;
        utv ( M, writerow, writecol, maxcol, longsize, tau ( writecol ), indices );

        writecol++;
        writerow++;
        work_dim++;
    }
}

void ChppGikSolverLNE::HupdM ( const matrixNxP& M, const vectorN& tau, unsigned int rank, const std::vector<unsigned int>& indices,  unsigned int longsize, matrixNxP& outMat, unsigned int  dimOut )
{
    unsigned int outcol,r;
    double ip;
    for ( outcol = 0; outcol<dimOut;outcol++ )
    {
        ip = outMat ( indices[rank],outcol );
        for ( r = rank+1;r<longsize;r++ )
            ip+=M ( indices[r],rank ) *outMat ( indices[r],outcol );
        ip*=tau ( rank );
        outMat ( indices[rank],outcol ) -= ip;
        for ( r = rank+1;r<longsize;r++ )
            outMat ( indices[r],outcol ) -= ip*M ( indices[r],  rank );
    }
}
