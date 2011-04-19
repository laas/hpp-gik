#ifndef GIK_LNE_SOLVER_H
#define GIK_LNE_SOLVER_H

#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"

class ChppGikSolverLNE
{
    public:
        ChppGikSolverLNE ( unsigned int ndof );
        void workindices ( const std::vector<unsigned int>& inindices );
        matrixNxP& workmatrix(){return work_matrix;};
        vectorN& workvector(){return work_vector;};
        void transform(unsigned int inDim);
        void solve ( double inLambda );
        void updatetransform();
        void saturateparameter(unsigned int iDof);
        void resettransform();
        unsigned int constraintdimension();
        void calculateresidual ();
        const vectorN& solution();

    private:
        void qtrunc ( matrixNxP&  M, vectorN& tau, unsigned int longsize, unsigned int m,  unsigned int startrow, const std::vector<unsigned int>& indices, unsigned int& outDim );
        void Qb ( const matrixNxP&  M, unsigned int rowOffset, unsigned int longsize, unsigned int columnOffset,  unsigned int dimQ, const std::vector<unsigned int>& indices, const vectorN& tau,  vectorN& outX );
        void QtM ( const matrixNxP&  M, const vectorN& tau , unsigned int dimQ, const std::vector<unsigned int>& indices, matrixNxP&  outMat,  unsigned int dimOut, unsigned int longsize );
        void trisolveT ( const matrixNxP&  inQR, const std::vector<unsigned int>& indices, unsigned int startrow, const vectorN& inb, unsigned int dim, vectorN& outY );
        void qrtrunc ( matrixNxP&  M, vectorN& tau, vectorN& outVal, unsigned int& outDim,  unsigned int startrow, const std::vector<unsigned int>& indices, unsigned int longsize, unsigned int m );
        void utv ( matrixNxP&  M, unsigned int rowu, unsigned int colu, unsigned int maxcol, unsigned int longsize, double gamma, const std::vector<unsigned int>& indices );
        void Qtb ( const matrixNxP&  M, const vectorN& tau , unsigned int dimQ, const std::vector<unsigned int>& indices, vectorN&  outX, unsigned int longsize, bool special );
        void HupdM ( const matrixNxP& M, const vectorN& tau, unsigned int rank, const std::vector<unsigned int>& indices,  unsigned int longsize, matrixNxP& outMat, unsigned int  dimOut );
        void loadundermatrix(double inLambda);
        bool work_damped;
        double attQRThreshold;
        unsigned int stored_ndof, work_dim, stored_eqdim,stored_longsize,stored_dimQtM,work_nindices,work_length;
        matrixNxP stored_qr, work_qrAt, work_matrix, stored_QtM;
        vectorN stored_tau, work_x, work_tauA, work_z, work_ya, work_vector,work_subs;
        std::vector<unsigned int> stored_indices,work_indices;
        vectorN unitVec;
};

#endif
