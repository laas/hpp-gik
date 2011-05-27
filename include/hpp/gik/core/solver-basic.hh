#ifndef HPP_GIK_SOLVER_BASIC_H
#define HPP_GIK_SOLVER_BASIC_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "gikTask/jrlGikStateConstraint.h"

/**
   \brief This is a prioritized inverse kinematics solver.

   It can be used to solve a sequence of linear systems from highest to lowest
   priority. A linear system is provided by a CjrlGikStateConstraint object
   through its methods jacobian() and value(). These are assumed to be already
   computed.
   <br>
   Each linear system is solved by a pseudo inversion. The weighted pseudo
   inverse is used.

\ingroup solver
*/

class ChppGikSolverBasic
{
public:

    /**
    \brief Constructor
    \param numberParam is the size of the variables vector
     */
    ChppGikSolverBasic(unsigned int numberParam);
    /**
    \brief Set the diagonal weights for the  Weighted Pseudoinverse.

    This calls resetSolution() before returning true.
    \return false if inWeights of incorrect size
     */
    bool weights(vectorN& inWeights);
    /**
    \brief Clear solution

    Reset nullspace and analyze weights to determine which parameters are
    being used and optimize computation accordingly. Call this prior to solving
    the first task of every stack of tasks.
    \return the number of non null weights found.
     */
    unsigned int resetSolution();
    /**
    \brief Set active paramters for nest solved task
    \return false if inMask of incorrect size
     */
    bool setActiveParameters(const vectorN& inMask);
    /**
    \brief solve one constraint in the current null space.

    The solution is added to ChppGikSolver::DeltaQ and ChppGikSolver::NullSpace
    is updated by the following procedure.
    -# get task value \f$v\f$ and jacobian \f$J\f$ from \a inConstraint
    -# compute residual task value: \f$ r \leftarrow v - J\delta q \f$
    -# compute projected jacobian: \f$ \hat{J} \leftarrow JN \f$
    -# singular value decomposition: \f$ S, U, V^t \leftarrow SVD(\hat{J}W\hat{J}^t) \f$
    -# compute inverse:\f$ (\hat{J}W\hat{J}^t)^{-1} \leftarrow US^{-1}V^t \f$
    -# compute pseudo inverse:\f$ \hat{J}^\# \leftarrow W\hat{J}^t(\hat{J}W\hat{J}^t)^{-1}\f$
    -# update solution:\f$ \delta q \leftarrow \delta q + \hat{J}^\#r \f$
    -# update null space:\f$ N \leftarrow N(I-\hat{J}^\#\hat{J}) \f$
    \param inConstraint a constraint to be solved
    \param inSRcoef if non-zero value is given, \f$ (\hat{J}W\hat{J}^t)^{-1} \f$ is computed using SR-Inverse.
    \param projectJacobian set to true to project connstaint jacobian on nullspace
    \param inComputeNullspace set to true to update null space projector
    \return false if task  jacobian's size is not equal to number of parameters
     */
    bool solveTask(CjrlGikStateConstraint *inConstraint, double inSRcoef=0.0, bool projectJacobian = false, bool inComputeNullspace = false);
    /**
    \brief get the solution to last serie of calls of solveTask

    Up to last resetSolution call
     */
    const vectorN& solution();
    /**
    \brief Set the minimum singular value.

    If a singular value is smaller than this value, it is set to zero in the
    computation process of pseudo inverse.
    \param i_threshold the minimum singular value
     */
    void SVDThreshold(double i_threshold);
    /**
    \brief Get Penrose mask vector (debug method)
    \return Penrose mask vector
     */
    const vectorN& penroseMask() const;
    /**
    \brief Destructor
     */
    ~ChppGikSolverBasic();

private:

    unsigned int attNumberParam;
    void resizeMatrices(unsigned int inSubtaskDefaultSize);
    unsigned int LongSize;
    unsigned int numDof;
    unsigned int numJoints;
    unsigned int xDefaultDim;
    unsigned int xDim;
    double      attSVDThreshold;
    unsigned int MaximumIteration;
    vectorN PenroseMask;
    vectorN PIWeights;
    vectorN Weights;
    boost::numeric::ublas::vector<unsigned int> UsedIndexes;
    vectorN CurFullConfig;
    vectorN DeltaQ, attSolution;
    vectorN ElementMask;
    vectorN   Residual;
    vectorN deltaqComplete,valueComplete;
    boost::numeric::ublas::matrix<double,
      boost::numeric::ublas::column_major >IdentityMat;
    boost::numeric::ublas::matrix<double,
      boost::numeric::ublas::column_major > HatJacobian;
    boost::numeric::ublas::matrix<double,
      boost::numeric::ublas::column_major > WJt;
    boost::numeric::ublas::matrix<double,
      boost::numeric::ublas::column_major > JWJt;
    boost::numeric::ublas::matrix<double,
      boost::numeric::ublas::column_major > Jsharp;
    boost::numeric::ublas::matrix<double,
      boost::numeric::ublas::column_major > NullSpace;
    boost::numeric::ublas::matrix<double,
      boost::numeric::ublas::column_major > InverseJWJt;
    boost::numeric::ublas::matrix<double,
      boost::numeric::ublas::column_major > CarvedJacobian;
    vectorN tempS;
    boost::numeric::ublas::matrix<double,
      boost::numeric::ublas::column_major> tempU;
    boost::numeric::ublas::matrix<double,
      boost::numeric::ublas::column_major> tempVt;
    char jobU;
    char jobVt;

};

#endif
