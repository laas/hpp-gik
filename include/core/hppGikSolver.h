#ifndef HPP_GIK_SOLVER_H
#define HPP_GIK_SOLVER_H

#include "gikTask/jrlGikStateConstraint.h"


/**
\brief This is a prioritized inverse kinematics solver. It produces a robot configuration complying with a vector of high-to-low-priority jrlGikStateConstraints
 */
class ChppGikSolver
{
public:
    /**
        \name Definition
        @{
     */
    /**
        \brief Constructor
     */
    ChppGikSolver(CjrlDynamicRobot* inRobot);

    /**
        @}
     */

    /**
        \name Resolution
        @{
     */

    /**
    \brief Update weights according to joint limits.
     */
    void accountForJointLimits();

    /**
    \brief set pseudo inverse weights according to current configuration and entered weights. A 0 weight indicates a disactivated joint. The given vector must be of size (robot.numberDofs - 6).
    The default vector (on the construction of an instance) is an all-one vector.
    \return false if inWeights of incorrect size
    */
    bool weights(vectorN& inWeights);

    /**
    \brief perform a single gradient descent step on the given constraints (ordered from the most to the least prioriary constraint). Constraints jacobians and values computations are left to the user. The result of the gradient descent, if any, is directly applied to the robot.
    The last weights vector set is used.
    \return false if a fixed joint is not set in the robot.
     */
    bool gradientStep(std::vector<CjrlGikStateConstraint*>& inSortedConstraints);

    /**
       \brief Set the minimum singular value. If a singular value is smaller than this value, it is set to zero in the computation process of pseudo inverse. 
       \param i_threshold the minimum singular value
     */
    void SVDThreshold(double i_threshold) { attSVDThreshold = i_threshold;}

    /**
        @}
     */

    /**
        \brief Destructor
     */
    ~ChppGikSolver();

private:

    /**
        \brief Pointer to the relevant robot.
     */
    CjrlDynamicRobot* attRobot;

    /**
        \brief Resize matrices used in solve().
        Default maximum size of subtask is assumed to be 6. If a subtask has a bigger dimension, matrices get resized internally by this method in order to match the subtask. They keep the latest allocated size. For now there is no mecanism to shrink the matrices back when subtasks are smaller.
     */
    void resizeMatrices(unsigned int inSubtaskDefaultSize);

    /**
        \brief Prepare smooth braking window for joint limits enforcement
     */
    void prepareBrakeWindow();

    /**
        \brief get dof braking coefficient to be applied in avoiding a joint limit
     */
    double brakeCoefForJoint(const double& qVal,const double& lowerLimit, const double& upperLimit, const double& dq);


    /**
       \brief solve one constraint in the current null space. The solution is added to ChppGikSolver::DeltaQ and ChppGikSolver::NullSpace is updated by the following procedure.
       -# get task value \f$v\f$ and jacobian \f$J\f$ from \a inConstraint
       -# compute residual task value: \f$ r \leftarrow v - J\delta q \f$
       -# compute projected jacobian: \f$ \hat{J} \leftarrow JN \f$
       -# singular value decomposition: \f$ S, U, V^t \leftarrow SVD(\hat{J}W\hat{J}^t) \f$
       -# compute inverse:\f$ (\hat{J}W\hat{J}^t)^{-1} \leftarrow US^{-1}V^t \f$
       -# compute pseudo inverse:\f$ \hat{J}^\# \leftarrow W\hat{J}^t(\hat{J}W\hat{J}^t)^{-1}\f$
       -# update solution:\f$ \delta q \leftarrow \delta q + \hat{J}^\#r \f$
       -# update null space:\f$ N \leftarrow N(I-\hat{J}^\#\hat{J}) \f$
       \param inConstraint a constraint to be solved
    */
    void solveOneConstraint(CjrlGikStateConstraint *inConstraint);

    /**
        \name Variables used by solve() and allocated in the constructor to avoid dynamic allocation
        @{
     */

    unsigned int LongSize;
    unsigned int LongSizeBackup;
    unsigned int numDof;
    unsigned int numJoints;
    unsigned int xDefaultDim;
    unsigned int xDim;
    unsigned int xDim2;
    unsigned int Iteration;
    double       ValueNorm;
    bool         SatisfactorySolution;
    CjrlJoint*  FixedJoint;
    double      attSVDThreshold;
    double      BrakingZone;
    double      WindowStep;

    unsigned int MaximumIteration;
    vectorN PenroseMask;
    vectorN PIWeights;
    vectorN NextPIWeights;
    vectorN Weights;
    vectorN PIWeightsBackup;
    vectorN JointUpperLimitWindow;


    ublas::vector<unsigned int> UsedIndexes;
    ublas::vector<unsigned int> NextUsedIndexes;
    ublas::vector<unsigned int> UsedIndexesBackup;
    

    matrixNxP H0;
    matrixNxP Hif;
    matrixNxP Hf;
    matrixNxP InvHf;
    vectorN BaseEuler;

    vectorN CurFullConfig;
    vectorN DeltaQ;


    vectorN   Residual;
    ublas::matrix<double, ublas::column_major > IdentityMat;
    ublas::matrix<double, ublas::column_major > BigMat1;
    ublas::matrix<double, ublas::column_major > BigMat2;
    ublas::matrix<double, ublas::column_major > HatJacobian;
    ublas::matrix<double, ublas::column_major > WJt;
    ublas::matrix<double, ublas::column_major > JWJt;
    ublas::matrix<double, ublas::column_major > Jsharp;
    ublas::matrix<double, ublas::column_major > NullSpace;
    ublas::matrix<double, ublas::column_major > InverseJWJt;
    ublas::matrix<double, ublas::column_major > CarvedJacobian;


    char jobU;
    char jobVt;
    /**
        @}
     */
};

#endif
