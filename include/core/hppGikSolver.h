#ifndef HPP_GIK_SOLVER_H
#define HPP_GIK_SOLVER_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
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
    The last vector contains the damping factors for solving the corresponding constraints. 0.01 should be a good filling value for this vector.
    \return false if a fixed joint is not set in the robot.
     */
    bool gradientStep(std::vector<CjrlGikStateConstraint*>& inSortedConstraints, std::vector<double>& inSRcoefs );
    
    /**
    \brief get the solution to last call of gradientStep
    */
    const vectorN& solution();

    /**
       \brief variant of above gradientStep(). Zero-vector is passed to inSRcoefs.
     */
    bool gradientStep(std::vector<CjrlGikStateConstraint*>& inSortedConstraints);

    /**
       \brief Set the minimum singular value. If a singular value is smaller than this value, it is set to zero in the computation process of pseudo inverse. 
       \param i_threshold the minimum singular value
     */
    void SVDThreshold(double i_threshold) { attSVDThreshold = i_threshold;}

    /**
       \brief Get Penrose mask vector
       \return Penrose mask vector
    */
    const vectorN& penroseMask() const { return PenroseMask; }

    /**
        @}
     */

    /**
        \brief Destructor
     */
    ~ChppGikSolver();

private:


    CjrlDynamicRobot* attRobot;
    void resizeMatrices(unsigned int inSubtaskDefaultSize);
    void prepareBrakeWindow();
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
       \param inSRcoef if non-zero value is given, \f$ (\hat{J}W\hat{J}^t)^{-1} \f$ is computed using SR-Inverse.
    */
    void solveOneConstraint(CjrlGikStateConstraint *inConstraint, 
                            double inSRcoef=0.0, bool computeHatJacobian = true, bool inComputeNullspace = true);

    /**
    \brief add supporting leg's dofs to constraint's dofs
    */
    void computeConstraintDofs(CjrlGikStateConstraint* inConstraint);


    unsigned int LongSize;
    unsigned int LongSizeBackup;
    unsigned int numDof;
    unsigned int numJoints;
    unsigned int xDefaultDim;
    unsigned int xDim;
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


    boost_ublas::vector<unsigned int> UsedIndexes;
    boost_ublas::vector<unsigned int> NextUsedIndexes;
    boost_ublas::vector<unsigned int> UsedIndexesBackup;

    matrixNxP H0;
    matrixNxP Hif;
    matrixNxP Hf;
    matrixNxP InvHf;
    vectorN BaseEuler;

    vectorN CurFullConfig;
    vectorN DeltaQ;
    vectorN ElementMask;
    
    std::vector<CjrlJoint*> supportJoints;
    std::vector<unsigned int> supportJointsRanks;

    vectorN   Residual;
    boost_ublas::matrix<double, boost_ublas::column_major > IdentityMat;
    boost_ublas::matrix<double, boost_ublas::column_major > HatJacobian;
    boost_ublas::matrix<double, boost_ublas::column_major > WJt;
    boost_ublas::matrix<double, boost_ublas::column_major > JWJt;
    boost_ublas::matrix<double, boost_ublas::column_major > Jsharp;
    boost_ublas::matrix<double, boost_ublas::column_major > NullSpace;
    boost_ublas::matrix<double, boost_ublas::column_major > InverseJWJt;
    boost_ublas::matrix<double, boost_ublas::column_major > CarvedJacobian;

    
    vectorN tempS;
    boost_ublas::matrix<double, boost_ublas::column_major> tempU;
    boost_ublas::matrix<double, boost_ublas::column_major> tempVt;

    char jobU;
    char jobVt;
    
    unsigned int Offset;
};

#endif
