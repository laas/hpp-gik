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
    ChppGikSolver(CjrlHumanoidDynamicRobot* inRobot);

    /**
        @}
     */

    /**
        \name Resolution
        @{
     */

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
    CjrlHumanoidDynamicRobot* attRobot;

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
    \brief Update weights according to joint limits.
     */
    void accountForJointLimits();

    /**
        \name Variables used by solve() and allocated in the constructor to avoid dynamic allocation
        @{
     */

    unsigned int LongSize;
    unsigned int numDof;
    unsigned int numJoints;
    unsigned int xDefaultDim;
    unsigned int xDim;
    unsigned int xDim2;
    unsigned int Iteration;
    double       ValueNorm;
    bool         SatisfactorySolution;
    CjrlJoint*  FixedJoint;
    double      SVDThreshold;
    double      ErrorThresh;
    double      BrakingZone;
    double      WindowStep;

    unsigned int MaximumIteration;
    vectorN PenroseMask;
    vectorN PIWeights;
    vectorN Weights;
    vectorN PIWeightsBackup;
    vectorN JointUpperLimitWindow;


    ublas::vector<unsigned int> UsedIndexes;

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
