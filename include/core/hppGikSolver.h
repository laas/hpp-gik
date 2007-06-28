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
    \brief Compute a configuration complying with the stack of constraitns.
    This uses the prioritized inverse kinematiks framework (ref Prof.Nakamura)
    \param inWeights is a vector of length robot.numberDof()-6 used in the formula W^{-1}Jt(JJt)^{-1} which gives the weighted pseudoinverse of a constraint jacobian
        If the provided parameter has a bad size, an all-one vector is used instead, giving no particular preference to any joint
    \param inJointsMask is also a vectorN of length robot.numberDof()-6. It defines the set of joints to be used (1= used, 0= not used)
    \param inMaxIter is the maxmum number of iterations of the algorithm
    \return false if the robot did not have a fixed joint.
     */
    bool solve(std::vector<CjrlGikStateConstraint*>& inSortedConstraints, vectorN& inWeights,vectorN& inJointsMask, unsigned int inMaxIter=1);

    /**
    \brief Get a vector containing the ranks of unsolved constraints. A constraint is considered unsolved if its value is inferior to the threshold ErrorThresh.
    */
    void getUnsolvedConstraints(std::vector<CjrlGikStateConstraint*>& inConstraintStack, std::vector<unsigned int>& outUnsolved, double inThreshold);
            
    /**
        \brief Get the computed configuration of the robot.
     */
    const vectorN& solutionConfiguration();
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
        \brief Output of solve(): configuration complying with the task stack.
     */
    vectorN attSolutionConfig;
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
        \name Variables used by solve() and allocated in the constructor to avoid dynamic allocation
        @{
     */

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
    vectorN PIWeightsBackup;
    vectorN JointMask;
    vectorN JointUpperLimitWindow;


    ublas::vector<unsigned int> UsedIndexes;
    std::vector<unsigned int> UnsolvedRanks;

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
