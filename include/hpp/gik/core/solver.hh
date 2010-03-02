#ifndef HPP_GIK_SOLVER_H
#define HPP_GIK_SOLVER_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "core/hppGikSolverBasic.h"
#include "core/hppGikBounder.h"

/**
\brief this object is a prioritized inverse kinematics for a dynamic robot (CjrlDynamicRobot).
This solver implements smooth enforcement of joint limits by using an object of class ChppGikBounder.

It uses the following methods from CjrlDynamicRobot interface:
\li currentConfiguration() (read and write)
\li currentVelocity()
\li upperBound()
\li lowerBound()
\li rootJoint()
\li numberDof()

and the following methods from CjrlJoint interface:
\li rankInConfiguration()
\li currentTransformation()
\li jointsFromRootToThis()
\li updateTransformation()
\li numberDof()

\ingroup solver
 */
class ChppGikSolver
{
public:

    /**
        \brief Constructor
        \param inRobot robot
     */
    ChppGikSolver(CjrlDynamicRobot& inRobot);

    /**
        \brief Set the root joint for computations
     */
    void rootJoint(CjrlJoint& inRootJoint);
    
    /**
        \brief Set the diagonal weights for the  Weighted Pseudoinverse
        \return false if inWeights not of size robot.numberDof()
     */
    bool weights(vectorN& inWeights);

    /**
        \brief prepare for solving. use this to set root joint and compute jacobians and values for given stack
    */
    void prepare(std::vector<CjrlGikStateConstraint*>& inTasks);
    /**
        \brief Compute a solution to the entered vector of linear systems. the linear systems(CjrlGikStateConstraint objects) should already be computed. The order in the vector of tasks follows decreasing priority.
     */
    void solve(std::vector<CjrlGikStateConstraint*>& inTasks, const std::vector<double>& inSRcoefs);
    /**
        \brief Same as previous without damping
     */
    void solve(std::vector<CjrlGikStateConstraint*>& inTasks);
    
    /**
    \name Retrieving the solution
    @{
    */
    /**
    \brief Get the full solution configuration including the root configuration
    \return a vector of size robot()->numberDof()
     */
    const vectorN& solution();
    /**
    \brief Get the new joint configuration excluding the root joint.
    \return a vector of size robot()->numberDof()-6
    */
    const vectorN& solutionJointConfiguration();
    /**
    \brief Get the new pose of the root as a 6d vector: X Y Z ROLL PITCH YAW
    \return a pair of vectors: first vector is [X Y Z], second is [ROLL PITCH YAW]
     */
    const std::pair<vector3d,vector3d>& solutionRootConfiguration();
    /**
    \brief Get the new pose of the root as a homogenous matrix
    \return the new transformation matrix of the root
     */
    const matrix4d& solutionRootPose();
    /**
    @}
    */

    /**
        \brief Destructor
     */
    ~ChppGikSolver();

private:
    void computeRobotSolution();
    unsigned int attNumParams;
    CjrlDynamicRobot* attRobot;
    vectorN attWeights, attSolution,attActive, attComputationWeights, attBackupConfig, attFullSolution, attVals, attRate;
    ChppGikSolverBasic* attSolver;
    ChppGikBounder* attBounder;
    CjrlJoint*  RootJoint;
    matrix4d H0;
    matrix4d Hif;
    matrix4d Hf;
    matrix4d HRC;
    matrix4d InvHf;
    matrix3d TmpR;
    vectorN CurFullConfig;
    std::vector<CjrlJoint*> supportJoints;
    bool attChangeRootJoint,attChangeRootPose;
    std::pair<vector3d,vector3d> attRetPair;
};

#endif
