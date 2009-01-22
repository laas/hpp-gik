#ifndef HPP_GIK_SOLVER_H
#define HPP_GIK_SOLVER_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "core/hppGikSolverBasic.h"
#include "core/hppGikBounder.h"

/**
\brief this object is a prioritized inverse kinematics for a dynamic robot (CjrlDynamicRobot).
The solver implements smooth enforcement of joint limits by using an object of class ChppGikBounder.
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
    void solve(std::vector<CjrlGikStateConstraint*>& inTasks, std::vector<double>& inSRcoefs);
    /**
        \brief Same as previous without damping
     */
    void solve(std::vector<CjrlGikStateConstraint*>& inTasks);
    /**
        \brief get the velocity vector, solution of the problem. The freeflyer rotational velocity part is given in angle rate around world frame axises.
        \return vector of size robot.numberDof()
     */
    const vectorN& solution();
    /**
        \brief Converts freeflyer velocity in solution() to eurler angles velocities and add then to the configuration of the robot. This does not call computeForwardKinematics().
     */
    void applySolution();
    /**
        \brief Destructor
     */
    ~ChppGikSolver();

private:
    void convertFreeFlyerVelocity();
    unsigned int attNumParams;
    CjrlDynamicRobot* attRobot;
    vectorN attWeights, attSolution,attEulerSolution, attActive, attComputationWeights, attBackupConfig, attVals, attRate;
    ChppGikSolverBasic* attSolver;
    ChppGikBounder* attBounder;
    CjrlJoint*  RootJoint;
    matrixNxP H0;
    matrixNxP Hif;
    matrixNxP Hf;
    matrixNxP InvHf;
    vectorN BaseEuler;
    vectorN CurFullConfig;
    vectorN ElementMask;
    
    std::vector<CjrlJoint*> supportJoints;
};

#endif
