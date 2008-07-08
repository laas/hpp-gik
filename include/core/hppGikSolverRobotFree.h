#ifndef HPP_GIK_SOLVER_ROBOT_FREE_H
#define HPP_GIK_SOLVER_ROBOT_FREE_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "core/hppGikSolverBasic.h"
#include "core/hppGikBounder.h"


/**
\brief this object is a prioritized inverse kinematics solver for a freeflying CjrlDynamicRobot.
The solver implements smooth enforcement of joint limits by using an object of class ChppGikBounder .
 */
class ChppGikSolverRobotFree
{
public:

    /**
        \brief Constructor
     */
    ChppGikSolverRobotFree(CjrlDynamicRobot& inRobot);
    /**
        \brief Set the diagonal weights for the  Weighted Pseudoinverse
        \return false if inWeights of incorrect size
     */
    bool weights(vectorN& inWeights);
    /**
        \brief Compute a solution to the entered vector of linear systems. the linear systems(CjrlGikStateConstraint objects) should already be computed. The order in the vector of tasks follows decreasing priority.
     */
    bool solve(std::vector<CjrlGikStateConstraint*>& inTasks, std::vector<double>& inSRcoefs);
    /**
        \brief Same as previous without damping
     */
    bool solve(std::vector<CjrlGikStateConstraint*>& inTasks);
    /**
        \brief get the solution (i.e. the computed joint velocities (including freeflyer))
        \return vector of size robot.numberDof()
     */
    const vectorN& solution();
    /**
        \brief Call robot->currentConfiguration(updatedConfig) with updatedConfig being the current robot configuration incremented with solution(). This method does not call computeForwardKinematics().
     */
    void applySolution();
    /**
        \brief Destructor
     */
    ~ChppGikSolverRobotFree();

private:
    void computeConstraintDofs(CjrlGikStateConstraint* inConstraint);
    void computeFreeFlyerVelocity();
    unsigned int attNumParams;
    CjrlDynamicRobot* attRobot;
    vectorN attWeights, attSolution, attActive, attComputationWeights, attBackupConfig, attVals, attRate;
    ChppGikSolverBasic* attSolver;
    ChppGikBounder* attBounder;
    vectorN CurFullConfig;
    vectorN ElementMask;
};

#endif
