#ifndef HPP_GIK_SOLVER_ROBOT_ATTACHED_H
#define HPP_GIK_SOLVER_ROBOT_ATTACHED_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "core/hppGikSolverBasic.h"
#include "core/hppGikBounder.h"


/**
\brief this object is a prioritized inverse kinematics solver tailored to a robot that has one fixed joint.
The solver implements smooth enforcement of joint limits by using an object of class ChppGikBounder .
Implementation Note: The 6 degrees of freedom of the root are for now supposed to be stored at the head of the robot configuration vector.
 */
class ChppGikSolverRobotAttached
{
public:

    /**
    \brief Constructor
    \parameter inRobot must be a CjrlDynamicRobot with a fixed joint.
     */
    ChppGikSolverRobotAttached(CjrlDynamicRobot& inRobot);
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
    \brief Call robot->currentConfiguration(updatedConfig) with updatedConfig being the current robot configuration incremented with solution(). This does not call computeForwardKinematics().
     */
    void applySolution();
    /**
    \brief Destructor
     */
    ~ChppGikSolverRobotAttached();

private:
    void computeConstraintDofs(CjrlGikStateConstraint* inConstraint);
    void computeFreeFlyerVelocity();
    unsigned int attNumParams;
    CjrlDynamicRobot* attRobot;
    vectorN attWeights, attSolution, attActive, attComputationWeights, attBackupConfig, attVals, attRate;
    ChppGikSolverBasic* attSolver;
    ChppGikBounder* attBounder;
    CjrlJoint*  FixedJoint;
    matrixNxP H0;
    matrixNxP Hif;
    matrixNxP Hf;
    matrixNxP InvHf;
    vectorN BaseEuler;
    vectorN CurFullConfig;
    vectorN ElementMask;
    
    std::vector<CjrlJoint*> supportJoints;
    std::vector<unsigned int> supportJointsRanks;
};

#endif
