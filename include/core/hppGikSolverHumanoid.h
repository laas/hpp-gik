#ifndef HPP_GIK_SOLVER_HUMANOID_H
#define HPP_GIK_SOLVER_HUMANOID_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "core/hppGikSolverBasic.h"
#include "core/hppGikBounder.h"


/**
\brief this object is a ChppGikSolverBasic object tailored to a humanoid robot.
These are the additional features of this solver compared to ChppGikSolverBasic:
- (smooth) Enforcement of joint limits
- Computation of humaoind robot's freeflyer's solution velocity, based on a fixed joint in the robot

The provided humanoid robot has to have one fixed joint.
Implementation Note: The 6 degrees of freedom of the root are for now supposed to be stored at the head of the robot configuration vector.
 */
class ChppGikSolverHumanoid
{
public:

    /**
    \brief Constructor
     */
    ChppGikSolverHumanoid(CjrlHumanoidDynamicRobot& inRobot);
    /**
    \brief Set the diagonal weights for the  Weighted Pseudoinverse
    \return false if inWeights of incorrect size
     */
    bool weights(vectorN& inWeights);
    /**
    \brief Compute a solution to the entered vector of linear systems. the linear systems(CjrlGikStateConstraint objects) should already be computed and sorted from highest to lowest priority.
    */
    bool solve(std::vector<CjrlGikStateConstraint*>& inSortedConstraints, std::vector<double>& inSRcoefs);
    /**
    \brief Same as previous without damping
     */
    bool solve(std::vector<CjrlGikStateConstraint*>& inSortedConstraints);
    /**
    \brief get the solution (i.e. the computed joint velocities (including freeflyer))
    \return vector of size robot.numberDof()
     */
    const vectorN& solution();
    /**
    \brief Call robot->currentConfiguration(updatedConfig) with updatedConfig being the current robot configuration incremented with solution(). This will also clear the solution for safety. \note This does not call computeForwardKinematics() !
     */
    void applySolution();
    /**
    \brief Destructor
     */
    ~ChppGikSolverHumanoid();

private:
    void computeConstraintDofs(CjrlGikStateConstraint* inConstraint);
    void computeFreeFlyerVelocity();
    unsigned int attNumParams;
    CjrlHumanoidDynamicRobot* attRobot;
    vectorN attWeights, attSolution, attActive, attComputationWeights, attBackupConfig;
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
