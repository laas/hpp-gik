#ifndef HPP_GIK_SOLVER_H
#define HPP_GIK_SOLVER_H

#include "jrl/mal/matrixabstractlayer.hh"
#include "gikTask/jrlGikStateConstraint.h"
#include "hpp/gik/core/lne-solver.hh"

/**
Compute the joints updates for a hierarchy of simultaneous tasks.


\ingroup solver
 */
class ChppGikSolver
{
    public:

        /**
        \brief Constructor
        \param inRobot robot
         */
        ChppGikSolver ( CjrlDynamicRobot& inRobot );

        /**
        \brief Select the root joint
         */
        void rootJoint ( CjrlJoint& inRootJoint );

        /**
        \brief Set weights on the degrees of freedom
        \return false if inWeights not of size robot.numberDof()
         */
        bool weights ( const vectorN& inWeights );

        /**
        \brief Deprecated. Does nothing.
         */
        void prepare ( std::vector<CjrlGikStateConstraint*>& inTasks );
        /**
        \brief Compute a solution to the entered vector of linear systems. the linear systems(CjrlGikStateConstraint objects) should already be computed. The order in the vector of tasks follows decreasing priority.
         */
        void solve ( std::vector<CjrlGikStateConstraint*>& inTasks, const std::vector<double>& inSRcoefs );
        /**
        \brief Same as previous without damping
         */
        void solve ( std::vector<CjrlGikStateConstraint*>& inTasks );

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
        void updateVelocityBounds();
        bool enforceBounds ( vectorN& inActiveDofs, vectorN& prev, const vectorN& extramove,unsigned int& outSaturated );
        void computeRobotSolution();
        void solveOver(CjrlGikStateConstraint* overTask, double overCoef,const vectorN& inFreedofs);

        CjrlDynamicRobot* attRobot;
        CjrlJoint*  RootJoint;
        std::vector<CjrlJoint*> supportJoints,attWorkingJoints;
        std::pair<vector3d,vector3d> attRetPair;
        ChppGikSolverLNE* attSolver;

        std::vector<unsigned int> stored_indices;
        unsigned int attNumParams,LongSize,LeftSize;
        bool attChangeRootJoint,attChangeRootPose;

        vectorN work_value, attSolution,attBackupConfig, attFullSolution, CurFullConfig, attActiveDofs, attWeights, attExtramove;
        vectorN attUb,attVUb,attLb,attVLb, work_freedofs,work_deltas;
        double attVeloLambda;

        matrix4d H0,Hif,Hf,HRC,InvHf;
        matrix3d TmpR;
};

#endif
