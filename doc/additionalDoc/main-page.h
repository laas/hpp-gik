/** \mainpage GIK START POINT

\author Oussama Kanoun
\author 
\section sec_intro It's time to GIK some linear system's butt.

So you would like to:
 \li \ref linSyst_section
 \li \ref multLinSyst_section
 \li \ref plan1_section
 \li \ref plan2_section
 \li \ref plan3_section
 \li \ref end_section


\section linSyst_section Solve a linear system

Let \f$ A \in \Re^{m \times n} \f$ and \f$ b \in \Re^{m} \f$. Solving linear system \f$ \lbrace A x = b \rbrace\f$ in \f$x\f$ consists in finding a solution vector \f$ x \in \Re^{n}\f$ such as \f$ A x = b\f$\n

When the linear system is underconstrained, i.e \f$ m<n \f$, an infinity of solutions exist. The general form of the solution can be written as:
\f[ x = x^{*} + N y\f]
where:
\li \f$x^{*}\f$ is a solution to the linear system
\li \f$N \in \Re^{n \times n}\f$ is the projector on the linear subspace \f$ {vect(A^{T})}^{\perp}\f$ called Nullspace.
\li \f$y \in \Re^{n}\f$

This defines an affine subspace of solutions.\n
The point \f$x^{*}\f$ can be the minimal norm solution of the system for which we have an analytical expression in case matrix A is of plain rank:
\f[ x^{*} = A^{T} ( A A^{T} )^{-1} b\f]
This particular solution is usually called the pseudoinverse or minimal norm solution.
There exists a weighted pseudoinverse solution, corresponding to minimization of \f$ x^{T} W x\f$ where \f$W\f$ is a positive semi definite matrix. The particular solution is:
\f[ x^{*} = W^{-1} A^{T} ( A W^{-1} A^{T} )^{-1} b\f]

\f$W\f$ can be used for a variety of effects, like priveledging certain parameters, disactivating others, etc.

\subsection singular_subsec Dealing with singular systems

When A is not of plain rank or is nearly singular, the matrice product \f$A A^{T} \f$ is singular of nearly singular. A solution that minimizes the cost \f$\|A x - b\|\f$ can be computed by inverting \f$A A^{T} \f$ while canceling contribution of low singular values:
\f[ x^{*} = A^{T} ( tsvd(A A^{T}) )^{-1} b\f] 
where \f$tsvd()\f$ is the function that enables a singular value \f$\sigma\f$ for inversion if larger that a given threshold and cancels it otherwise.\n

It is also possible to "damp" the pseudo inverse solution by choosing to compute:
\f[ x^{*} = A^{T} ( A A^{T} + \lambda I )^{-1} b\f]
where:
 \li \f$ \lambda \f$ is the damping factor
 \li \f$ I \f$ is the identity matrix of size n.

This is equivalent to expanding the initial linear system with the following:
\f[ \lambda I = 0\f]
which tends to attract the solution vector \f$ x^{*} \f$ to the null vector, depending on the value of \f$\lambda\f$.


\subsection solverBasic_section Implementation

The linear system solver in hppGik library is called ChppGikSolverBasic.
The constructor ChppGikSolverBasic::ChppGikSolverBasic() requires the space dimension \f$ n \f$.\n
To construct a ChppGikSolverBasic object:

\code
ChppGikSolverBasic solver(n);
\endcode

A linear system is provided as an object of class CjrlGikStateConstraint. The method CjrlGikStateConstraint::jacobian() should return matrix \f$A \in \Re^{m \times n} \f$ and the method CjrlGikStateConstraint::value() should return vector \f$b \in \Re^{m}\f$.
Let \c constraint be an object of type CjrlGikStateConstraint. To compute the pseudoinverse solution of the linear system implied by \c constraint do:

\code
solver.solveTask(constraint);
solution = solver.solution();
\endcode


For the weighted pseudoinverse solution, use:
\code
solver.weights(weights);
solver.solveTask(constraint);
solution = solver.solution();
\endcode

Note that only diagonal weight matrix W is implemented. The argument of method ChppGikSolverBasic::weights() is a vector of size \f$n\f$ used to fill the diagonal of \f$W\f$.\n

 
The default method for dealing with singularity is the SVD thresholding described above. The threshold can be modified through access methods ChppGikSolverBasic::SVDThreshold().\n
\n
For damping method, use:

\code
solver.solveTask(constraint, lambda);
solution = solver.solution();
\endcode
where parameter lambda stands for damping factor \f$\lambda\f$.\n

Any of the previous code can be used to solve a single linear system. To use the same solver object to solve another independant linear system, call ChppGikSolverBasic::resetSolution() or ChppGikSolverBasic::weights() prior to calling ChppGikSolverBasic::solveTask().\n\n

ChppGikSolverBasic is also capable of solving a series of linear systems given in decreasing priority order. Next section will present an overview of the theory behind the algorithm, followed by implementation usage info.
\n\n

\section multLinSyst_section Solve several linear systems in priority order

\subsection algo_subsection Algorithm

Let \f$ (E1): (A_{1},b_{1})\f$ and \f$ (E2):  (A_{2},b_{2})\f$ denote two linear systems.
As seen in the previous section, the general form of \f$E1\f$ is:
\f[ x = {x_{1}}^{*} + N_1 y\f]
We have the analytical expression for \f$N_1\f$:
\f[ N_1 = (I - A^{T}(A A^{T})^{-1}A)\f]
Solving \f$E2\f$ in second priority means to compute a solution to \f$E2\f$ that is within the solutions of \f$E1\f$. Therefore \f$E2\f$ is modified to take into account \f$E1\f$ priority and becomes:
\f[ N_1 A_2 x = b_2- A_2 {x_{1}}^{*}\f]
This system is solved in the same fashion as \f$E1\f$ to obtain a solution point \f${x_{2}}^{*}\f$. The new general form of solution to prioritized systems \f$[(E1), (E2)]\f$ is :
\f[ x = {x_{1}}^{*} + {x_{2}}^{*} + N_1 N_2 z\f]


Because \f$E2\f$ might become singular when projected on \f$E1\f$, the methods used to deal with singular cases presented in previous section can still be used here. Then the point \f${x_{2}}^{*}\f$ would not strictly be solution to \f$(E1)\f$, but a least-square- error solution.

\subsection impl2_subsection Implementation

ChppGikSolverBasic implements prioritized linear system resolution.
The systems must be presented one at a time, the solution vector is incremented after each system resolution and the nullspace is updated.\n
To construct a ChppGikSolverBasic object:
\code
ChppGikSolverBasic solver(n);
\endcode

Let \c constraint1 , \c constraint2 and \c constraint3 be 3 CjrlGikStateConstraint objects.
To solve the stack of prioritized systems do:

\code
solver.resetSolution();
solver.solveTask(constraint1, 0.0, false, true);
solver.solveTask(constraint2, 0.0, true, true);
solver.solveTask(constraint3, 0.0, true, false);
solution = solver.solution()
\endcode

Second argument ChppGikSolverBasic::solveTask() is the damping factor which can be any positive double value (like in previous section). Argument 3 tells the method whether to project the linear system's matrix \f$A\f$ with the previous nullspace projector or not. Since \c constraint1 is the first constraint in the stack, there is no need to do so, hence value \c false. Argument 4 tells the method wether to compute the null space projector \f$N_1\f$ or not. It is set to \c true here since it will be used to compute \c constraint2 's solution. The exact opposite thing happens for last linear system \c constraint3.\n
In sum, argument 3 an 4 are options to optimize first and last system computation.\n
ChppGikSolverBasic::setActiveParameters can be used before each ChppGikSolverBasic::solveTask() call to specify the columns of linear system matrix \f$A\f$ to be used. This can give some computation speedup.\n
ChppGikSolverBasic::weights() can replace ChppGikSolverBasic::resetSolution() in case the weights for the weighted pseudoinverse should be changed. The user can set zero weights for some parameters (for instance when they are not used), leading to faster computation.

\n\n
 \section plan1_section Plan motion for your robot
Give a look to \ref linSyst_section first.
So you have a robot, formed of several articulated bodies, for which you want to compute joint motion in order to realize a certain task. Most of the time, the task is naturally expressed in the cartesian space, like position and/or orientation of a body in the robot. The non-linear realationship between body transformations and joint angles is most of the time not invertible, especially when the number of degrees of freedom outnumbers the task dimension. This is where numerical resolution schemes come handy. Here is an outline:

\code
WHILE task not achieved DO:
   (1) Compute value (vector) of task
   (2) Compute gradient (jacobian) of task
   (3) Follow task gradient descent to compute a slight change in values of joints
   (4) IF no change in joint values QUIT
   (5) Apply new joint value to robot.
\endcode

Let \f$q\f$ be the vector of joint values and let \f$P(q)\f$ be the position of point \f$M\f$ attached to a body in the robot and expressed in a frame \f$R\f$ independant of robot motion (for simplicity's sake). Suppose we want point \f$M\f$ to reach a point whose position is \f$T(q)\f$ in frame \f$R\f$. The above code becomes:

\code
WHILE P(q) - T(q) != 0 DO:
   (1) Value             V = eval(T(q) - P(q))
   (2) Jacobian          J = d (P - T) / d q 
   (3) Slight change     dq = r * pseudoinverse(J) * V
   (4) IF dq = 0 QUIT
   (5) q = q + dq
\endcode

where \c r is a positive real used to scale the value.\n
The joint update computed at step (3) is the solution to the linear system:
\f[ {{\partial (P - T)} \over {\partial {q}}}(q)  \delta q = r V \f]

If \c r is controlled properly through iterations, the above algorithm yields a smooth motion. The achievement of position \f$T\f$ is not guaranteed because the linear system might become singular after a few iterations, even from start. In that case the algorithm is said to be trapped in a local minimum and it cannot achieve any better.\n


\subsection impl3_subsection Implementation


Given the tasks, the user should implement CjrlGikStateConstraint objects that compute the jacobians and values correponding to the respective tasks.
To solve the linear systems implied by step (3) ChppGikSolverBasic can be used.\n
However, a problem arises for a robot: joint limits. Joint limits are inequality constraints on the joint values that must constantly be verified. ChppGikSolver is a prioritized linear system solver based on ChppGikSolverBasic, it uses a ChppGikBounder object to enforce joint limits. See these objects' documentation for more details.\n

First construct a solver and pile up CjrlGikStateConstraint objects in a priority-decreasing vector:

\code
ChppGikSolver solver(robot);
std::vector<CjrlGikStateConstraint*> stack;
stack.push_back(highest_priority_constraint);
stack.push_back(mid_priority_constraint);
stack.push_back(low_priority_constraint);
\endcode

This code corresponds to a single iteration of the above algorithm (but with several prioritized tasks):
\code
//Steps (1) and (2)
for (unsigned int i = 0; i< stack.size();i++)
{
    stack[i]->computeValue();
    stack[i]->computeJacobian();
}
//Step (3)
solver.solve( stack );
//Step (4)
if (norm_2(solver.solution()) < 1e-5)
    return;
//Step (5)
//Compute new configuration/velocity/acceleration vectors of robot and apply them
\endcode

There is a variation of ChppGikSolver::solve() to add damping factors. See documentation for details.\n

See \ref example1_page for a source code using ChppGikSolver.

\n\n
\section plan2_section Plan motion for your humanoid robot
This section is merely about drawing the user's attention to humanoid's stability issue.\n As a matter of fact, to reach something with the right hand, and supposing the humanoid is standing upon a flat ground, it needs to move while keeping its Zero Momentum Point inside its support polygon. There is NOTHING implemented in this library that provides such a guarantee. However, if the planned motion is smooth enough (does not necessarily mean slow), then keeping the center of mass's projection inside the support polygon should be enough to achieve a stable reaching motion.\n\n
See \ref example1_page for a code where center of mass and feet constraints are added to maintain stability while reaching.

\n\n
 \section plan3_section Obtain humanoid motion without reading all previous sections
Well, this is no joke, you will be able to compute all kinds of motion just reading this section. At the expense of flexibility, of course. The API shown in previous sections would be "low-level" compared to the one shown here.\n
Attention is focused on ChppGikGenericTask class. Objects of this class define a time scale \f$[0,+\inf[\f$ on which the user schedules finite-time motions.\n
A motion is either a ChppGikPrioritizedMotion or a ChppGikLocomotionElement.\n
ChppGikPrioritizedMotion motions are defined by a desired constraint of class ChppGikVectorizableConstraint, a time frame (i.e a start time and end time), a priority integer (the higher the integer the lower the priority) and a "mask vector" that defines active and inactive joints for the accomplishment of the motion.\n
ChppGikLocomotionElement are special motions that affect feet and center of mass. ChppGikStepElement is an example of ChppGikLocomotionElement that an be used to plan stepping motion for the humanoid robot.\n
ChppGikGenericTask object takes care of stability of the robot. For example, to move the right hand from its current position to a given position, the user need not specify feet and center of mass tasks.\n
The output of ChppGikGenericTask is a stable ChppRobotMotion.\n
Further details can be found in ChppGikGenericTask documentation. Let us just give usage outline :\n\n

Prepare a robot and a generic task:
\code
//Supposing object "robot" is of class CjrlHumanoidDynamicRobot, and is in double support configuration:
ChppGikStandingRobot standingRobot(robot);
double samplingPeriod = 5e-3;
ChppGikGenericTask genericTask(standingRobot, samplingPeriod);
\endcode

Schedule prioritized motion and locomotion objects in generic task:
\code
genericTask.addElement(motion1);
genericTask.addElement(step1);
\endcode

Solve and retrieve solution if successful:
\code
bool ok = genericTask.solve();
if (ok)
    motion = genericTask.solutionMotion();
\endcode

\n
Have a look at \ref tasks for ready-to-use motion planners built upon ChppGikGenericTask.

 \n\n
 \section end_section Do something else
Perfectly understandable.
*/
















/**
\defgroup constraints State Constraints
\brief Various objects implementing abstract class CjrlGikStateConstraint
*/

/**
\defgroup solver Prioritized State Constraints Solvers
\brief For solving prioritized linear systems given by CjrlGikStateConstraint objects
 */

/**
\defgroup motions Prioritized Motions
\brief Various objects implementing abstract class ChppGikPrioritizedMotion.
 */

/**
\defgroup motionsplan Prioritized Motions Plan
\brief ChppGikPrioritizedMotion organizer.
The Motion Plan contains references to ChppGikPrioritizedMotion objects, organized by descending priority in motion plan rows. References to ChppGikPrioritizedMotion are stacked according to their priority.<br>
A ChppGikMotionPlan is equivalent to a matrix of ChppGikMotionPlanElement objects. <br>
A ChppGikMotionPlanElement is a container for CjrlGikStateConstraint objects that share the same priority.<br>
A ChppGikMotionPlanRow is a container for ChppGikPrioritizedMotion that share the same priority, so the horizontal direction is for time progression. <br>
A ChppGikMotionPlanColumn is a container for all the ChppGikMotionPlanElement objects that can be collected at a given time. The vertical direction stacks the constaints at a given time.
<br>
<br>
This module was originally intended for internal use.
 */

/**
\defgroup motionsolver Prioritized Motions Solver
\brief Compute a whole body joint motion ( ChppRobotMotion object ) according to entered 
ChppGikPrioritizedMotion objects and/or ChppGikLocomotionElement objects.
 */

/**
\defgroup tasks Dedicated Tasks
\brief  Dedicated Tasks are whole body joint motion planners for specific cases.
For example, a ChppGikReachTask will produce reaching motion for a humanoid robot, possibly involving a step, based on a target's cartesian position and a few more option specifications.
Dedicated tasks derive from Abstract class ChppGikRobotTask.
*/

/**
\defgroup robot Robot
\brief Additional information describing a Humnoid biped robot state.
 */

/**
\defgroup tools Tools
\brief Various mathematical function used in this packages are regrouped here.
 */
