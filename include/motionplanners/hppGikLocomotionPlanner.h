#ifndef HPP_GIK_LOCOMOTION_PLANNER_H
#define HPP_GIK_LOCOMOTION_PLANNER_H

#include "robot/hppGikFootprintRelated.h"
#include "robot/hppGikStandingRobot.h"
#include "core/hppGikSolver.h"
#include "constraints/hppGikPlannableConstraint.h"
#include "constraints/hppGikPlaneConstraint.h"
#include "robot/hppRobotMotion.h"
#include "constraints/hppGikLPConstraint.h"
#include "constraints/hppGikLP2Constraint.h"
#include "constraints/hppGikComConstraint.h"
#include "constraints/hppGikPlaneConstraint.h"
#include "constraints/hppGikPositionConstraint.h"
#include "constraints/hppGikParallelConstraint.h"
#include "constraints/hppGikRotationConstraint.h"
#include "constraints/hppGikTransformationConstraint.h"
#include "constraints/hppGikPointingConstraint.h"


/**
\brief This plans steps according to given stack of constraints, selected actuated dofs and weights
 */
class ChppGikLocomotionPlanner
{
public:

    /**
    \brief Constructor.
     */
    ChppGikLocomotionPlanner(ChppGikStandingRobot* inStandingRobot);

    /**
    \brief Set the prioritized constraints to plan locomotion for. THE CONSTRAINTS MUST NOT BE DELETED BEFORE CALLING SOLVE METHOD. The entered constraints are not const qualified to permit computation of vectorized targets
     */
    void constraints(const std::vector<ChppGikPlannableConstraint*>& inConstraints);

    /**
    \brief Set the weights used to solve the constraints. 0 indicates that the corresponding dof is not to be used to compute the solution. The higher the value, the more the dof is preferred to others for the solution.
     */
    void weights(vectorN& inWeights);

    
    
    /**
    \brief Set the supportFoot for the next solve method (little or big). This should be called once the constraints have been entered, otherwise it would return false. The algorithm tries both feet (i.e. performs a littleSolve()) and selects the one which gives the biggest waist tranformation change.
    */
    bool automaticSupportFoot();

    /**
    \brief Set the current support foot.
    Can be used together with little solve or with bigSolve. In the latter case, the initial support foot is set according to the given value.
     */
    void supportFoot(const CjrlJoint* inFootJoint);

    /**
    \brief Modify the configuration of the robot in the way that tends to realize the stack of constraints currently stored. The modification keeps the following constraints:
        - The waist's frame center projection on the groud is the ankles' projections middlepoint.
        - The waist's roll and pitch are null
        - The feet are flat on the ground
        - The center of mass can be placed above both feet
    This method depends on values entered through automaticInitialSupportFoot() and initialSupportFoot()
    \parameter allowFootChange : If one foot can no longer contribute to the constraints realization this parameter specifies wether the algorithm should try the other foot before returning false
    \return false if not contribution to the constraints realization is noticed
     */
    bool littleSolve(bool allowFootChange = true);

    /**
    \brief Starting from the current robot configuration and support polygon, find a sequence of support polygons yielding to satisfied constraints.
    This alternates calls to littleSolve() and defaultStopConditionMet().
    The solution is a vector of pointer to ChppGikFoot objects that can be retrieved using the method bigSolution();
    \parameter inMaxIterations sets the maximum number of foot steps to be planned. 0 corresponds to letting the algorithm use its default value.
    */
    bool bigSolve(unsigned int inMaxIterations);
    
    /**
    \brief A default method to checks wether the entered constraints are met. All the joints are used. An additional constraint is added: the center of mass must be in the middle of the support polygon.
    \return true if all constraints solved (computed constraint values found beneath 1e-5), false otherwise
    */
    bool defaultStopConditionMet();

    /**
    \brief Get the big solution computed by batchSolve
     */
    std::vector<ChppGikFoot*> bigSolution();

    /**
    \brief Dump big solution to file
     */
    void dumpBigSolutionTo(const char* inFilename);

    /**
    \brief Destructor
     */
    ~ChppGikLocomotionPlanner();

private:
    /**
    \brief
    */
    bool enforceFeetDomain();

    /**
    \brief 
    */
    void clearFeet();

    /**
    \brief 
     */
    void clearConstraints();

    /**
    \brief
    */
    bool solveGik(std::vector<CjrlGikStateConstraint*>& inConstraints, std::vector<CjrlGikStateConstraint*>& watchedConstraints, bool verbose = false);

   
    /**
    \brief Get the new footprint accordting to new support polygon and moved joint
    */
    const ChppGikFootprint* getFootprint(ChppGikSupportPolygon* inSP, CjrlJoint* movedFoot);

    /**
    \brief Clamp gap between current state and target state for every constraint before solving
    */
    void appendClampedTargets(std::vector<CjrlGikStateConstraint*>& outResult, double inMaxVal);
    
    /**
    \brief Append last computed clamped constraints
    */
    void appendTargets(std::vector<CjrlGikStateConstraint*>& outResult);

    /**
    \brief
    */
    void appendFoot(const ChppGikFootprint& inFp, CjrlJoint* inFoot);

    /**
    \brief Check the support polygon against awkward configurations
     */
    bool supportPolygonValid();

    /**
    \brief Check the support polygon for center of mass reachability
     */
    bool comMovable();
    
    /**
    \brief
    */
    bool solveConstraints();
    
    /**
    \brief
    */
    void saveState();
    
    /**
    \brief
     */
    void switchFeetRoles();
    
    /**
    \brief
     */
    void appendLast();
    
    /**
    \brief
     */
    void recordCurrentValues();
        
    /**
    \brief
     */
    void computeProgress();

    /**
    \brief
     */
    bool anyProgress();
    
    /**
    \brief
     */
    void updateSupportFootRelated();

    /**
    \brief Associated standing robot
     */
    ChppGikStandingRobot* attStandingRobot;

    /**
    \brief Associated robot
     */
    CjrlHumanoidDynamicRobot* attRobot;

    /**
    \brief Big solution feet
     */
    std::vector<ChppGikFoot*> attRawFeet, attSolutionFeet;

    /**
    \brief Pointers to the copies of the entered constaints
     */
    std::vector<ChppGikPlannableConstraint*> attConstraints;

    /**
    \brief Pointers to the original constraints
     */
    std::vector<ChppGikPlannableConstraint*> attUserConstraints;
    
    /**
    \brief
    */
    std::vector<double> attLastValues, attCurrentValues, attDeltaValues;

    /**
    \brief Weights used to solve the constraints
     */
    vectorN attWeights, attWeightedWholeBody;

    /**
    \brief GIK Solver
     */
    ChppGikSolver* attGikSolver;

    /**
    \brief maximum number of iterations for gikSolver to solve a constraint
     */
    unsigned int attGikMaxIter;

    /**
    \brief maximum number of foot steps planned by solve()
     */
    unsigned int attSolveDefaultMaxIter;

    /**
    \brief
    */
    double attNvalthresh;

    /**
    \brief
     */
    ChppRobotMotion* attDetailedMotion, *attRefinedMotion;

    /**
    \brief
     */
    double attMinDist;

    /**
    \brief
     */
    double attMaxDist;

    /**
    \brief
     */
    double attMinRelLeftY;

    /**
    \brief
     */
    double attMaxComDist;

    /**
    \brief
     */
    double attMinValProgress,attMinWaistProgress;


    /**
    \brief
     */
    double attMaxRelTheta, attMaxPosTheta;

    /**
    \brief
     */
    double attMinRelTheta, attMinPosTheta, attGikValueStep, attSPUpdate;


    ChppGikComConstraint* attComConstraint;
    ChppGikPlaneConstraint* attFootPlane,*attWaistPlane;
    ChppGikPositionConstraint* attWaistPosition, *attFootPosition;
    ChppGikParallelConstraint* attFootParallel, *attWaistParallel;
    ChppGikTransformationConstraint* attFootTransformation, *attWaistTransformation;
    ChppGikLPConstraint* attLPConstraint;
    ChppGikLP2Constraint* attLP2Constraint;
    ChppGikRotationConstraint* attFootRot;
    ChppGikPointingConstraint* attWaistPointing;

    vectorN attFinalConfiguration, attPreviousConfiguration, attDeltaWaistState;

    std::vector<CjrlGikStateConstraint*> attJrlWatched, attLocomotionConstraints, attJrlAll;
    CjrlJoint* attSupportFoot, *attNonSupportFoot, *attDummyFoot;
    vector3d attDummyVec;
    
    bool attCheckForPos, attCheckForRot, attCheckForDist;
    
    ChppGikSupportPolygon* attLastSP;

};

#endif
