#ifndef HPP_GIK_BOUNDER_H
#define HPP_GIK_BOUNDER_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"


/**
\brief Object used in ChppGikSolver to smoothly enforce the joint limits.
Let I be an interval bounded by a lower value LV and an upper value UV. Let X be a variable whose value is continuously changing in I. Call D the change rate in X. 
X is free to move at any rate D if not too close to LV or UV. if X is close enough to UV (resp LV) and going toward UV (resp LV), we want to compute a coefficient tending from 1 (instant of entering ineraction region) to 0 (when reaching UV) to be used in clamping the change rate. This object computes this coefficient for several parameters acording to their current values and change rates and the upper and lower bounds. 
\ingroup solver
 */
class ChppGikBounder
{
public:

    /**
    \brief Constructor
    \param numberParam is the size of the variables vector
     */
    ChppGikBounder(unsigned int numberParam);
    /**
    \brief Set the upper bound on a control variable. 
    \return false if rank is out of bounds or given upper bound is smaller than recorded lower bound
     */
    bool upperBound(unsigned int i_rank, double i_bound);
    /**
    \brief Set the lower bound on a control variable
    \return false if rank is out of bounds or given lower bound is bigger than recorded lower bound
     */
    bool lowerBound(unsigned int i_rank, double i_bound);
    /**
    \brief Get the upper bound on a control variable. 
    \return false if rank is out of bounds
    */
    bool getUpperBound(unsigned int i_rank, double &i_bound);
    /**
    \brief Get the lower bound on a control variable
    \return false if rank is out of bounds
     */
    bool getLowerBound(unsigned int i_rank, double &i_bound);
    /**
    \brief set the braking zone size near upper and lower bound in percentage of the total interval. Will apply for all intervals. Entered value clamped between 0.01 and 0.5. Default to 0.12;
    */
    void brakingZone(double inPercent);
    /**
    \brief Compute coefficients according to parameters values and parameters change rates.
    \param values current values
    \param changerate current values change rate
    \param outCoefficients computed coefficients
    \return false when incorrect vector size is found
     */
    bool computeCoefficients(const vectorN& values, const vectorN& changerate, vectorN& outCoefficients);
    /**
    \brief Does the same as ChppGikBounder::computeCoefficients() but applies the coefficients directly to a weights vector
    \param values current values
    \param changerate current values change rate
    \param ioWeights weights modified and returned weights for pseudoinverse use
    \return false when incorrect vector size is found
     */
    bool modifyWeights(const vectorN& values, const vectorN& changerate, vectorN& ioWeights);
    /**
    \brief Destructor
     */
    ~ChppGikBounder();

private:
    
    void prepareBrakeWindow();
    double brakeCoefForJoint(const double& qVal,const double& lowerLimit, const double& upperLimit, const double& dq);
    unsigned int attNumberParam;
    vectorN attUpperBound, attLowerBound, attCoefs,JointUpperLimitWindow;
    double      BrakingZone;
    double      WindowStep;
};

#endif
