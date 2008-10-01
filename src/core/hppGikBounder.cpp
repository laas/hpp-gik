#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "core/hppGikBounder.h"
#include "hppGikTools.h"

using namespace ublas;

ChppGikBounder::ChppGikBounder(unsigned int numberParam)
{
    attNumberParam = numberParam;
    attUpperBound = scalar_vector<double>(numberParam, 1e10);
    attLowerBound = scalar_vector<double>(numberParam, -1e10);
    attCoefs.resize(numberParam);
    BrakingZone = 0.12;
    WindowStep = 1e-3;
    prepareBrakeWindow();
}

bool ChppGikBounder::upperBound(unsigned int i_rank, double i_bound)
{
    if (i_rank >= attNumberParam)
    {
        std::cout << "ChppGikSolverGeneric::upperBound() rank out of bounds\n";
        return false;
    }

    if (attLowerBound(i_rank) > i_bound)
    {
        std::cout << "ChppGikSolverGeneric::upperBound() discarding upper bound smaller than lower bound at rank "<< i_rank<<"\n";
        return false;
    }

    attUpperBound(i_rank) = i_bound;
    return true;
}

bool ChppGikBounder::lowerBound(unsigned int i_rank, double i_bound)
{
    if (i_rank >= attNumberParam)
    {
        std::cout << "ChppGikSolverGeneric::lowerBound() rank out of bounds\n";
        return false;
    }

    if (attUpperBound(i_rank) < i_bound)
    {
        std::cout << "ChppGikSolverGeneric::upperBound() discarding lower bound bigger than upper bound at rank "<< i_rank<<"\n";
        return false;
    }
    attLowerBound(i_rank) = i_bound;
    return true;
}

bool ChppGikBounder::getUpperBound(unsigned int i_rank, double &i_bound)
{
    if (i_rank >= attNumberParam)
    {
        std::cout << "ChppGikSolverGeneric::lowerBound() rank out of bounds\n";
        return false;
    }
    i_bound = attUpperBound(i_rank);
    return true;
}

bool ChppGikBounder::getLowerBound(unsigned int i_rank, double &i_bound)
{
    if (i_rank >= attNumberParam)
    {
        std::cout << "ChppGikSolverGeneric::lowerBound() rank out of bounds\n";
        return false;
    }
    i_bound = attLowerBound(i_rank);
    return true;
}

void ChppGikBounder::brakingZone(double inPercent)
{
    if (inPercent > 0.5)
        BrakingZone = 0.5;
    else
        if (inPercent < 0.01)
            BrakingZone = 0.01;
        else
            BrakingZone = inPercent;

    prepareBrakeWindow();
}

void ChppGikBounder::prepareBrakeWindow()
{
    unsigned int extra = 10;
    unsigned int sizeWindow = ChppGikTools::timetoRank ( 0.0,1.0,WindowStep ) +1 ;

    JointUpperLimitWindow.resize ( sizeWindow,false );

    ChppGikTools::minJerkCurve ( 1.0-WindowStep*extra,WindowStep,1.0,0.0,0.0,0.0,JointUpperLimitWindow );

    subrange ( JointUpperLimitWindow, sizeWindow - extra, sizeWindow ) = zero_vector<double> ( extra );
}


double ChppGikBounder::brakeCoefForJoint ( const double& qVal,const double& lowerLimit, const double& upperLimit, const double& dq )
{

    if ( upperLimit ==lowerLimit )
        return 0;

    double qFraction = ( qVal-lowerLimit ) / ( upperLimit-lowerLimit );


    if ( ( dq>0 ) && ( qFraction > 1 - BrakingZone ) )
    {
        double curve_Fraction = ( qFraction + BrakingZone - 1 ) /BrakingZone;
        unsigned int windowTick = ChppGikTools::timetoRank ( 0.0, curve_Fraction, WindowStep );
        if ( windowTick >= JointUpperLimitWindow.size() )
            return 0;
        return JointUpperLimitWindow ( windowTick );
    }

    if ( ( dq<0 ) && ( qFraction < BrakingZone ) )
    {
        double curve_Fraction = qFraction/BrakingZone;
        unsigned int windowTick = ChppGikTools::timetoRank ( 0.0, curve_Fraction, WindowStep );
        if (windowTick >= JointUpperLimitWindow.size())
            return 0;

        return JointUpperLimitWindow ( JointUpperLimitWindow.size()-1-windowTick );
    }

    return 1;
}


bool ChppGikBounder::computeCoefficients(const vectorN& values, const vectorN& rate, vectorN& outCoefficients)
{
    if (values.size()!=attNumberParam )
    {
        std::cout << "ChppGikBounder::computeCoefficients() argument 1 incorrect size\n";
        return false;
    }

    if ( rate.size()!=attNumberParam)
    {
        std::cout << "ChppGikBounder::computeCoefficients() argument 2 incorrect size\n";
        return false;
    }

    if ( outCoefficients.size()!=attNumberParam)
        outCoefficients.resize(attNumberParam);

    for ( unsigned int i=0; i<attNumberParam;i++ )
        outCoefficients(i) =  brakeCoefForJoint(values(i),attLowerBound(i),attUpperBound(i),rate(i));

    return true;
}


bool ChppGikBounder::modifyWeights(const vectorN& values, const vectorN& changerate, vectorN& ioWeights)
{
    if ( ioWeights.size()!=attNumberParam)
    {
        std::cout << "ChppGikBounder::computeCoefficients() argument 3 incorrect size\n";
        return false;
    }
    if (!computeCoefficients(values,changerate,attCoefs))
    {
        std::cout << "ChppGikBounder::modifyWeights() failed\n";
        return false;
    }
    for ( unsigned int i=0; i<attNumberParam;i++ )
        ioWeights(i) *= attCoefs(i);
    return true;
}

ChppGikBounder::~ChppGikBounder()
{}
