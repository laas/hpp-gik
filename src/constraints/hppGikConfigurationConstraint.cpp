#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikConfigurationConstraint.h"
#include "hppGikTools.h"

using namespace boost::numeric::ublas;


ChppGikConfigurationConstraint::ChppGikConfigurationConstraint(CjrlDynamicRobot& inRobot, const vectorN& inTargetConfig, const vectorN& inMask)
{
    if ((inTargetConfig.size() == inRobot.numberDof()) && (inMask.size() == inRobot.numberDof()))
    {
        attTargetConfiguration = inTargetConfig;
        attInfluencingDofs = inMask;
    }
    else
    {
        std::cout << "ChppGikConfigurationConstraint:: incorrect sizes" <<std::endl;
        attTargetConfiguration = inRobot.currentConfiguration();
        attInfluencingDofs = scalar_vector<double>(inRobot.numberDof(), 1);
    }

    attRobot = &inRobot;

    attDimension = 0;
    for (unsigned int i = 0; i< attRobot->numberDof();i++ )
        if (attInfluencingDofs(i) == 1)
            attDimension++;


    attJacobian.resize(attDimension,attRobot->numberDof());
    attJacobian.clear();
    unsigned int line = 0;

    for (unsigned int col = 0; col< attRobot->numberDof();col++ )
    {
        if (attInfluencingDofs(col) == 1)
        {
            attJacobian(line, col) = 1;
            line++;
        }
    }

    attValue.resize(attDimension);
    attVectorizedState.resize(3*attTargetConfiguration.size());
    attTarget.resize(attTargetConfiguration.size());

}

bool ChppGikConfigurationConstraint::target(const vectorN& inTargetConfig)
{
    if (inTargetConfig.size() == attTargetConfiguration.size())
    {
        attTargetConfiguration = inTargetConfig;
        return true;
    }
    return false;
}

ChppGikConfigurationConstraint::~ChppGikConfigurationConstraint()
{}


CjrlGikStateConstraint* ChppGikConfigurationConstraint::clone() const
{
    return new ChppGikConfigurationConstraint(*attRobot,attTargetConfiguration, attInfluencingDofs);
}


CjrlDynamicRobot& ChppGikConfigurationConstraint::robot()
{
    return *attRobot;
}

unsigned int ChppGikConfigurationConstraint::dimension() const
{
    return attDimension;
}

void ChppGikConfigurationConstraint::jacobianRoot(CjrlJoint& inJoint)
{}

void ChppGikConfigurationConstraint::computeInfluencingDofs()
{}


vectorN& ChppGikConfigurationConstraint::influencingDofs()
{
    return attInfluencingDofs;
}


void ChppGikConfigurationConstraint::computeValue()
{
    noalias(attTarget) = attTargetConfiguration - robot().currentConfiguration();
    unsigned int k = 0;
    for (unsigned int l = 0; l< robot().numberDof();l++ )
        if (attInfluencingDofs(l) == 1)
        {
            attValue(k) =  attTarget(l);
            k++;
        }
}


void ChppGikConfigurationConstraint::computeJacobian()
{}


const vectorN& ChppGikConfigurationConstraint::value()
{
    return attValue;
}


const matrixNxP& ChppGikConfigurationConstraint::jacobian()
{
    return attJacobian;
}


void ChppGikConfigurationConstraint::computeVectorizedState()
{
    subrange(attVectorizedState,0,attTargetConfiguration.size()) = attRobot->currentConfiguration();
    subrange(attVectorizedState,attTargetConfiguration.size(),2*attTargetConfiguration.size()) = attRobot->currentVelocity();
    subrange(attVectorizedState,2*attTargetConfiguration.size(),3*attTargetConfiguration.size()) = attRobot->currentAcceleration();
}

void ChppGikConfigurationConstraint::computeVectorizedTarget()
{
    attVectorizedTarget = attTargetConfiguration;
}

bool ChppGikConfigurationConstraint::vectorizedTarget( const vectorN& inTarget )
{
    return target( inTarget );
}

