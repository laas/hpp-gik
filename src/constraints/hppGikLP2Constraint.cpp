
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikLP2Constraint.h"
#include "hppGikTools.h"

#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J

using namespace ublas;

ChppGikLP2Constraint::ChppGikLP2Constraint(CjrlHumanoidDynamicRobot& inRobot)
{
    attRobot = &inRobot;
    attNumberActuatedDofs = attRobot->numberDof() - 6;
    attInfluencingDofs.resize(attRobot->numberDof(),false);
    computeInfluencingDofs();

    attDimension = 1;

    attMovingFoot = attRobot->rightFoot();

    attFixedFoot = attRobot->leftFoot();

    attValue.resize(attDimension, false);

    attJacobian.resize(attDimension,attNumberActuatedDofs,false);
    attTempJacobian.resize(2,attNumberActuatedDofs,false);
}

CjrlGikStateConstraint* ChppGikLP2Constraint::clone() const
{
    return new ChppGikLP2Constraint(*this);
}

CjrlDynamicRobot& ChppGikLP2Constraint::robot()
{
    return *attRobot;
}

unsigned int ChppGikLP2Constraint::dimension() const
{
    return attDimension;
}

void ChppGikLP2Constraint::movingFoot(CjrlJoint* inJoint)
{
    if (inJoint == attRobot->rightFoot())
    {
        attMovingFoot = inJoint;
        attFixedFoot = attRobot->leftFoot();
    }
    else
        if (inJoint == attRobot->leftFoot())
        {
            attMovingFoot = inJoint;
            attFixedFoot = attRobot->rightFoot();
        }

}

void ChppGikLP2Constraint::computeInfluencingDofs()
{
    unsigned int i;
    attInfluencingDofs.clear();
    std::vector<CjrlJoint *> joints;

    //Free flyer dofs
    CjrlJoint *root = attRobot->rootJoint();
    unsigned int start = root->rankInConfiguration();
    for (i=0; i<root->numberDof(); i++)
        attInfluencingDofs[start+i] = 1;

    //Dofs from free flyer to effector joint
    joints = attRobot->rightFoot()->jointsFromRootToThis();
    for(i=1; i< joints.size(); i++)
        attInfluencingDofs(joints[i]->rankInConfiguration()) = 1;
    joints = attRobot->leftFoot()->jointsFromRootToThis();
    for(i=1; i< joints.size(); i++)
        attInfluencingDofs(joints[i]->rankInConfiguration()) = 1;
}

vectorN& ChppGikLP2Constraint::influencingDofs()
{
    return attInfluencingDofs;
}

void ChppGikLP2Constraint::computeValue()
{
    /*
    const matrix4d& mw = attRobot->waist()->currentTransformation();
    const matrix4d& mf = attMovingFoot->currentTransformation();
    const matrix4d& nmf = attFixedFoot->currentTransformation();
    
    double nwx = M4_IJ(mw,0,0);
    double nwy = M4_IJ(mw,1,0);

    double nmx = M4_IJ(mf,0,0);
    double nmy = M4_IJ(mf,1,0);
    
    double nfxx = M4_IJ(nmf,0,0);
    double nfxy = M4_IJ(nmf,1,0);
    
    attValue(0) = nmx*nwy - nmy*nwx;
    */
    
    ///*
    const matrix4d& mw = attRobot->waist()->currentTransformation();
    const matrix4d& mf = attMovingFoot->currentTransformation();
    const matrix4d& nmf = attFixedFoot->currentTransformation();
    
    double nwx = M4_IJ(mw,0,0);
    double nwy = M4_IJ(mw,1,0);

    double nmx = M4_IJ(mf,0,0);
    double nmy = M4_IJ(mf,1,0);
    
    double nfx = M4_IJ(nmf,0,0);
    double nfy = M4_IJ(nmf,1,0);
    
    double nmidx = nfx+nmx;
    double nmidy = nfy+nmy;
    double normmid = sqrt(nmidx*nmidx + nmidy*nmidy);
    nmidx/=normmid;
    nmidy/=normmid;
    
    attValue(0) = nmidx*nwy - nmidy*nwx;
    //*/
}

void ChppGikLP2Constraint::computeJacobian()
{

    attJacobian.clear();

    attMovingFoot->computeJacobianJointWrtConfig();

    const matrixNxP& fixedFootJacobian = (attFixedFoot->jacobianJointWrtConfig());
    const matrixNxP& movingFootJacobian =  attMovingFoot->jacobianJointWrtConfig();
    const matrix4d& mw = attRobot->waist()->currentTransformation();
    const matrix4d& mf = attMovingFoot->currentTransformation();
    const matrix4d& nmf = attFixedFoot->currentTransformation();
    /*
    double nwxx = M4_IJ(mw,0,0);
    double nwxy = M4_IJ(mw,1,0);

    double nmxx = M4_IJ(mf,0,0);
    double nmxy = M4_IJ(mf,1,0);
    
    double nwyx = M4_IJ(mw,0,1);
    double nwyy = M4_IJ(mw,1,1);

    double nmyx = M4_IJ(mf,0,1);
    double nmyy = M4_IJ(mf,1,1);
    
    double sin1 = nwyx*nmxy-nwyy*nmxx;
    double sin2 = nwxx*nmyy-nwxy*nmyx;
    */
    ///*
    double nwxx = M4_IJ(mw,0,0);
    double nwxy = M4_IJ(mw,1,0);

    double nmxx = M4_IJ(mf,0,0);
    double nmxy = M4_IJ(mf,1,0);
    
    double nfxx = M4_IJ(nmf,0,0);
    double nfxy = M4_IJ(nmf,1,0);
    
    double nmidxx = nfxx+nmxx;
    double nmidxy = nfxy+nmxy;
    double normmid = sqrt(nmidxx*nmidxx + nmidxy*nmidxy);
    nmidxx/=normmid;
    nmidxy/=normmid;
    
    double nwyx = M4_IJ(mw,0,1);
    double nwyy = M4_IJ(mw,1,1);

    double nmyx = M4_IJ(mf,0,1);
    double nmyy = M4_IJ(mf,1,1);
    
    double sin1 = nwyx*nmidxy-nwyy*nmidxx;
    double sin2 = 0.5*(nwxx*nmyy-nwxy*nmyx);
    //*/
    
    noalias(attJacobian) -= subrange(fixedFootJacobian,5,6,6,attRobot->numberDof());
    attJacobian *= sin1 + sin2;
    noalias(attJacobian) += sin2*subrange(movingFootJacobian,5,6,6,attRobot->numberDof());
}

const vectorN& ChppGikLP2Constraint::value()
{
    return attValue;
}

const matrixNxP& ChppGikLP2Constraint::jacobian()
{
    return attJacobian;
}

ChppGikLP2Constraint::~ChppGikLP2Constraint()
{}
