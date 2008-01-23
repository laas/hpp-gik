
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "constraints/hppGikLPConstraint.h"
#include "hppGikTools.h"

#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J

using namespace ublas;

ChppGikLPConstraint::ChppGikLPConstraint(CjrlHumanoidDynamicRobot& inRobot)
{
    attRobot = &inRobot;
    attNumberActuatedDofs = attRobot->numberDof() - 6;
    attInfluencingDofs.resize(attRobot->numberDof(),false);
    computeInfluencingDofs();

    attDimension = 2;

    attMovingFoot = attRobot->rightFoot();

    attFixedFoot = attRobot->leftFoot();

    attValue.resize(attDimension, false);

    attJacobian.resize(attDimension,attNumberActuatedDofs,false);
    attTempJacobian.resize(2,attNumberActuatedDofs,false);
}

CjrlGikStateConstraint* ChppGikLPConstraint::clone() const
{
    return new ChppGikLPConstraint(*this);
}

CjrlDynamicRobot& ChppGikLPConstraint::robot()
{
    return *attRobot;
}

unsigned int ChppGikLPConstraint::dimension() const
{
    return attDimension;
}

void ChppGikLPConstraint::movingFoot(CjrlJoint* inJoint)
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

void ChppGikLPConstraint::computeInfluencingDofs()
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

vectorN& ChppGikLPConstraint::influencingDofs()
{
    return attInfluencingDofs;
}

void ChppGikLPConstraint::computeValue()
{
   // /*
    const matrix4d& mw = attRobot->waist()->currentTransformation();

    const matrix4d& mf = attMovingFoot->currentTransformation();

    const matrix4d& nmf = attFixedFoot->currentTransformation();

    attValue(0) = 0.5*(M4_IJ(nmf,0,3) + M4_IJ(mf,0,3)) - M4_IJ(mw,0,3) ;
    attValue(1) = 0.5*(M4_IJ(nmf,1,3) + M4_IJ(mf,1,3)) - M4_IJ(mw,1,3) ;
    /*
    double kx = - (M4_IJ(mf,1,3) - M4_IJ(nmf,1,3));
    double ky = M4_IJ(mf,0,3) - M4_IJ(nmf,0,3);
    
    double l = sqrt(kx*kx+ky*ky);
   
    kx /= l;
    ky /= l;
    
    double nx = M4_IJ(mw,0,0);
    double ny = M4_IJ(mw,1,0);
    
    //k times n
    attValue(2) = kx*ny - ky*nx;
    */
   // */
    /*
    const matrix4d& mw = attRobot->waist()->currentTransformation();
    const matrix4d& mf = attMovingFoot->currentTransformation();
    const matrix4d& nmf = attFixedFoot->currentTransformation();
    double kx = - (M4_IJ(mf,1,3) - M4_IJ(nmf,1,3));
    double ky = M4_IJ(mf,0,3) - M4_IJ(nmf,0,3);
    double l = sqrt(kx*kx+ky*ky);
    kx /= l;
    ky /= l;
    double nx = M4_IJ(mw,0,0);
    double ny = M4_IJ(mw,1,0);
    attValue(0) = kx*ny - ky*nx;
    */
}

void ChppGikLPConstraint::computeJacobian()
{
    ///*
    attJacobian.clear();
    
    attMovingFoot->computeJacobianJointWrtConfig();

    const matrixNxP& fixedFootJacobian = (attFixedFoot->jacobianJointWrtConfig());
    const matrixNxP& movingFootJacobian =  attMovingFoot->jacobianJointWrtConfig();
    const matrix4d& mw = attRobot->waist()->currentTransformation();
    const matrix4d& mf = attMovingFoot->currentTransformation();
    const matrix4d& nmf = attFixedFoot->currentTransformation();

    
    subrange(attJacobian,0,2,0,attNumberActuatedDofs).minus_assign(subrange(fixedFootJacobian,0,2,6,attRobot->numberDof()));

    subrange(attJacobian,0,2,0,attNumberActuatedDofs).minus_assign(subrange(movingFootJacobian,0,2,6,attRobot->numberDof()));

    subrange(attJacobian,0,2,0,attNumberActuatedDofs) *= 0.5;

    vectorN asymVec(3);
    matrixNxP asymMat(3,3);
    for(unsigned int i = 0; i< 3; i++)
        asymVec(i) = -0.5*(M4_IJ(nmf,i,3) + M4_IJ(mf,i,3)) + M4_IJ(mw,i,3);
    
    ChppGikTools::equivAsymMat( asymVec, asymMat );
    asymMat.resize(2,3,true);

    noalias(subrange(attJacobian,0,2,0,attNumberActuatedDofs)) += prod(asymMat,subrange(fixedFootJacobian,3,6,6,attRobot->numberDof()));
    /*
    double ux = M4_IJ(mf,0,3) - M4_IJ(nmf,0,3);
    double uy = M4_IJ(mf,1,3) - M4_IJ(nmf,1,3);
    
    double l = sqrt(ux*ux+uy*uy);
    
    ux /= l;
    uy /= l;
    
    double kx = -uy;
    double ky = ux;

    double nx = M4_IJ(mw,0,0);
    double ny = M4_IJ(mw,1,0);
    
    double sin1 = -ny*kx+nx*ky;
    double sin2 = nx*kx+ny*ky;
    
    noalias(subrange(attJacobian,2,3,0,attNumberActuatedDofs)) = sin1*subrange(fixedFootJacobian,5,6,6,attRobot->numberDof());
    
    noalias(attTempJacobian) = subrange(movingFootJacobian,0,2,6,attRobot->numberDof()) - subrange(fixedFootJacobian,0,2,6,attRobot->numberDof());
    
    asymVec(0) = M4_IJ(mf,0,3) - M4_IJ(nmf,0,3);
    asymVec(1) = M4_IJ(mf,1,3) - M4_IJ(nmf,1,3);
    asymVec(2) = 0.0;
    asymMat.resize(3,3,false);
    ChppGikTools::equivAsymMat( asymVec, asymMat );
    asymMat.resize(2,3,true);
    noalias(attTempJacobian) += prod(asymMat, subrange(fixedFootJacobian,3,6,6,attRobot->numberDof()));
    
    row(attJacobian,2) += (sin2/l)*(kx*row(attTempJacobian,0) + ky*row(attTempJacobian,1));
    */
    //*/
/*
    attJacobian.clear();
    
    attMovingFoot->computeJacobianJointWrtConfig();

    const matrixNxP& fixedFootJacobian = (attFixedFoot->jacobianJointWrtConfig());
    const matrixNxP& movingFootJacobian =  attMovingFoot->jacobianJointWrtConfig();
    const matrix4d& mw = attRobot->waist()->currentTransformation();
    const matrix4d& mf = attMovingFoot->currentTransformation();
    const matrix4d& nmf = attFixedFoot->currentTransformation();

    double ux = M4_IJ(mf,0,3) - M4_IJ(nmf,0,3);
    double uy = M4_IJ(mf,1,3) - M4_IJ(nmf,1,3);
    
    double l = sqrt(ux*ux+uy*uy);
    
    ux /= l;
    uy /= l;
    
    double kx = -uy;
    double ky = ux;

    double nx = M4_IJ(mw,0,0);
    double ny = M4_IJ(mw,1,0);
    
    double sin1 = -ny*kx+nx*ky;
    double sin2 = nx*kx+ny*ky;
    
    noalias(subrange(attJacobian,0,1,0,attNumberActuatedDofs)) = sin1*subrange(fixedFootJacobian,5,6,6,attRobot->numberDof());
    
    noalias(attTempJacobian) = subrange(movingFootJacobian,0,2,6,attRobot->numberDof()) - subrange(fixedFootJacobian,0,2,6,attRobot->numberDof());
    
    vectorN asymVec(3);
    matrixNxP asymMat(3,3);
    asymVec(0) = M4_IJ(mf,0,3) - M4_IJ(nmf,0,3);
    asymVec(1) = M4_IJ(mf,1,3) - M4_IJ(nmf,1,3);
    asymVec(2) = 0.0;
    ChppGikTools::equivAsymMat( asymVec, asymMat );
    asymMat.resize(2,3,true);
    noalias(attTempJacobian) += prod(asymMat, subrange(fixedFootJacobian,3,6,6,attRobot->numberDof()));
    
    row(attJacobian,0) += (sin2/l)*(kx*row(attTempJacobian,0) + ky*row(attTempJacobian,1));
    */
}

const vectorN& ChppGikLPConstraint::value()
{
    return attValue;
}

const matrixNxP& ChppGikLPConstraint::jacobian()
{
    return attJacobian;
}

ChppGikLPConstraint::~ChppGikLPConstraint()
{}
