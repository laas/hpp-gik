#include "robot/hppGikStandingRobot.h"
#include "hppGikTools.h"

#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J
#define V3_I  MAL_S3_VECTOR_ACCESS

ChppGikStandingRobot::ChppGikStandingRobot(CjrlHumanoidDynamicRobot* inRobot):attMaskFactory(NULL)
{
    attRobot = inRobot;

    attSupportPolygonConfig.resize(attRobot->numberDof(),false);

    attCurrentSupportPolygon = ChppGikSupportPolygon::makeSupportPolygon(attRobot->leftFoot()->currentTransformation(),attRobot->rightFoot()->currentTransformation(),attRobot->footHeight());

    attSupportPolygonConfig = attRobot->currentConfiguration();

    attHalfSittingConfig = attRobot->currentConfiguration();

    attWaist = attRobot->waist()->currentTransformation();
    attRFoot = attRobot->rightFoot()->currentTransformation();
    attLFoot = attRobot->leftFoot()->currentTransformation();
    attRWrist = attRobot->rightWrist()->currentTransformation();
    attLWrist = attRobot->leftWrist()->currentTransformation();
    attHead = attRobot->gazeJoint()->currentTransformation();

    attRelativeCOM = attRobot->positionCenterOfMass();
    for (unsigned int i=0; i < 3; i++)
        V3_I(attRelativeCOM,i) = V3_I(attRelativeCOM,i) - M4_IJ(attRFoot,i,3);
}

void ChppGikStandingRobot::updateDynamics(double inSamplingPeriod, const vector3d& inZMPworPla, vector3d& outZMPworObs, vector3d& outZMPwstObs, vector3d& outZMPwstPla)
{
    //Update kinematics (finite difference method)
    attRobot->FiniteDifferenceStateUpdate(inSamplingPeriod);

    //Observed ZMP
    outZMPworObs = attRobot->zeroMomentumPoint();

    tempM4 =attRobot->waist()->currentTransformation();
    MAL_S4x4_INVERSE(tempM4,tempInv,double);
    MAL_S4x4_C_eq_A_by_B(outZMPwstObs,tempInv,outZMPworObs);
    MAL_S4x4_C_eq_A_by_B(outZMPwstPla,tempInv,inZMPworPla);
}

CjrlHumanoidDynamicRobot* ChppGikStandingRobot::robot()const
{
    return attRobot;
}

vectorN ChppGikStandingRobot::computeConfigurationWrtFreeFlyer(CjrlJoint* inJoint, matrix4d& inFreeFlyerInWorld)
{
    vectorN config = attRobot->currentConfiguration();
    matrix4d previousRootT = attRobot->rootJoint()->currentTransformation();
    matrix4d jointT = inJoint->currentTransformation();
    matrix4d inverseJointT;
    MAL_S4x4_INVERSE(jointT,inverseJointT,double);
    matrix4d freeflyer2root, newRoot;
    MAL_S4x4_C_eq_A_by_B(freeflyer2root,inverseJointT,previousRootT);
    MAL_S4x4_C_eq_A_by_B(newRoot,inFreeFlyerInWorld,freeflyer2root);

    matrixNxP rot(3,3);
    vectorN vec(3), euler(3);
    ChppGikTools::HtoRT( newRoot, rot, vec);
    ChppGikTools::RottoEulerZYX( rot, euler);
    ublas::subrange(config,0,3) = vec;
    ublas::subrange(config,3,6) = euler;
    
    return config;
}

ChppGikMaskFactory* ChppGikStandingRobot::maskFactory()
{
    if (!attMaskFactory)
        attMaskFactory = new ChppGikMaskFactory(attRobot);
    return attMaskFactory;
}

ChppGikSupportPolygon* ChppGikStandingRobot::supportPolygon()
{

    bool idem = true;
    for (unsigned int i=0;i<attRobot->numberDof();i++)
        if (attSupportPolygonConfig(i)!= attRobot->currentConfiguration()(i))
        {
            idem = false;
            break;
        }

    if (!idem)
    {


        delete attCurrentSupportPolygon;

        attCurrentSupportPolygon = ChppGikSupportPolygon::makeSupportPolygon(attRobot->leftFoot()->currentTransformation(),attRobot->rightFoot()->currentTransformation(),attRobot->footHeight());

        attSupportPolygonConfig = attRobot->currentConfiguration();
    }

    return attCurrentSupportPolygon;
}

bool ChppGikStandingRobot::leftFootAhead()
{
    bool ret;

    if (!supportPolygon()->isDoubleSupport())
    {
        std::cout << "ChppGikStandingRobot::leftFootAhead() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    ChppGikFootprint *testInitialFootprint;

    testInitialFootprint = new ChppGikFootprint(*attCurrentSupportPolygon->leftFootprint());
    ChppGikFootprint::makeRelative(attCurrentSupportPolygon->rightFootprint(),  testInitialFootprint);
    if (testInitialFootprint->x() > 1e-3)
        ret = true;
    else
        ret = false;

    delete testInitialFootprint;
    return ret;
}


bool ChppGikStandingRobot::rightFootAhead()
{
    bool ret;

    if (!supportPolygon()->isDoubleSupport())
    {
        std::cout << "ChppGikStandingRobot::rightFootAhead() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    ChppGikFootprint *testInitialFootprint;

    testInitialFootprint = new ChppGikFootprint(*attCurrentSupportPolygon->rightFootprint());
    ChppGikFootprint::makeRelative(attCurrentSupportPolygon->leftFootprint(),  testInitialFootprint);
    if (testInitialFootprint->x() > 1e-3)
        ret = true;
    else
        ret = false;

    delete testInitialFootprint;
    return ret;
}


vector3d& ChppGikStandingRobot::halfsittingRelativeCOM()
{
    return attRelativeCOM;
}


vectorN& ChppGikStandingRobot::halfsittingConfiguration()
{
    return attHalfSittingConfig;
}

double ChppGikStandingRobot::halfsittingFeetDistance()
{
    return fabs(M4_IJ(attRFoot,1,3) - M4_IJ(attLFoot,1,3));
}


double ChppGikStandingRobot::halfsittingWaistHeight()
{
    return M4_IJ(attWaist,2,3);
}


matrix4d& ChppGikStandingRobot::halfsittingWaistTransformation()
{
    return attWaist;
}


matrix4d& ChppGikStandingRobot::halfsittingLeftWristTransformation()
{
    return attLWrist;
}


matrix4d& ChppGikStandingRobot::halfsittingRightWristTransformation()
{
    return attRWrist;
}


matrix4d& ChppGikStandingRobot::halfsittingRightFootTransformation()
{
    return attRFoot;
}


matrix4d& ChppGikStandingRobot::halfsittingLeftFootTransformation()
{
    return attLFoot;
}


matrix4d& ChppGikStandingRobot::halfsittingHeadTransformation()
{
    return attHead;
}

void ChppGikStandingRobot::staticHalfsitting()
{
    attRobot->staticState( attHalfSittingConfig );
}

ChppGikStandingRobot::~ChppGikStandingRobot()
{
    delete attCurrentSupportPolygon;
    delete attMaskFactory;
    delete attRobot;
}
