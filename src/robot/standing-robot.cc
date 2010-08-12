#include "hpp/gik/robot/standing-robot.hh"
#include "hpp/gik/tools.hh"

#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J
#define V3_I  MAL_S3_VECTOR_ACCESS
#define M3_IJ  MAL_S3x3_MATRIX_ACCESS_I_J

using namespace boost::numeric::ublas;

ChppGikStandingRobot::ChppGikStandingRobot ( CjrlHumanoidDynamicRobot& inRobot, ChppGik2DShape& inLeftFootprintShape, ChppGik2DShape& inRightFootprintShape ) :attMaskFactory ( NULL )
{
    attRobot = &inRobot;
    attLeftFootShape = inLeftFootprintShape;
    attRightFootShape = inRightFootprintShape;

    attSupportPolygonConfig.resize ( attRobot->numberDof(),false );
    vector3d anklePos;

    attRobot->rightFoot()->getAnklePositionInLocalFrame ( anklePos );


    attAnklePos = anklePos[2];
    attCurrentSupportPolygon = ChppGikSupportPolygon::makeSupportPolygon
                               ( attRobot->leftAnkle()->currentTransformation(),
                                 attRobot->rightAnkle()->currentTransformation(),
                                 attAnklePos );
    attSupportPolygonConfig = attRobot->currentConfiguration();

    attHalfSittingConfig = attRobot->currentConfiguration();

    attWaist = attRobot->waist()->currentTransformation();
    attRFoot = attRobot->rightAnkle()->currentTransformation();
    attLFoot = attRobot->leftAnkle()->currentTransformation();

    attRelativeCOM = attRobot->positionCenterOfMass();
    for ( unsigned int i=0; i < 3; i++ )
        attRelativeCOM[i] = attRelativeCOM[i] - M4_IJ ( attRFoot,i,3 );
    double yaw = atan2 ( M4_IJ ( attRFoot,1,0 ), M4_IJ ( attRFoot,0,0 ) );

    double relcx = cos ( yaw ) * attRelativeCOM[0] + sin ( yaw ) * attRelativeCOM[1];
    double relcy = cos ( yaw ) * attRelativeCOM[1] - sin ( yaw ) * attRelativeCOM[0];
    attRelativeCOM[0] = relcx;
    attRelativeCOM[1] = relcy;

    attConfiguration.resize ( attRobot->numberDof() );
	attVelocity.resize(attRobot->numberDof());
	attAcceleration.resize(attRobot->numberDof());

    matrix4d tempM4 = attRobot->waist()->currentTransformation();
    matrix4d tempInv;
    MAL_S4x4_INVERSE ( tempM4,tempInv,double );
    MAL_S4x4_C_eq_A_by_B ( attWaistVertical,tempInv,vector3d(0,0,1) );
	
}

const vector3d& ChppGikStandingRobot::halfsittingLocalWaistVertical()
{
    return attWaistVertical;
}

const ChppGik2DShape& ChppGikStandingRobot::supportPolygonShape()
{
    attSPShape.vertices.clear();
    ChppGik2DVertex attVertex;
    supportPolygon();
    if ( ! ( attCurrentSupportPolygon->isDoubleSupport() ) )
    {
        if ( attCurrentSupportPolygon->isLeftLegSupporting() )
        {

            double co = cos ( attCurrentSupportPolygon->leftFootprint()->th() );
            double si = sin ( attCurrentSupportPolygon->leftFootprint()->th() );
            double posX= attCurrentSupportPolygon->leftFootprint()->x();
            double posY= attCurrentSupportPolygon->leftFootprint()->y();
            for ( unsigned int i=0;i<attLeftFootShape.vertices.size();i++ )
            {
                attVertex.x = posX + co * attLeftFootShape.vertices[i].x - si * attLeftFootShape.vertices[i].y ;
                attVertex.y = posY + si * attLeftFootShape.vertices[i].x + co * attLeftFootShape.vertices[i].y ;
                attSPShape.vertices.push_back ( attVertex );
            }
        }
        else
            if ( attCurrentSupportPolygon->isRightLegSupporting() )
            {
                attSPShape.vertices.clear();
                double co = cos ( attCurrentSupportPolygon->rightFootprint()->th() );
                double si = sin ( attCurrentSupportPolygon->rightFootprint()->th() );
                double posX= attCurrentSupportPolygon->rightFootprint()->x();
                double posY= attCurrentSupportPolygon->rightFootprint()->y();
                for ( unsigned int i=0;i<attRightFootShape.vertices.size();i++ )
                {
                    attVertex.x = posX + co * attRightFootShape.vertices[i].x - si * attRightFootShape.vertices[i].y ;
                    attVertex.y = posY + si * attRightFootShape.vertices[i].x + co * attRightFootShape.vertices[i].y ;
                    attSPShape.vertices.push_back ( attVertex );
                }
            }
    }
    else
    {
        attSPShape.vertices.clear();
        std::vector<const ChppGikLinkedVertex*> linkedVertices;
        computeFeet2DConvexHull ( linkedVertices );
        for ( unsigned int i=0;i<linkedVertices.size();i++ )
            attSPShape.vertices.push_back ( linkedVertices[i]->world );

    }
    return attSPShape;
}

void ChppGikStandingRobot::computeFeet2DConvexHull ( std::vector<const ChppGikLinkedVertex*>& outVertices )
{
    supportPolygon();
    if ( ! ( attCurrentSupportPolygon->isDoubleSupport() ) )
        return;

    std::vector<ChppGik2DVertex> points;
    std::vector<ChppGik2DVertex> localpoints;
    std::vector<CjrlJoint*> feet;
    ChppGik2DVertex vl,vw;
    ChppGik2DVertex bary;
    bary.x = 0;
    bary.y = 0;
    unsigned int nv = attLeftFootShape.vertices.size() + attRightFootShape.vertices.size();

    double th = attCurrentSupportPolygon->leftFootprint()->th();
    for ( unsigned int i=0;i<attLeftFootShape.vertices.size();i++ )
    {
        vl = attLeftFootShape.vertices[i];
        vw.x = attCurrentSupportPolygon->leftFootprint()->x() + cos ( th ) *vl.x -sin ( th ) *vl.y;
        vw.y = attCurrentSupportPolygon->leftFootprint()->y() + sin ( th ) *vl.x +cos ( th ) *vl.y;
        points.push_back ( vw );
        localpoints.push_back ( vl );
        feet.push_back ( attRobot->leftAnkle() );
        bary.x += vw.x;
        bary.y += vw.y;
    }

    th = attCurrentSupportPolygon->rightFootprint()->th();
    for ( unsigned int i=0;i<attRightFootShape.vertices.size();i++ )
    {
        vl = attRightFootShape.vertices[i];
        vw.x = attCurrentSupportPolygon->rightFootprint()->x() + cos ( th ) *vl.x -sin ( th ) *vl.y;
        vw.y = attCurrentSupportPolygon->rightFootprint()->y() + sin ( th ) *vl.x +cos ( th ) *vl.y;
        points.push_back ( vw );
        localpoints.push_back ( vl );
        feet.push_back ( attRobot->rightAnkle() );
        bary.x += vw.x;
        bary.y += vw.y;
    }

    bary.x /=nv;
    bary.y /=nv;

    std::vector<ChppGik2DVertex> sortedpoints;
    std::vector<ChppGik2DVertex> sortedlocalpoints;
    std::vector<CjrlJoint*> sortedfeet;
    std::vector<double> sortedangles;
    std::vector<double> sorteddist;

    ChppGik2DVertex axis;
    axis.x= points[0].x - bary.x;
    axis.y= points[0].y - bary.y;
    double normAxis = sqrt ( axis.x*axis.x +axis.y*axis.y );
    axis.x /= normAxis;
    axis.y /= normAxis;

    sortedpoints.push_back ( points[0] );
    sortedlocalpoints.push_back ( localpoints[0] );
    sortedfeet.push_back ( feet[0] );
    sortedangles.push_back ( 0 );
    sorteddist.push_back ( normAxis );

    double norm,dx,dy,cs,sn, ang;
    std::vector<double>::iterator iterangle;
    std::vector<double>::iterator iterdist;
    std::vector<ChppGik2DVertex>::iterator itervert;
    std::vector<ChppGik2DVertex>::iterator iterlocalvert;
    std::vector<CjrlJoint*>::iterator iterfoot;
    bool inserted;
    for ( unsigned int i=1;i<nv;i++ )
    {
        dx= points[i].x - bary.x;
        dy= points[i].y - bary.y;
        norm = sqrt ( dx*dx + dy*dy );
        dx /= norm;
        dy /= norm;
        cs = dx*axis.x + dy*axis.y;
        sn = dy*axis.x - dx*axis.y;
        ang = atan2 ( sn,cs );

        iterangle = sortedangles.begin();
        itervert = sortedpoints.begin();
        iterlocalvert = sortedlocalpoints.begin();
        iterfoot = sortedfeet.begin();
        iterdist = sorteddist.begin();

        inserted = false;
        while ( iterangle != sortedangles.end() )
        {
            if ( *iterangle == ang )
            {
                if ( norm > *iterdist )
                {
                    *iterdist = norm;
                    *itervert = points[i];
                    *iterlocalvert = localpoints[i];
                    *iterfoot = feet[i];
                }
                inserted = true;
                break;
            }
            if ( *iterangle > ang )
            {
                sortedangles.insert ( iterangle, ang );
                sortedpoints.insert ( itervert, points[i] );
                sortedlocalpoints.insert ( iterlocalvert, localpoints[i] );
                sortedfeet.insert ( iterfoot, feet[i] );
                sorteddist.insert ( iterdist, norm );
                inserted = true;
                break;
            }
            iterangle++;
            iterfoot++;
            itervert++;
            iterlocalvert++;
            iterdist++;
        }
        if ( !inserted )
        {
            sortedangles.push_back ( ang );
            sortedpoints.push_back ( points[i] );
            sortedlocalpoints.push_back ( localpoints[i] );
            sortedfeet.push_back ( feet[i] );
            sorteddist.push_back ( norm );
        }
    }

    double dxnext,dxprev,dynext,dyprev;

    attElements.clear();

    ChppGikLinkedVertex lv;
    iterlocalvert = sortedlocalpoints.begin();
    iterfoot = sortedfeet.begin();
    for ( itervert = sortedpoints.begin(); itervert != sortedpoints.end(); itervert++ )
    {
        lv.world = *itervert;//sortedpoints[i];
        lv.local = *iterlocalvert;//sortedlocalpoints[i];
        lv.joint = *iterfoot;//sortedfeet[i];
        attElements.push_back ( lv );
        iterlocalvert++;
        iterfoot++;
    }
    unsigned int lastind = attElements.size()-1;
    for ( unsigned int i = 1; i < lastind; i++ )
    {
        attElements[i].prev = & ( attElements[i-1] );
        attElements[i].next = & ( attElements[i+1] );
    }
    attElements[0].prev = & ( attElements[lastind] );
    attElements[0].next = & ( attElements[1] );
    attElements[lastind].prev = & ( attElements[lastind-1] );
    attElements[lastind].next = & ( attElements[0] );


    ChppGikLinkedVertex* currentVertex = & ( attElements[0] );
    unsigned int nelements = attElements.size(), checked = 0;


    while ( checked!=nelements )
    {
        dxprev= currentVertex->prev->world.x - currentVertex->world.x;
        dyprev= currentVertex->prev->world.y - currentVertex->world.y;
        norm = sqrt ( dxprev*dxprev + dyprev*dyprev );
        dxprev /= norm;
        dyprev /= norm;

        dxnext= currentVertex->next->world.x - currentVertex->world.x;
        dynext= currentVertex->next->world.y - currentVertex->world.y;
        norm = sqrt ( dxnext*dxnext + dynext*dynext );
        dxnext /= norm;
        dynext /= norm;

        currentVertex->ntonext.x = dxnext;
        currentVertex->ntonext.y = dynext;

        currentVertex->nout.x = dynext;
        currentVertex->nout.y = -dxnext;

        currentVertex->b = currentVertex->nout.x * currentVertex->world.x + currentVertex->nout.y * currentVertex->world.y;


        currentVertex->dtonext = norm;

        sn = dxnext*dyprev - dxprev*dynext;

        if ( sn<=0 )
        {
            currentVertex->next->prev = currentVertex->prev;
            currentVertex->prev->next = currentVertex->next;
            currentVertex = currentVertex->prev;
            nelements--;
            checked = 0;
        }
        else
        {
            checked++;
            currentVertex = currentVertex->next;
        }
    }

    outVertices.clear();
    for ( unsigned int i = 0; i<nelements;i++ )
    {
        outVertices.push_back ( currentVertex );
        currentVertex = currentVertex->next;
    }
}

const ChppGik2DShape& ChppGikStandingRobot::leftFootShape() const
{
    return attLeftFootShape;
}

const ChppGik2DShape& ChppGikStandingRobot::rightFootShape() const
{
    return attRightFootShape;
}

bool ChppGikStandingRobot::isPointInsideSupportPolygon ( double inX, double inY,double safetyMargin )
{
    double msfm = ( safetyMargin>0.0 ) ?safetyMargin:0.0;
    std::vector<const ChppGikLinkedVertex*> vertices;
    computeFeet2DConvexHull ( vertices );
    bool oneIsOut = false;
    for ( unsigned int i = 0; i<vertices.size();i++ )
    {
        if ( vertices[i]->nout.x * inX + vertices[i]->nout.y * inY > vertices[i]->b - msfm )
        {
            oneIsOut = true;
            break;
        }
    }
    return !oneIsOut;
}

CjrlHumanoidDynamicRobot* ChppGikStandingRobot::robot() const
{
    return attRobot;
}

vectorN ChppGikStandingRobot::computeConfigurationWrtFreeFlyer ( CjrlJoint* inJoint, matrix4d& inFreeFlyerInWorld )
{
    vectorN config = attRobot->currentConfiguration();
    matrix4d previousRootT = attRobot->rootJoint()->currentTransformation();
    matrix4d jointT = inJoint->currentTransformation();
    matrix4d inverseJointT;
    MAL_S4x4_INVERSE ( jointT,inverseJointT,double );
    matrix4d freeflyer2root, newRoot;
    MAL_S4x4_C_eq_A_by_B ( freeflyer2root,inverseJointT,previousRootT );
    MAL_S4x4_C_eq_A_by_B ( newRoot,inFreeFlyerInWorld,freeflyer2root );

    matrixNxP rot ( 3,3 );
    vectorN vec ( 3 ), euler ( 3 );
    ChppGikTools::HtoRT ( newRoot, rot, vec );
    ChppGikTools::RottoEulerZYX ( rot, euler );
    subrange ( config,0,3 ) = vec;
    subrange ( config,3,6 ) = euler;

    return config;
}

ChppGikMaskFactory* ChppGikStandingRobot::maskFactory()
{
    if ( !attMaskFactory )
        attMaskFactory = new ChppGikMaskFactory ( attRobot );
    return attMaskFactory;
}

ChppGikSupportPolygon* ChppGikStandingRobot::supportPolygon()
{

    bool idem = true;
    for ( unsigned int i=0;i<attRobot->numberDof();i++ )
        if ( attSupportPolygonConfig ( i ) != attRobot->currentConfiguration() ( i ) )
        {
            idem = false;
            break;
        }

    if ( !idem )
    {
        delete attCurrentSupportPolygon;


        attCurrentSupportPolygon = ChppGikSupportPolygon::makeSupportPolygon
                                   ( attRobot->leftAnkle()->currentTransformation(),
                                     attRobot->rightAnkle()->currentTransformation(),
                                     attAnklePos );

        attSupportPolygonConfig = attRobot->currentConfiguration();
    }

    return attCurrentSupportPolygon;
}

bool ChppGikStandingRobot::leftFootAhead()
{
    bool ret;

    if ( !supportPolygon()->isDoubleSupport() )
    {
        std::cout << "ChppGikStandingRobot::leftFootAhead() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    ChppGikFootprint *testInitialFootprint;

    testInitialFootprint = new ChppGikFootprint ( *attCurrentSupportPolygon->leftFootprint() );
    ChppGikFootprint::makeRelative ( attCurrentSupportPolygon->rightFootprint(),  testInitialFootprint );
    if ( testInitialFootprint->x() > 1e-2 )
        ret = true;
    else
        ret = false;

    delete testInitialFootprint;
    return ret;
}


bool ChppGikStandingRobot::rightFootAhead()
{
    bool ret;

    if ( !supportPolygon()->isDoubleSupport() )
    {
        std::cout << "ChppGikStandingRobot::rightFootAhead() : failed to identify a double support polygon on the current robot configuration\n";
        return false;
    }

    ChppGikFootprint *testInitialFootprint;

    testInitialFootprint = new ChppGikFootprint ( *attCurrentSupportPolygon->rightFootprint() );
    ChppGikFootprint::makeRelative ( attCurrentSupportPolygon->leftFootprint(),  testInitialFootprint );
    if ( testInitialFootprint->x() > 1e-2 )
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
    double dx = M4_IJ ( attRFoot,0,3 )-M4_IJ ( attLFoot,0,3 );
    double dy = M4_IJ ( attRFoot,1,3 )-M4_IJ ( attLFoot,1,3 );
    return sqrt ( dx*dx + dy*dy );
}


double ChppGikStandingRobot::halfsittingWaistHeight()
{
    return M4_IJ ( attWaist,2,3 );
}

void ChppGikStandingRobot::staticHalfsitting()
{
    staticState ( attHalfSittingConfig );
}

ChppGikStandingRobot::~ChppGikStandingRobot()
{
    delete attCurrentSupportPolygon;
    delete attMaskFactory;
    delete attRobot;
}


bool ChppGikStandingRobot::staticState ( const vectorN& inConfig )
{
    if ( inConfig.size() != attRobot->numberDof() )
        return false;

    attRobot->currentConfiguration ( inConfig );
    attRobot->currentVelocity ( zero_vector<double> ( attRobot->numberDof() ) );
    attRobot->currentAcceleration ( zero_vector<double> ( attRobot->numberDof() ) );
    attRobot->computeForwardKinematics();
    attRobot->computeForwardKinematics();

    return true;
}

void ChppGikStandingRobot::updateRobot ( const matrix4d& inRootPose, const vectorN& inJoints, double inTimeStep )
{
    unsigned int i;

    //compute root Pose with euler angle
    ChppGikTools::splitM4 ( inRootPose, FD_Ro, FD_tmp );
    ChppGikTools::M3toEulerZYX ( FD_Ro, FD_tmp2 );

    //Compute root's V and Omega
    ChppGikTools::splitM4 ( attRobot->rootJoint()->currentTransformation(),FD_Rt,FD_tmp3 );
    FD_Roo = FD_Ro - FD_Rt;
    FD_Rt = MAL_S3x3_RET_TRANSPOSE ( FD_Ro );
    FD_Ro = FD_Roo * FD_Rt;
    FD_w[0]  = M3_IJ ( FD_Ro,2,1 );
    FD_w[1]  = M3_IJ ( FD_Ro,0,2 );
    FD_w[2]  = M3_IJ ( FD_Ro,1,0 );

    //Build Configuration vector
    for ( i=0;i<3;i++ )
    {
        attConfiguration ( i ) = FD_tmp[i];
        attConfiguration ( i+3 ) = FD_tmp2[i];
    }
    subrange ( attConfiguration,6,attRobot->numberDof() ) = inJoints;


    //Build Velocity vector
    noalias(attVelocity) = attConfiguration;
    attVelocity.minus_assign ( attRobot->currentConfiguration() );
    for ( i=0;i<3;i++ )
        attVelocity ( i+3 ) = FD_w[i];

    attVelocity /= inTimeStep;


    //Build Acceleration vector
    noalias(attAcceleration) = attVelocity;
    attAcceleration.minus_assign ( attRobot->currentVelocity() );
    attAcceleration /= inTimeStep;

    //Put state vectors in jrl robot
    attRobot->currentConfiguration ( attConfiguration );
    attRobot->currentVelocity ( attVelocity );
    attRobot->currentAcceleration ( attAcceleration );

    attRobot->computeForwardKinematics();
}
