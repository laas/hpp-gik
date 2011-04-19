#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/robot/foot-print-related.hh"
#include "hpp/gik/tools.hh"

using namespace boost::numeric::ublas;

//** ChppGikFootprint
ChppGikFootprint::ChppGikFootprint ( double inX, double inY, double inTheta )
{
    attX = inX;
    attY = inY;
    attTh = inTheta;
}

void ChppGikFootprint::worldTranslate ( double inDx, double inDy )
{
    attX += inDx;
    attY += inDy;
}

void ChppGikFootprint::localTranslate ( double inDx, double inDy )
{
    attX += cos ( attTh ) *inDx-sin ( attTh ) *inDy;
    attY += sin ( attTh ) *inDx + cos ( attTh ) *inDy;
}

void ChppGikFootprint::makeRelative ( const ChppGikFootprint* inReferenceFootprint,  ChppGikFootprint* outFootprint )
{
    double deltax = outFootprint->x() - inReferenceFootprint->x();
    double deltay = outFootprint->y() - inReferenceFootprint->y();
    outFootprint->x ( cos ( inReferenceFootprint->th() ) *deltax + sin ( inReferenceFootprint->th() ) *deltay );
    outFootprint->y ( cos ( inReferenceFootprint->th() ) *deltay - sin ( inReferenceFootprint->th() ) *deltax );

    double diffTh = outFootprint->th() - inReferenceFootprint->th();
    if (diffTh > M_PI)
        diffTh -= 2*M_PI;
    else
        if (diffTh < -M_PI)
            diffTh += 2*M_PI;
    outFootprint->th ( diffTh );
}

void ChppGikFootprint::makeAbsolute ( const ChppGikFootprint* inReferenceFootprint,  ChppGikFootprint* outFootprint )
{
    double absX = inReferenceFootprint->x() + cos ( inReferenceFootprint->th() ) * outFootprint->x() - sin ( inReferenceFootprint->th() ) * outFootprint->y();

    double absY = inReferenceFootprint->y() + sin ( inReferenceFootprint->th() ) * outFootprint->x() + cos ( inReferenceFootprint->th() ) * outFootprint->y();

    outFootprint->x ( absX );
    outFootprint->y ( absY );
    double diffTh = outFootprint->th() + inReferenceFootprint->th() ;

    if (diffTh > M_PI)
        diffTh -= 2*M_PI;
    else
        if (diffTh < -M_PI)
            diffTh += 2*M_PI;
    outFootprint->th ( diffTh );
}

double ChppGikFootprint::distanceTo ( const ChppGikFootprint* inFootprint ) const
{
    return sqrt ( pow ( inFootprint->x()-attX,2 ) +pow ( inFootprint->y()-attY,2 ) );
}

void ChppGikFootprint::rotate ( double inDtheta )
{
    attTh += inDtheta;
}

void ChppGikFootprint::x ( double inX )
{
    attX = inX;
}
void ChppGikFootprint::y ( double inY )
{
    attY = inY;
}
void ChppGikFootprint::th ( double inTh )
{
    attTh = inTh;
}

double ChppGikFootprint::x() const
{
    return attX;
}
double ChppGikFootprint::y() const
{
    return attY;
}
double ChppGikFootprint::th() const
{
    return attTh;
}

bool ChppGikFootprint::isPointInsideSafeZone(double inX, double inY)const
{
    if (sqrt(pow(inX - attX,2)+pow(inY - attY,2)) > 5e-2)
        return false;
    return true;
}

ChppGikFootprint*  ChppGikFootprint::cookFootprint ( const matrix4d& inTransformation, double inNormalFootHeight )
{
    double threshDist = 4e-3;//4 mm
    matrixNxP ublasM4 ( 4,4 );
    ChppGikTools::Matrix4toUblas ( inTransformation, ublasM4 );

    if ( fabs(ublasM4 ( 2,3 ) - inNormalFootHeight) >  threshDist )
        return 0;

    //assuming the planner ensures the flatness of the feet
    vectorN eulerZYX ( 3 );
    ChppGikTools::RottoEulerZYX ( subrange ( ublasM4,0,3,0,3 ), eulerZYX );

    return new ChppGikFootprint ( ublasM4 ( 0,3 ),ublasM4 ( 1,3 ),eulerZYX ( 2 ) );
}

ChppGikFootprint::~ChppGikFootprint()
{}


ChppGikStepTarget::ChppGikStepTarget(const ChppGikFootprint&  inFootprint, bool isRight)
{
    attIsRight = isRight;
    attFootprint = new ChppGikFootprint(inFootprint);
}
void ChppGikStepTarget::print() const
{
    if (isForRight())
        std::cout << "Right ";
    else
        std::cout << "Left  ";

    std::cout << "step to X = " << attFootprint->x()<< " and Y = " << attFootprint->y()<<"\n";
}

ChppGikStepTarget::ChppGikStepTarget(const ChppGikStepTarget&  sourceObject)
{
    attIsRight = sourceObject.isForRight();
    attFootprint = new ChppGikFootprint ( sourceObject.footprint() );
}

const ChppGikFootprint& ChppGikStepTarget::footprint() const
{
    return *attFootprint;
}

bool ChppGikStepTarget::isForRight() const
{
    return attIsRight;
}

ChppGikStepTarget::~ChppGikStepTarget()
{
    delete attFootprint;
}



//** ChppGikSupportPolygon

ChppGikSupportPolygon::ChppGikSupportPolygon ( const ChppGikFootprint&  inFootprint, bool isRight )
{
    attDiscRadiusSqr = 0.0025;//m
    if ( isRight )
    {
        attRightFootprint = new ChppGikFootprint ( inFootprint );
        attLeftFootprint  = 0;
    }
    else
    {
        attLeftFootprint  = new ChppGikFootprint ( inFootprint );
        attRightFootprint = 0;
    }

    attLfootM = 0;
    attRfootM = 0;
}

ChppGikSupportPolygon::ChppGikSupportPolygon ( const ChppGikFootprint&  inLeftFootprint, const  ChppGikFootprint&  inRightFootprint )
{
    attDiscRadiusSqr = 0.0025;//m
    attRightFootprint = new ChppGikFootprint ( inRightFootprint );
    attLeftFootprint = new ChppGikFootprint ( inLeftFootprint );
    attLfootM = 0;
    attRfootM = 0;
}

ChppGikSupportPolygon::ChppGikSupportPolygon ( const ChppGikSupportPolygon&  sourceObject )
{
    attDiscRadiusSqr = 0.0025;//m
    attRightFootprint = attLeftFootprint = 0;
    attRfootM = attLfootM = 0;

    if ( sourceObject.isLeftLegSupporting() )
        attLeftFootprint = new ChppGikFootprint ( *sourceObject.leftFootprint() );

    if ( sourceObject.isRightLegSupporting() )
        attRightFootprint = new ChppGikFootprint ( *sourceObject.rightFootprint() );

    if ( sourceObject.rfootTransformation() )
    {
        attRfootM = new matrix4d();
        *attRfootM = * ( sourceObject.rfootTransformation() );
    }

    if ( sourceObject.lfootTransformation() )
    {
        attLfootM = new matrix4d();
        *attLfootM = * ( sourceObject.lfootTransformation() );
    }
}

const matrix4d* ChppGikSupportPolygon::rfootTransformation() const
{
    return attRfootM;
}
const matrix4d* ChppGikSupportPolygon::lfootTransformation() const
{
    return attLfootM;
}

void ChppGikSupportPolygon::rfootTransformation ( const  matrix4d& inMatrix )
{
    delete attRfootM;
    attRfootM = new matrix4d();
    *attRfootM = inMatrix;
}

void ChppGikSupportPolygon::lfootTransformation ( const matrix4d& inMatrix )
{
    delete attLfootM;
    attLfootM = new matrix4d();
    *attLfootM = inMatrix;
}

const ChppGikFootprint* ChppGikSupportPolygon::leftFootprint() const
{
    return attLeftFootprint;
}

const ChppGikFootprint* ChppGikSupportPolygon::rightFootprint() const
{
    return attRightFootprint;
}


bool ChppGikSupportPolygon::isLeftLegSupporting() const
{
    return attLeftFootprint != 0;
}

bool ChppGikSupportPolygon::isRightLegSupporting() const
{
    return attRightFootprint != 0;
}

bool ChppGikSupportPolygon::isDoubleSupport() const
{
    return isRightLegSupporting() && isLeftLegSupporting();
}

void ChppGikSupportPolygon::applyStep ( const ChppGikFootprint*  inTargetFootprint, bool isRight )
{
    if ( isRight )
    {
        if ( !attRightFootprint )
            delete attRightFootprint;
        attRightFootprint = new ChppGikFootprint ( *inTargetFootprint );
    }
    else
    {
        if ( !attLeftFootprint )
            delete attLeftFootprint;
        attLeftFootprint = new ChppGikFootprint ( *inTargetFootprint );
    }
}

bool ChppGikSupportPolygon::isPointInsideSafeZone ( double inx, double iny ) const
{
    double dxr,dyr,dxl,dyl,uX,uY,f2f,coord;

    //std::cout << "lf ( " << attLeftFootprint->x() << " , " << attLeftFootprint->y() << " )  rf ( " << attRightFootprint->x() << " , " << attRightFootprint->y() << " )  point ( " << inx << " , " <<iny<<" ) "<<std::endl;

    if ( isRightLegSupporting() )
    {
        dxr = inx - attRightFootprint->x();
        dyr = iny - attRightFootprint->y();
        if ( isLeftLegSupporting() )
        {
            dxl = inx - attLeftFootprint->x();
            dyl = iny - attLeftFootprint->y();

            //right to left vector
            uX = attLeftFootprint->x() - attRightFootprint->x();
            uY = attLeftFootprint->y() - attRightFootprint->y();
            f2f = sqrt ( uX*uX+uY*uY );

            uX = uX/f2f;
            uY = uY/f2f;
            coord = dxr*uX+dyr*uY;

            if ( coord <= 0 && ( ( dxr*dxr+dyr*dyr ) >= attDiscRadiusSqr ) )
                return false;
            if ( coord > f2f && ( ( dxl*dxl+dyl*dyl ) >= attDiscRadiusSqr ) )
                return false;

            double normalX = dxr-coord*uX;
            double normalY = dyr-coord*uY;

            if ( ( normalX*normalX + normalY*normalY ) > attDiscRadiusSqr )
                return false;

            return true;
        }
        else
            return ( ( dxr*dxr+dyr*dyr ) < attDiscRadiusSqr );

    }
    else
    {
        dxl = inx - attLeftFootprint->x();
        dyl = iny - attLeftFootprint->y();
        return ( ( dxl*dxl+dyl*dyl ) < attDiscRadiusSqr );
    }

}

vector3d ChppGikSupportPolygon::nearestCenterPointTo(const vector3d& inPoint)
{
    vector3d result;
    result[2] = 0.0;
    double dxr,dyr,uX,uY,f2f,coord;

    if ( isRightLegSupporting() )
    {
        if ( isLeftLegSupporting() )
        {
            dxr = inPoint[0] - attRightFootprint->x();
            dyr = inPoint[1] - attRightFootprint->y();

            //right to left vector
            uX = attLeftFootprint->x() - attRightFootprint->x();
            uY = attLeftFootprint->y() - attRightFootprint->y();
            f2f = sqrt ( uX*uX+uY*uY );

            uX = uX/f2f;
            uY = uY/f2f;
            coord = dxr*uX+dyr*uY;

            if ( coord <= 0 )
            {
                result[0] = attRightFootprint->x();
                result[1] = attRightFootprint->y();

            }
            else
                if (coord >= f2f)
                {
                    result[0] = attLeftFootprint->x();
                    result[1] = attLeftFootprint->y();
                }
                else
                {
                    result[0] = attRightFootprint->x() + coord*uX;
                    result[1] = attRightFootprint->y() + coord*uY;
                }
        }
        else
        {
            result[0] = attRightFootprint->x();
            result[1] = attRightFootprint->y();
        }
    }
    else
    {
        result[0] = attLeftFootprint->x();
        result[1] = attLeftFootprint->y();
    }
    return result;
}

void ChppGikSupportPolygon::print() const
{
    std::cout << "Support Polygon:---\n";
    if ( attLeftFootprint )
        std::cout << "\tLeft foot center: "<<attLeftFootprint->x() << " , " << attLeftFootprint->y() <<"\n";
    if ( attRightFootprint )
        std::cout << "\tRightfoot center: "<<attRightFootprint->x() << " , " << attRightFootprint->y() <<"\n";
    std::cout << "-------------------\n";
}


ChppGikSupportPolygon* ChppGikSupportPolygon::makeSupportPolygon ( const matrix4d& leftFootH, const matrix4d& rightFootH, double normalAnkleHeight )
{
    ChppGikSupportPolygon* spPl = 0;
    ChppGikFootprint* leftFP =  ChppGikFootprint::cookFootprint ( leftFootH,normalAnkleHeight );

    ChppGikFootprint* rightFP =  ChppGikFootprint::cookFootprint ( rightFootH,normalAnkleHeight );

    if ( leftFP )
        if ( rightFP )
            spPl = new ChppGikSupportPolygon ( *leftFP,*rightFP );
        else
            spPl = new ChppGikSupportPolygon ( *leftFP,false );
    else
        if ( rightFP )
            spPl = new ChppGikSupportPolygon ( *rightFP,true );

    delete leftFP;
    delete rightFP;
    return spPl;
}

vector3d ChppGikSupportPolygon::meanOrientation()
{
    vector3d mean;
    double angle = 0;
    if ( isRightLegSupporting() )
    {
        angle = rightFootprint()->th();
        if ( isLeftLegSupporting() )
        {
            angle += rightFootprint()->th();
            angle /= 2;
        }
    }
    else
        angle = leftFootprint()->th();

    mean[0] = cos ( angle );
    mean[1] = sin ( angle );
    mean[2] = 0;

    return mean;
}

void ChppGikSupportPolygon::center ( double& outX, double& outY )
{
    if ( isRightLegSupporting() )
        if ( isLeftLegSupporting() )
        {
            outX = 0.5* ( attLeftFootprint->x() +attRightFootprint->x() );
            outY = 0.5* ( attLeftFootprint->y() +attRightFootprint->y() );
        }
        else
        {
            outX = attRightFootprint->x();
            outY = attRightFootprint->y();
        }
    else
    {
        outX = attLeftFootprint->x();
        outY = attLeftFootprint->y();
    }
}

ChppGikSupportPolygon::~ChppGikSupportPolygon()
{
    delete attRightFootprint;
    delete attLeftFootprint;
    delete attRfootM;
    delete attLfootM;
}
