#ifndef HPP_FOOTPRINT_RELATED_H
#define HPP_FOOTPRINT_RELATED_H

#include <vector>
#include <iostream>
#include <jrl/mal/matrixabstractlayer.hh>
#include "abstract-robot-dynamics/traits/default-pointer.hh"
#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"


class ChppGikFootprint;
/**
\brief This object describes a footprint on a flat floor, by its (x,y) position and its rotation in world frame. A local frame is associated with a footprint: The local frame is set as follows: axis OX points ahead (to the toes), axis OY points to the left and axis OZ points upwards.<br>
The footprint has a circular shape of radius (hardcoded 5cm, to be replaced after description of a footprint shape in CjrlHumanoidDynamicRobot). 

\ingroup robot
 */
class ChppGikFootprint
{
public:
    /**
    \brief Constructor. inX inY and inTheta are absolute coordinates
     */
    ChppGikFootprint(double inX, double inY, double inTheta);

    /**
    \brief Translate with inDx and inDy being given in world coordinates
     */
    void worldTranslate(double inDx, double inDy);

    /**
    \brief Translate with inDx and inDy being given in local coordinates
     */
    void localTranslate(double inDx, double inDy);

    /**
    \brief rotate footprint by inDtheta
     */
    void rotate(double inDtheta);

    /**
    \brief Express outSubjectFootprint (supposed initially expressed in the same frame as inReferenceFootprint) in the local frame of inReferenceFootprint.

     */
    static void makeRelative(const ChppGikFootprint* inReferenceFootprint,  ChppGikFootprint* outSubjectFootprint);

    /**
    \brief Reverse method of makeRelative (express outSubjectFootprint (supposed initially relative to inReferenceFootprint) in the frame in which inReferenceFootprint is expressed)
     */
    static void makeAbsolute(const ChppGikFootprint* inReferenceFootprint,  ChppGikFootprint* outSubjectFootprint);

    /**
    \brief Distance to the given footprint
     */
    double distanceTo(const ChppGikFootprint* inFootprint) const;

    /**
    \brief Determine wether the given point is inside the footprint
    */
    bool isPointInsideSafeZone(double inX, double inY)const;
    
    /**
    \brief make a footprint from a transformation. Return 0 if the foot transformation did not turn flat on the ground
     */
    static ChppGikFootprint*  cookFootprint(const matrix4d& inTransformation, double inNormalFootHeight);

    /**
    \name Get and Set methods for attX attY and attTheta (absolute coordinates)
        {@
     */
    void x(double inX);
    void y(double inY);
    void th(double inTh);

    double x() const;
    double y() const;
    double th() const;

    /**
    @}
     */

    ~ChppGikFootprint();

private:
    double attX;
    double attY;
    double attTh;
};

/**
\brief This is a step target with information on the moving foot
*/
class ChppGikStepTarget
{
public:
    /**
    \brief constructor
    */
    ChppGikStepTarget(const ChppGikFootprint&  inFootprint, bool isRight);

    /**
    \brief copy constructor
     */
    ChppGikStepTarget(const ChppGikStepTarget&  sourceObject);

    /**
    \brief Get the associated footprint
    */
    const ChppGikFootprint& footprint() const;
    
    /**
    \brief Print the x,y coordinates and the identity of foot (left/right)
     */
    void print() const;

    /**
    \brief Is this a step target for the right foot
    */
    bool isForRight() const;
    
    /**
    \brief destructor
    */
    ~ChppGikStepTarget();

private:
    bool attIsRight;
    ChppGikFootprint* attFootprint;
};

/**
\brief This is a support polygon class. A support polygon is defined by at least one foot print.
\ingroup robot
 */
class ChppGikSupportPolygon
{
public:
    /**
    \brief Constructor for one-foot support polygon
     */
    ChppGikSupportPolygon(const ChppGikFootprint&  inFootprint, bool isRight);
    /**
    \brief Constructor for two-foot support polygon
     */
    ChppGikSupportPolygon(const ChppGikFootprint&  inLeftFootprint, const ChppGikFootprint&  inRightFootprint);
    /**
    \brief Deep copy constructor
     */
    ChppGikSupportPolygon(const ChppGikSupportPolygon&  sourceObject);

    /**
    \brief Apply step
     */
    void applyStep(const ChppGikFootprint*  inTargetFootprint, bool isRight);
    /**
    \brief Get left footprint.
    \return 0 if left foot is not on the ground
     */
    const ChppGikFootprint* leftFootprint() const;
    /**
    \brief Get right footprint.
    \return 0 if right foot is not on the ground
     */
    const ChppGikFootprint* rightFootprint() const;
    /**
    \return false is left foot is not on the ground
     */
    bool isLeftLegSupporting() const;
    /**
    \return false is right foot is not on the ground
     */
    bool isRightLegSupporting() const;
    /**
    \return false if single support
     */
    bool isDoubleSupport() const;

    /**
    \brief print the current support polygon
    */
    void print() const;

    /**
    \brief try to construct the support polygon from the given transformation.
    \return 0 if no foot is found on ground
     */
    static ChppGikSupportPolygon* makeSupportPolygon(const matrix4d& leftFootH, const matrix4d& rightFootH, double normalAnkleHeight);

    /**
    \brief Get the center of the support polygon: single support: ankle projection, double support: middle point of ankle projections
    */
    void center(double& outX, double& outY);

    /**
    \brief Get the "mean orientation vector" defined by (xf1 (+ xf2) )/2 where xfi is the x axis of foot i
     */
    vector3d meanOrientation();

    /**
    \brief Tell if the given 2D point is inside this support polygon's safe zone.  if the support polygon is simple, check wether the given point is in the disc centered on the orthogonal projection of the foot ankle on the ground. The radius is arbitrarily set to 4 cm. If the support is double, see if the point is in the area covered by the continuous translation of the same disc from one ankle projection to the other.
     */
    bool isPointInsideSafeZone(double x, double y) const;

    /**
    Compute a point on the line segment linking the projections of the ankles on the ground such as M is the nearest to the parameter point inPoint.
    */
    vector3d nearestCenterPointTo(const vector3d& inPoint);
            
    /**
    \brief Get a pointer to a stored transformation matrix for the right foot. Temporary method (until new version is ready):
    \return 0 if nothing stored
    */
    const matrix4d* rfootTransformation() const;

    /**
    \brief Get a pointer to a stored transformation matrix for the left foot. Temporary method (until new version is ready):
    \return 0 if nothing stored
     */
    const matrix4d* lfootTransformation() const;

    /**
    \brief store a transformation matrix for the right foot. Temporary method (until new version is ready):
    \return 0 if nothing stored
    */
    void rfootTransformation( const matrix4d& inMatrix);

    /**
    \brief store matrix for the left foot. Temporary method (until new version is ready):
    \return 0 if nothing stored
     */
    void lfootTransformation( const matrix4d& inMatrix);


    ~ChppGikSupportPolygon();

private:
    ChppGikFootprint* attLeftFootprint;
    ChppGikFootprint* attRightFootprint;
    double attDiscRadiusSqr;
    matrix4d* attRfootM;
    matrix4d* attLfootM;
};


#endif
