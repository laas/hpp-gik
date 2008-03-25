#ifndef HPP_FOOTPRINT_RELATED_H
#define HPP_FOOTPRINT_RELATED_H

#include <vector>
#include <iostream>
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"

/**
\brief A footprint is a shape (not defined) located at (attX, attY, 0.0) in the world frame and is rotated by attTheta around OZ.  The local frame is set as follows: locOX points ahead (to the toes), locOY points to the left and z points up.
 */
class ChppGikFootprint;
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
    bool isPointInside(double inX, double inY)const;
    
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
\brief This is a foot ! A footprint with the information right or left
 */
class ChppGikFoot
{
public:
    /**
    \constructor
    */
    ChppGikFoot(const ChppGikFootprint&  inFootprint, bool isRight);

    /**
    \copy constructor
     */
    ChppGikFoot(const ChppGikFoot&  sourceObject);

    /**
    \brief Get the associated footprint
    */
    const ChppGikFootprint& footprint() const;

    /**
    \brief Is this the right foot
    */
    bool isRight() const;
    
    /**
    \brief destructor
    */
    ~ChppGikFoot();

private:
    bool attIsRight;
    ChppGikFootprint* attFootprint;
};

/**
\brief This is a support polygon class. A support polygon is defined by at least one foot print.
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
    \brief Tell if the given 2D point is inside this support polygon. A weak version is implemented: if the support polygon is simple, check wether the given point is in the disc centered on the orthogonal projection of the foot ankle on the ground. The radius is arbitrarily set to 4 cm. If the support is double, see if the point is in the area covered by the continuous translation of the same disc from one ankle projection to the other.
     */
    bool isPointInside(double x, double y) const;

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




#include "boost/numeric/ublas/matrix.hpp"
namespace ublas = boost::numeric::ublas;

/**
\brief This is a support polygon motion class. The successive support polygons are atored in a vector and the corresponding start and end times are stored in a separate vector.
*/
class ChppGikSupportPolygonMotion
{
public:
    /**
    \brief Constructor
     */
    ChppGikSupportPolygonMotion(double inStartTime);
    /**
    \brief reset suport polygon stack
     */
    void reset(double inStartTime);
    /**
    \brief Append the given support polygon to the end of the motion (a copy of inSupportPolygon is made)
     */
    void pushbackSupportPolygon(ChppGikSupportPolygon* inSupportPolygon, double inDuration);
    /**
    \brief Append the given support polygon at the beginning of the motion (a copy of inSupportPolygon is made)
     */
    void pushfrontSupportPolygon(ChppGikSupportPolygon* inSupportPolygon, double inDuration);
    /**
    \brief Extend the last support polygon recorded by the given time
     */
    void extendLastSupportPolygonDuration(double inExtraTime);
    /**
    \brief Get the support polygon at the given time
    \return null pointer if invalid time
     */
    const ChppGikSupportPolygon* lastSupportPolygon() const;
    /**
    \brief Get the support polygon at the given time
    \return null pointer if invalid time
     */
    const ChppGikSupportPolygon* supportPolygonAtTime(double inTime) const;
    /**
    \return false is right foot is not on the ground at the given time
     */
    bool isRightLegSupportingAtTime(double inTime) const;
    /**
    \return false is left foot is not on the ground at the given time
     */
    bool isLeftLegSupportingAtTime(double inTime) const;
    /**
    \return false if single support at the given time
     */
    bool isDoubleSupportAtTime(double inTime) const;

    /**
    \brief Tell if the given trajectory lies within the planned support polygon motion
    inTrajectory is a matrix with two rows: first for xZMP and second for yZMP
     */
    bool isTrajectoryInside(const matrixNxP& inTrajectory, double inStartTime, double inSamplingPeriod) const;

    /**
    \brief Get the motion end time
     */
    double endTime() const;
    /**
    \brief Get the motion start time
     */
    double startTime() const;
    /**
    \brief Tell if support vector motion is empty
     */
    bool empty();
    /**
    \brief Destructor
     */
    ~ChppGikSupportPolygonMotion();

private:
    double attStartTime;
    double attEndTime;
    std::vector<ChppGikSupportPolygon*> attVectorSupportPolygon;
    std::vector<double> attVectorStartTimes;

};

#endif
