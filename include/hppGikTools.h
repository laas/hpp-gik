#ifndef HPP_GIK_TOOLS_H
#define HPP_GIK_TOOLS_H

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"


class ChppGikTools
{
public :

    static void Rodrigues(const vectorN& inAxisVector, double inRotationAngle, matrixNxP& outRot);

    static void RottoOmega(const matrixNxP& inRot, vectorN& outVec);

    /**
    Get euler angles stored thetaX, thetaY, thetaZ inside outEuler. The entered rotation matrix is: inRot = Rot(vecZ,thetaZ) * Rot(vecY',thetaY) * Rot(vecX'',thetaX)
    \return true if no singularity was encoutered, false otherwise
     */
    static bool RottoEulerZYX(const matrixNxP& inRot, vectorN& outEuler);
    /**
    Same as RottoEulerZYX but from instantaneous rotation vector inOmega.
    */
    static bool OmegatoEulerZYX(const vectorN& inOmega, vectorN& outEuler);

    /**
    Inverse of RottoEulerZYX
     */
    static void EulerZYXtoRot(const vectorN& inEuler, matrixNxP& outRot);

    //inH and outH are supposed distinct
    static void invertTransformation(const matrixNxP& inH, matrixNxP& outH);

    static void flyerTransformation(const vectorN& inDof, matrixNxP& outH);

    static void HtoT(const matrix4d& inH, vectorN& outT);

    static void HtoRT(const matrix4d& inH, matrixNxP& outRot, vectorN& outT);

    static void equivAsymMat(const vectorN& inVec, matrixNxP& outH);

    static void CrossProduct(const vectorN &v1, const vectorN &v2, vectorN &ret);

    static double eps();

    static void Vector3toUblas(const vector3d& v, vectorN& uv);
    static void UblastoVector3(const vectorN& uv, vector3d& v);

    static void Matrix3toUblas(const matrix3d& m, matrixNxP& um);
    static void UblastoMatrix3(const matrixNxP& um, matrix3d& m);

    static void Matrix4toUblas(const matrix4d& m, matrixNxP& um);
    static void UblastoMatrix4(const matrixNxP& um, matrix4d& m);

    static void printBlasMat(const matrixNxP& M);

    static void RotFromYaw(double inYaw, matrixNxP& outM);

    /**
    \brief Produce a trajectory interpolating xi and xf given the intial first and second derivatives of x (vi and ai). The final first and second derivatives (following the previous notations, those would be denoted vf and af) are null. The computed trajectory samples are appended to outPosition. Their number is N = round(inMotionDuration/inSamplingPeriod) + 1.
    if outVelocity and outAcceleration pointers are given, the first and second derivative of the trajectory are also computed and returned in these.
     */
    static bool minJerkCurve(double inMotionDuration, double inSamplingPeriod, double xi, double vi, double ai, double xf, vectorN& outPosition, vectorN* outVelocity = 0, vectorN* outAcceleration = 0);

    /**
    \brief return n = round((inTime-startTime)/inSamplingPeriod)
    */
    static unsigned int timetoRank(double startTime, double inTime, double inSamplingPeriod);

    static void targetTransformationU(const matrixNxP& referenceBase, const matrixNxP& referenceTarget, const matrixNxP& nowBase, matrixNxP& outNowTarget);

    static void targetTransformationM(const matrix4d& referenceBase, const matrix4d& referenceTarget, const matrix4d& nowBase, matrix4d& outNowTarget);

    static double inner_prod_3points(double originX, double originY, double point1X, double point1Y, double point2X, double point2Y);

    /**
    \brief matrix has samples in concatenated columns
    */
    static void dumpMatrix(const char* inFilename, const matrixNxP& inData, double inStartTime, double inSamplingPeriod);

    static bool sinFilter(vectorN& inSignal, double inSamplingPeriod, vectorN& outSignal);
    
    static void filterWindow(double T, double dt, vectorN& outFilter);
    
    static bool multiFilter(double inSamplingPeriod, matrixNxP& inSignal, matrixNxP& outSignal);

    static bool linearInterpolation(double D, double inSamplingPeriod, double xi, double xf, vectorN& outTrajectory);

    static bool multiLinearInterpolation(double D, double inSamplingPeriod, vectorN& xi, vectorN& xf, matrixNxP& outTrajectory);

    


private:

    static double attEps;
};

#endif

