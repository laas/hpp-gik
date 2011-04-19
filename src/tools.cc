#include <cstdio>

#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/io.hpp"
#include "hpp/gik/tools.hh"


#define M3_IJ MAL_S3x3_MATRIX_ACCESS_I_J
#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J
#define V3_I  MAL_S3_VECTOR_ACCESS

double ChppGikTools::attEps = 1e-8;
using namespace boost::numeric::ublas;

void ChppGikTools::Rodrigues(const vectorN& inW, double inAngle, matrixNxP& outRot)
{
    double nor = norm_2(inW);
    identity_matrix<double> id(3);
    if (nor < attEps)
    {
        outRot = id;
    }
    else
    {
        vectorN wn = inW / nor;
        equivAsymMat(wn,outRot);
        outRot = id + sin(inAngle)*outRot + (1-cos(inAngle)) * prod(outRot,outRot);
    }
}


void ChppGikTools::splitM4(const matrix4d& inM, matrix3d& outR, vector3d& outV)
{
    unsigned int i,j;
    for (i=0;i<3;i++)
    {
        for (j=0;j<3;j++)
            M3_IJ(outR,i,j) = M4_IJ(inM,i,j);
        V3_I(outV,i) = M4_IJ(inM,i,3);
    }
}

bool ChppGikTools::Matrix4dFromVec(const vectorN& inVec, matrix4d& outH)
{
    if (inVec.size()<6)
        return false;

    vector3d NE_wn;
    V3_I(NE_wn,0) = inVec(3);
    V3_I(NE_wn,1) = inVec(4);
    V3_I(NE_wn,2) = inVec(5);
    
    double th = MAL_S3_VECTOR_NORM(NE_wn);
    
    if (th< attEps)
    {
        MAL_S4x4_MATRIX_SET_IDENTITY(outH);
    }
    else
    {
        NE_wn = NE_wn / th;
        double ct = cos(th);
        double lct= (1-ct);
        double st = sin(th);
        M4_IJ(outH,0,0) = ct + NE_wn[0]*NE_wn[0]* lct;
        M4_IJ(outH,0,1) = NE_wn[0]*NE_wn[1]*lct-NE_wn[2]*st;
        M4_IJ(outH,0,2) = NE_wn[1] * st+NE_wn[0]*NE_wn[2]*lct;
        M4_IJ(outH,1,0) = NE_wn[2]*st +NE_wn[0]*NE_wn[1]*lct;
        M4_IJ(outH,1,1) = ct + NE_wn[1]*NE_wn[1]*lct;
        M4_IJ(outH,1,2) = -NE_wn[0]*st+NE_wn[1]*NE_wn[2]*lct;
        M4_IJ(outH,2,0) = -NE_wn[1]*st+NE_wn[0]*NE_wn[2]*lct;
        M4_IJ(outH,2,1) = NE_wn[0]*st + NE_wn[1]*NE_wn[2]*lct;
        M4_IJ(outH,2,2) = ct + NE_wn[2]*NE_wn[2]*lct;
    }

    M4_IJ(outH,0,3) = inVec(0);
    M4_IJ(outH,1,3) = inVec(1);
    M4_IJ(outH,2,3) = inVec(2);
    
    return true;
}

bool ChppGikTools::OmegaToR(const vectorN& inOmega, matrixNxP& outR)
{
    if (inOmega.size()!=3)
        return false;
    outR = identity_matrix<double>(3);
    double angle = norm_2(inOmega);
    if (angle > attEps)
    {
        vectorN axis =  inOmega/angle;
        Rodrigues(axis, angle, outR);
    }
    return true;
}

void ChppGikTools::EulerZYXtoRot(const vectorN& inEuler, matrixNxP& outRot)
{
    EulerZYXtoRot(inEuler(0), inEuler(1), inEuler(2), outRot);
}

void ChppGikTools::EulerZYXtoRot(double inThetaX, double inThetaY,
                                 double inThetaZ, matrixNxP& outRot)
{

    double cx = cos(inThetaX);
    double sx = sin(inThetaX);

    double cy = cos(inThetaY);
    double sy = sin(inThetaY);

    double cz = cos(inThetaZ);
    double sz = sin(inThetaZ);

    outRot(0,0) =cy*cz ;
    outRot(1,0) =cy*sz;
    outRot(2,0) =-sy;

    outRot(0,1) =cz*sx*sy-cx*sz;
    outRot(1,1) =cx*cz+sx*sy*sz;
    outRot(2,1) =cy*sx;

    outRot(0,2) =cx*cz*sy+sz*sx;
    outRot(1,2) =-cz*sx+cx*sy*sz;
    outRot(2,2) =cx*cy;
}

void ChppGikTools::RottoEulerZYX(const matrixNxP& inRot, vectorN& outEuler)
{
    double qz = atan2(inRot(1,0),inRot(0,0));
    double cz = cos(qz);
    double sz = sin(qz);
    double qx = atan2(inRot(0,2)*sz - inRot(1,2)*cz, -inRot(0,1)*sz + inRot(1,1)*cz);


    if (fabs(qx) > M_PI/2)
    {
        int sig = (qx>0)? 1 : -1;
        qz = qz - sig*M_PI;
        qx = qx - sig*M_PI;
        cz = -cz;
        sz = -sz;
    }
    double qy = atan2(-inRot(2,0),inRot(0,0)*cz + inRot(1,0)*sz);

    outEuler(0) = qx;
    outEuler(1) = qy;
    outEuler(2) = qz;
}

void ChppGikTools::M3toEulerZYX(matrix3d& inRot, vector3d& outEuler)
{
    double qz = atan2(M3_IJ(inRot,1,0),M3_IJ(inRot,0,0));
    double cz = cos(qz);
    double sz = sin(qz);
    double qx = atan2(M3_IJ(inRot,0,2)*sz - M3_IJ(inRot,1,2)*cz, -M3_IJ(inRot,0,1)*sz + M3_IJ(inRot,1,1)*cz);

    if (fabs(qx) > M_PI/2)
    {
        int sig = (qx>0)? 1 : -1;
        qz = qz - sig*M_PI;
        qx = qx - sig*M_PI;
        cz = -cz;
        sz = -sz;
    }
    double qy = atan2(-M3_IJ(inRot,2,0),M3_IJ(inRot,0,0)*cz + M3_IJ(inRot,1,0)*sz);

    outEuler[0] = qx;
    outEuler[1] = qy;
    outEuler[2] = qz;
}

void ChppGikTools::OmegatoEulerZYX(const vectorN& inOmega, vectorN& outEuler)
{
    matrixNxP attR(3,3);

    double th = norm_2(inOmega);
    Rodrigues(inOmega,th,attR);
    RottoEulerZYX(attR, outEuler);
}


void ChppGikTools::RottoOmega(const matrixNxP& inRot, vectorN& outVec)
{

    double alpha = (inRot(0,0)+inRot(1,1)+inRot(2,2)-1)/2;

    if (fabs(alpha-1) < attEps)
    {
        outVec(0) = 0;
        outVec(1) = 0;
        outVec(2) = 0;
    }
    else
    {
        double th = acos(alpha);
        double coef = 0.5*th/sin(th);

        outVec(0) = coef*(inRot(2,1)-inRot(1,2));
        outVec(1) = coef*(inRot(0,2)-inRot(2,0));
        outVec(2) = coef*(inRot(1,0)-inRot(0,1));
    }
}


void ChppGikTools::invertTransformation(const matrixNxP& inH, matrixNxP& outH)
{

    subrange(outH,0,3,0,3) = trans(subrange(inH,0,3,0,3));

    vectorN attT(3);

    attT = prod(subrange(outH,0,3,0,3),subrange(column(inH,3),0,3));
    for (unsigned int i=0; i< 3; i++)
        outH(i,3) = -attT(i);

    outH(3,0) = 0;
    outH(3,1) = 0;
    outH(3,2) = 0;
    outH(3,3) = 1;
}

void ChppGikTools::flyerTransformation(const vectorN& inDof, matrixNxP& outH)
{
    matrixNxP attR(3,3);

    EulerZYXtoRot(subrange(inDof,3,6),attR);
    subrange(outH,0,3,0,3) = attR;
    for (unsigned int i=0; i< 3; i++)
        outH(i,3) = inDof(i);
    outH(3,0) = 0;
    outH(3,1) = 0;
    outH(3,2) = 0;
    outH(3,3) = 1;
}

void ChppGikTools::equivAsymMat(const vectorN& inVec, matrixNxP& outH)
{

    outH(0,0) = 0;
    outH(0,1) = -inVec(2);
    outH(0,2) = inVec(1);
    outH(1,0) = inVec(2);
    outH(1,1) = 0;
    outH(1,2) = -inVec(0);
    outH(2,0) = -inVec(1);
    outH(2,1) = inVec(0);
    outH(2,2) = 0;
}



void ChppGikTools::CrossProduct(const vectorN &v1, const vectorN &v2, vectorN &retV)
{
    retV(0) = v1(1)*v2(2) - v1(2)*v2(1);
    retV(1) = v1(2)*v2(0) - v1(0)*v2(2);
    retV(2) = v1(0)*v2(1) - v1(1)*v2(0);
}

double ChppGikTools::eps()
{
    return attEps;
}

void ChppGikTools::Vector3toUblas(const vector3d& v, vectorN& uv)
{
    uv(0) = V3_I(v,0);
    uv(1) = V3_I(v,1);
    uv(2) = V3_I(v,2);
}

void ChppGikTools::UblastoVector3(const vectorN& uv, vector3d& v)
{
    V3_I(v,0) = uv(0);
    V3_I(v,1) = uv(1);
    V3_I(v,2) = uv(2);
}

void ChppGikTools::Matrix3toUblas(const matrix3d& mm, matrixNxP& um)
{
    matrix3d& m = const_cast<matrix3d&>(mm);

    for (unsigned int i=0;i<3;i++)
        for (unsigned int j=0;j<3;j++)
            um(i,j) = M3_IJ(m,i,j) ;

}

void ChppGikTools::UblastoMatrix3(const matrixNxP& um, matrix3d& m)
{

    for (unsigned int i=0;i<3;i++)
        for (unsigned int j=0;j<3;j++)
            M3_IJ(m,i,j) = um(i,j);
}

void ChppGikTools::Matrix4toUblas(const matrix4d& m, matrixNxP& um)
{
    for (unsigned int i=0;i<4;i++)
        for (unsigned int j=0;j<4;j++)
            um(i,j) = M4_IJ(m,i,j);
}

void ChppGikTools::UblastoMatrix4(const matrixNxP& um, matrix4d& m)
{
    for (unsigned int i=0;i<4;i++)
        for (unsigned int j=0;j<4;j++)
            M4_IJ(m,i,j) = um(i,j);
}


void ChppGikTools::printBlasMat(const matrixNxP& M)
{
    for (unsigned int i =0; i<M.size1(); i++)
    {
        for (unsigned int j =0; j<M.size2(); j++)
        {
            printf("%.4f\t",M(i,j));
        }
        std::cout << "\n";
    }
}

void ChppGikTools::filterWindow(double T, double dt, vectorN& outFilter)
{
    unsigned int n = (unsigned int)floor(T/dt);
    if (n<3) //???
    {
        outFilter.resize(1);
        outFilter(0) = 1;
        return;
    }
    outFilter.resize(n+1,false);
    double S = 0.0;
    for (unsigned int i=0;i<n+1;i++)
    {
        outFilter(i) = pow(sin(M_PI*i/n),2);
        S += outFilter(i);
    }
    outFilter /= S ;
}

bool ChppGikTools::sinFilter(vectorN& inSignal, double inSamplingPeriod, vectorN& outSignal)
{
    double window_width = 0.2;
    vectorN filterGains;
    filterWindow(window_width,inSamplingPeriod,filterGains);
    unsigned int filter_n = filterGains.size();
    outSignal.resize(inSignal.size(),false);

    vectorN extendedSignal(inSignal.size()+2*filter_n);
    vectorN result(inSignal.size()+2*filter_n);



    for (unsigned int i= 0; i<filter_n; i++)
    {
        extendedSignal(i) = inSignal(0);
        extendedSignal(filter_n+i+inSignal.size()) = inSignal(inSignal.size()-1);
    }
    subrange(extendedSignal,filter_n,filter_n+inSignal.size()) = inSignal;


    result.clear();
    for (unsigned int i= filter_n; i<extendedSignal.size(); i++)
        for (unsigned int j= 0; j<filter_n; j++)
            result(i) += extendedSignal(i-j)*filterGains(j);

    unsigned int start = (unsigned int)round(filter_n/2) + filter_n +1;
    outSignal = subrange(result,start,start+inSignal.size());
    return true;

}


bool ChppGikTools::multiFilter(double inSamplingPeriod, matrixNxP& inSignal, matrixNxP& outSignal)
{
    outSignal.resize(inSignal.size1(),inSignal.size2(),false);
    vectorN temp1,temp2;
    for (unsigned int i=0;i<inSignal.size1();i++)
    {
        temp1 = row(inSignal,i);
        //4 passes
        for (unsigned int j=0;j<4;j++)
        {
            sinFilter(temp1,inSamplingPeriod,temp2);
            temp1 = temp2;
        }
        row(outSignal,i) = temp2;
    }
    return true;
}

bool ChppGikTools::linearInterpolation(double D, double inSamplingPeriod, double xi, double xf, vectorN& outTrajectory)
{
    unsigned int n = (unsigned int)round(D/inSamplingPeriod);
    outTrajectory.resize(n+1,false);
    double alpha;
    for (unsigned int i =0; i<n+1;i++)
    {
        alpha = i/n;
        outTrajectory(i) = (1-alpha)*xi + alpha*xf;
    }
    return true;
}

bool ChppGikTools::multiLinearInterpolation(double D, double inSamplingPeriod, vectorN& xi, vectorN& xf, matrixNxP& outTrajectory)
{
    if (xi.size() != xf.size())
    {
        std::cout << "ChppGikTools::linearVectorInterpolation() Inital and final vector not of the same size\n" << std::endl;
        return false;
    }
    unsigned int n = (unsigned int)round(D/inSamplingPeriod);
    outTrajectory.resize(xi.size(),n+1,false);
    vectorN temp;
    for (unsigned int i =0; i<xi.size();i++)
    {
        linearInterpolation(D,inSamplingPeriod,xi(i),xf(i),temp);
        row(outTrajectory,i) = temp;
    }
    return true;
}

bool ChppGikTools::minJerkCurve(double D, double inSamplingPeriod, double xi, double vi, double ai, double xf, vectorN& outPosition, vectorN* outVelocity, vectorN* outAcceleration)
{

    double gap = xf - xi;
    double coeff[6];
    double D2 = D*D/2;
    double fracT = inSamplingPeriod/D;
    bool doVel = (outVelocity != 0);
    bool doAcc = (outAcceleration != 0);

    coeff[0] = xi;
    coeff[1] = D*vi;
    coeff[2] = D2*ai;
    coeff[3] = -3*D2 * ai - 6*D*vi + 10*gap;
    coeff[4] = 3*D2 * ai + 8*D*vi - 15*gap;
    coeff[5] = -D2 * ai - 3*D*vi + 6*gap;

    unsigned int nsamples = timetoRank(0,D,inSamplingPeriod) +1;

    if (nsamples < 2)
    {
        std::cout << "ChppGikTools::computeMinJerkMotion Too few samples\n";
        return false;
    }

    //resize //?
    //outPosition.resize(nsamples,false);
    outPosition.clear();

    if (doVel)
    {
        //outVelocity->resize(nsamples,false);
        outVelocity->clear();
    }
    if (doAcc)
    {
        //outAcceleration->resize(nsamples,false);
        outAcceleration->clear();
    }

    double ti = 0;
    for (unsigned int i=0; i<nsamples; i++)
    {
        ti += fracT;
        for (unsigned int j=0;j<6;j++)
            outPosition(i) += coeff[j]*pow(ti,(double)j);
    }

    ti = 0;

    if (doVel)
        for (unsigned int i=0; i<nsamples; i++)
        {
            ti += fracT;
            for (unsigned int j=1;j<6;j++)
                (*outVelocity)(i) += j*coeff[j]*pow(ti,(double)j-1)/D;
        }

    ti = 0;
    if (doAcc)
    {
        for (unsigned int i=0; i<nsamples; i++)
        {
            ti += fracT;

            for (unsigned int j=2;j<6;j++)
                (*outAcceleration)(i) += j*(j-1)*coeff[j]*pow(ti,(double)j-2)/D2;
        }
    }
    return true;
}


void ChppGikTools::RotFromYaw(double inYaw, matrixNxP& outM)
{
    if (outM.size1() != 3 || outM.size2() != 3)
        std::cout <<"ChppGikTools::RotFromYaw\n outMatrix of da wrong shape :)";

    outM(0,0) = cos(inYaw);
    outM(1,1) = outM(0,0);
    outM(1,0) = sin(inYaw);
    outM(0,1) = -outM(1,0);
    outM(2,2) = 1;
    outM(0,2) = 0;
    outM(1,2) = 0;
    outM(2,0) = 0;
    outM(2,1) = 0;
}

unsigned int ChppGikTools::timetoRank( double inStartTime, double inTime, double inSamplingPeriod)
{
    if (inStartTime>inTime)
        return 0;
    return (unsigned int)round((inTime-inStartTime)/inSamplingPeriod);
}

void ChppGikTools::HtoT(const matrix4d& inH, vectorN& outT)
{
    for (unsigned int i=0; i< 3; i++)
        outT(i) = M4_IJ(inH,i,3);
}

void ChppGikTools::HtoRT(const matrix4d& inH, matrixNxP& outRot, vectorN& outT)
{
    for (unsigned int i=0; i< 3; i++)
    {
        for (unsigned int j=0; j< 3; j++)
            outRot(i,j) = M4_IJ(inH,i,j);
        outT(i) = M4_IJ(inH,i,3);
    }
}

void ChppGikTools::targetTransformationU(const matrixNxP& referenceBase, const matrixNxP& referenceTarget, const matrixNxP& nowBase, matrixNxP& outNowTarget)
{
    //warning, no size checks, expects 4 homogeneous matrices !
    matrixNxP inverseRefBase(4,4);
    ChppGikTools::invertTransformation(referenceBase, inverseRefBase);
    noalias(outNowTarget) = prod(inverseRefBase, referenceTarget);
    outNowTarget = prod(nowBase,outNowTarget);
}

void ChppGikTools::targetTransformationM(const matrix4d& referenceBase, const matrix4d& referenceTarget, const matrix4d& nowBase, matrix4d& outNowTarget)
{
    //warning, no size checks, expects 4 homogeneous matrices !
    matrixNxP mat1(4,4);
    matrixNxP mat2(4,4);
    matrixNxP mat3(4,4);
    matrixNxP mat4(4,4);

    /* //when MAL_JRL is ready enable this and erase targetTransformationU
        matrix4d matr;
        MAL_S4x4_INVERSE(referenceBase,outNowTarget,double);
        MAL_S4x4_C_eq_A_by_B(matr,outNowTarget,referenceTarget);
        MAL_S4x4_C_eq_A_by_B(outNowTarget,nowBase,matr);
    */

    Matrix4toUblas( referenceBase,mat1);
    Matrix4toUblas( referenceTarget,mat2);
    Matrix4toUblas( nowBase,mat3);

    targetTransformationU(mat1,mat2,mat3,mat4);
    UblastoMatrix4( mat4, outNowTarget);

}

double ChppGikTools::inner_prod_3points(double originX, double originY, double point1X, double point1Y, double point2X, double point2Y)
{
    return (point1X-originX)*(point2X-originX) + (point1Y-originY)*(point2Y-originY);
}

void ChppGikTools::dumpMatrix(const char* inFilename, const matrixNxP& inData, double inStartTime, double inSamplingPeriod)
{
    FILE* dumpFile;
    dumpFile = fopen (inFilename, "w");

    //double time = inStartTime;
    double time = 0.0;
    for (unsigned int i=0; i!=inData.size2(); i++)
    {
        fprintf(dumpFile,"%lf ",time);
        for (unsigned int j= 0; j<inData.size1();j++)
            fprintf(dumpFile,"%lf ",inData(j,i));
        fprintf(dumpFile,"\n");
        time += inSamplingPeriod;
    }

    fclose (dumpFile);
}

void ChppGikTools::m4dFromXyzt(double inX, double inY, double inZ, double inTh, matrix4d& outMat)
{
    M4_IJ(outMat,0,0) = M4_IJ(outMat,1,1) = cos(inTh);
    M4_IJ(outMat,1,0) = sin(inTh);
    M4_IJ(outMat,0,1) = -M4_IJ(outMat,1,0);
    M4_IJ(outMat,0,3) = inX;
    M4_IJ(outMat,1,3) = inY;
    M4_IJ(outMat,2,3) = inZ;
}

void ChppGikTools::prolongateSizeBased(unsigned int size1, unsigned int size2, const matrixNxP& inData, matrixNxP& outData)
{
    unsigned int sizeM = size1+size2+inData.size2();
    outData.resize(inData.size1(),sizeM);

    unsigned int i;
    for (i=0; i<size1;i++)
        column(outData,i) = column(inData,0);

    for (i=0; i<inData.size2();i++)
        column(outData,i+size1) = column(inData,i);

    for (i=0; i<size2;i++)
        column(outData,i+size1+inData.size2()) = column(inData,inData.size2()-1);
}

void ChppGikTools::prolongateTimeBased(double timePre, double timePost, double samplingPeriod, const matrixNxP& inData, matrixNxP& outData)
{
    unsigned int sizePre = (unsigned int)round(timePre/samplingPeriod);
    unsigned int sizePost = (unsigned int)round(timePost/samplingPeriod);
    prolongateSizeBased(sizePre, sizePost, inData, outData);
}

bool ChppGikTools::overlapConcat(matrixNxP& data, const matrixNxP& addedData, unsigned int overlap)
{
    if (overlap > data.size2())
        return false;
    if (data.size1() != addedData.size1())
        return false;



    data.resize(data.size1(), data.size2() + addedData.size2()- overlap, true);
    subrange(data, 0, data.size1(), data.size2()-addedData.size2(),data.size2()) = addedData;
    return true;
}

bool ChppGikTools::combineMasks(const vectorN& inMask1, const vectorN& inMask2, vectorN& outMask)
{
    unsigned int sz1 = inMask1.size();
    unsigned int sz2 = inMask2.size();

    if (sz1 != sz2)
        return false;
    if (sz1 != outMask.size())
        return false;

    for (unsigned int i=0; i<sz1; i++)
        if ((inMask1(i) > 0) || (inMask2(i) > 0))
            outMask(i) = 1;
        else
            outMask(i) = 0;

    return true;

}
