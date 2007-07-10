#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "motionplanners/hppGikPreviewController.h"
#include "hppGikTools.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace ublas;

ChppGikPreviewController::ChppGikPreviewController(double inSamplingPeriod)
{
    if (fabs(inSamplingPeriod-5e-3)>1e-5 && fabs(inSamplingPeriod-5e-2)>1e-5)
    {
        std::cout << "ChppGikPreviewController: Constructor: The sampling period does not match the stored gains. Interrupting instance construction\n";
        return;
    }

    attPreviewTime = 1.6;
    attSamplingPeriod = inSamplingPeriod;
    attNumberPreviewSamples = ChppGikTools::timetoRank(0.0,attPreviewTime,attSamplingPeriod);
    if (fabs(inSamplingPeriod-5e-3)<1e-5)
        loadGains(GAINDIR "/hppGikPreviewController5ms.ini");
    else
        loadGains(GAINDIR "/hppGikPreviewController50ms.ini");
}

double ChppGikPreviewController::previewTime()
{
    return attPreviewTime;
}

unsigned int  ChppGikPreviewController::numberPreviewSamples()
{
    return attNumberPreviewSamples;
}

bool ChppGikPreviewController::loadGains(const char* inFileName)
{
    // reading a text file
    std::string lineF;
    std::string lineKx;
    std::string lineKs;
    std::string lineZg;

    std::string* stringList[4] = {&lineF,&lineKx,&lineKs,&lineZg};

    unsigned int i = 0;
    std::ifstream myfile(inFileName);
    if (myfile.is_open())
    {
        while (! myfile.eof() && i<4)
        {
            getline (myfile,*(stringList[i]));
            i++;
        }
        myfile.close();
    }
    else
    {
        std::cout << "ChppGikPreviewController::loadGains(): Unable to open file\n";
        return false;
    }

    std::istringstream stream;
    //gainsF
    unsigned int lastind;
    double readValue;
    stream.str(lineF);
    while (!stream.eof())
    {
        lastind = gainsF.size2();
        gainsF.resize(1,lastind+1,true);
        stream >> readValue;
        gainsF(0,lastind) = readValue;
    }
    if (gainsF.size2() != attNumberPreviewSamples)
    {
        std::cout << "ChppGikPreviewController::loadGains(): Bad F gains vector\n";
        std::cout << "expected length " << attNumberPreviewSamples <<"\n";
        std::cout << "found " << gainsF.size2() <<"\n";
        return false;
    }

    //gainsKx
    stream.clear();
    stream.str(lineKx);
    while (!stream.eof())
    {
        lastind = gainsKx.size2();
        gainsKx.resize(1,lastind+1,true);
        stream >> readValue;
        gainsKx(0,lastind) = readValue;
    }
    if (gainsKx.size2() != 3)
    {
        std::cout << "ChppGikPreviewController::loadGains(): Bad Kx gains vector\n";
        std::cout << "expected length " << 3 <<"\n";
        std::cout << "found " << gainsKx.size2() <<"\n";
        return false;
    }


    
    //gainsKs
    stream.clear();
    stream.str(lineKs);
    if (!stream.eof())
        stream >> gainsKs;
    else
    {
        std::cout << "ChppGikPreviewController::loadGains(): Ks value missing\n";
        return false;
    }

    
    //comHeight
    stream.clear();
    stream.str(lineZg);
    if (!stream.eof())
        stream >> comHeight;
    else
    {
        std::cout << "ChppGikPreviewController::loadGains(): Zg value missing\n";
        return false;
    }
    
    systemA.resize(3,3,false);
    systemA = ublas::identity_matrix<double>(3);

    systemA(0,1) = attSamplingPeriod;
    systemA(1,2) = attSamplingPeriod;
    systemA(0,2) = attSamplingPeriod*attSamplingPeriod;

    systemB.resize(3,1,false);
    systemB(0,0) = attSamplingPeriod*attSamplingPeriod*attSamplingPeriod/6;
    systemB(1,0) = attSamplingPeriod*attSamplingPeriod/2;
    systemB(2,0) = attSamplingPeriod;

    systemC.resize(1,3,false);
    systemC(0,0) = 1;
    systemC(0,1) = 0;
    systemC(0,2) = -comHeight/9.8;
    
    u.resize(1,2,false); 
    plannedZMP.resize(1,2,false); 
    return true;
}

bool ChppGikPreviewController::ZMPtoCOM(const matrixNxP& inZMP, matrixNxP& outCOM)
{
    unsigned int ZMPlength = inZMP.size2();
    
    if (inZMP.size1() <=2)
    {
        std::cout << "ChppGikPreviewController::ZMPtoCOM(). ZMP matrix must have a least two columns\n";
        return false;
    }
    
    if (ZMPlength < attNumberPreviewSamples)
    {
        std::cout << "ChppGikPreviewController::ZMPtoCOM(). Not enough ZMP samples to compute a single COM state.\n";
        return false;
    }
    
    matrixNxP ZMPref = trans(inZMP);
    
    //resize COM output matrix
    outCOM.resize(2, ZMPlength-attNumberPreviewSamples+1, false);

    matrixNxP ZMPChunk(attNumberPreviewSamples,2);
    matrixNxP currentComState = ublas::zero_matrix<double>(3,2);
    tempoCalc = currentComState;
    matrixNxP plannedZMPError(1,2);
    plannedZMPError(0,0) = 0;
    plannedZMPError(0,1) = 0;

    double startZMPoffsetX = ZMPref(0,0),startZMPoffsetY = ZMPref(0,1);
    
    for (unsigned int i=0;i < ZMPlength;i++)
    {
        ZMPref(i,0) = ZMPref(i,0) - startZMPoffsetX;
        ZMPref(i,1) = ZMPref(i,1) - startZMPoffsetY;
    }

    for (unsigned int i=0;i < ZMPlength-attNumberPreviewSamples+1;i++)
    {
        ZMPChunk = ublas::subrange(ZMPref,i,i+attNumberPreviewSamples,0,2);
        singleZMPtoCOMState(ZMPChunk, currentComState, plannedZMPError);
        outCOM(0,i) = currentComState(0,0) + startZMPoffsetX;
        outCOM(1,i) = currentComState(0,1) + startZMPoffsetY;
    }
    
    return true;
}

void ChppGikPreviewController::singleZMPtoCOMState(matrixNxP& inZMPChunk, matrixNxP& inCOMState, matrixNxP& inZmpError)
{
    // OPTIMAL COM (LQR on JERK)
    //----------------------------
    u = (gainsKs * inZmpError);
    noalias(u) += prod(gainsF,inZMPChunk);
    noalias(u) -= prod(gainsKx,inCOMState);
    //state after preview compensation
    noalias(tempoCalc) = prod(systemA,inCOMState);
    noalias(tempoCalc) += prod(systemB, u);
    inCOMState = tempoCalc;
    //predicted zmp after preview compensation
    noalias(plannedZMP) = prod(systemC, inCOMState);
    //integral zmp error in simplified model after application of optimal command
    inZmpError +=  subrange(inZMPChunk,0,1,0,2) - plannedZMP;
}





