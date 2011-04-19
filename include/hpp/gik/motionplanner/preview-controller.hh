#ifndef HPP_GIK_PREVIEW_CONTROLLER_H
#define HPP_GIK_PREVIEW_CONTROLLER_H

#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"

class ChppGikPreviewController
{
public:
    /**
    \brief Constructor
    \param inSamplingPeriod: currently only two values supported : 5e-3 and 5e-2. If the value entered in the constructor call is different the instance cannot be constructed.
    The used gains were computed for a robot having its center of mass at height 0.80914.
    The program using this class needs to find either of the files "preview-controller-5ms.ini" and "preview-controller-50ms.ini" in the running directory
     */
    ChppGikPreviewController(double inSamplingPeriod);
    /**
    \brief Compute COM motion from ZMP motion.
    \param inZMP is a two-row matrix; column1 for X and column2 for Y. the first column gives the initial position of the center of mass
    \param outCOM is a two-row matrix used to store the computed com trajectory. The number of computed samples is equal to numberZMPSamples - numberPreviewSamples() +1. outCOM is automatically resized.
    \return false if inZMP does not hold enough samples or other paramters fail to match expected (minimum)size
     */
    bool ZMPtoCOM(const matrixNxP& inZMP, matrixNxP& outCOM);
    /**
    \brief The time corresponding to the preview window
     */
    double previewTime();
    
    /**
    \brief The number of ZMP samples required to plan a single COM state
     */
    unsigned int numberPreviewSamples();

    /**
    \brief Destructor
     */
    ~ChppGikPreviewController()
    {}


private:
    /**
    \brief Load preview controller gains from file
     */
    bool loadGains(const char* inFileName);
    /**
    \brief Compute a single COM state from a previewed chunck of ZMP positions
     */
    void singleZMPtoCOMState(matrixNxP& inZMPChunk, matrixNxP& inCOMState, matrixNxP& inZmpError);

    double attPreviewTime;

    double attSamplingPeriod;

    unsigned int attNumberPreviewSamples;

    double  comHeight;
    double  gainsKs;
    matrixNxP    gainsKx;
    matrixNxP    gainsF;
    matrixNxP    systemA;
    matrixNxP    systemB;
    matrixNxP    systemC;
    matrixNxP    u;
    matrixNxP    plannedZMP;
    matrixNxP   tempoCalc;
};

#endif
