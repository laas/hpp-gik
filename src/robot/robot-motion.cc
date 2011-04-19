#include <cstdio>
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include <jrl/mal/matrixabstractlayer.hh>
#include "hpp/gik/robot/robot-motion.hh"
#include "hpp/gik/tools.hh"

#define V3_I MAL_S3_VECTOR_ACCESS

using namespace boost::numeric::ublas;


ChppRobotMotion::ChppRobotMotion(CjrlHumanoidDynamicRobot* inRobot, double inStartTime, double inSamplingPeriod)
{
    attRobot = inRobot;
    attStartTime = inStartTime;
    attEndTime = attStartTime;
    attSamplingPeriod = inSamplingPeriod;
    attAccessedSample = attSamples.begin();
    attNumerSamples = 0;
}



void ChppRobotMotion::startTime(double inStartTime)
{
    attEndTime += inStartTime - attStartTime;
    attStartTime = inStartTime;
}

double ChppRobotMotion::startTime() const
{
    return attStartTime;
}

double ChppRobotMotion::endTime() const
{
    return attEndTime;
}

const CjrlHumanoidDynamicRobot& ChppRobotMotion::robot()
{
    return *attRobot;
}

bool ChppRobotMotion::getSampleAtTime(double inTime, ChppRobotMotionSample& outSample) const
{
    if (inTime >= attEndTime+attSamplingPeriod/2 || inTime < attStartTime+attSamplingPeriod/2 )
    {
        std::cout << "ChppRobotMotion::getSampleAtTime: requested time( "<< inTime<<" ) is out of range ( "<< attStartTime<<"; "<< attEndTime<<" )\n";
        return false;
    }
    unsigned int i = ChppGikTools::timetoRank(attStartTime,inTime,attSamplingPeriod)-1;

    std::vector<ChppRobotMotionSample> resVec;

    listGetRange(i,1,resVec);

    outSample = resVec[0];
    return true;
}

bool ChppRobotMotion::listGetRange(unsigned int index, unsigned int numberElements, std::vector<ChppRobotMotionSample>& outVectorConfig) const
{

    if ((index+numberElements-1)>=attSamples.size())
    {
        std::cout << " ChppRobotMotion::listGetRange() requested range is out of bounds\n";
        return false;
    }
    std::list<ChppRobotMotionSample>::const_iterator iter = attSamples.begin() ;
    for (unsigned int i = 0; i<index; i++)
        iter++;
    for (unsigned int i = 0; i< numberElements; i++)
    {
        outVectorConfig.push_back(*iter);
        iter++;
    }

    return true;
}

void ChppRobotMotion::appendSample(const ChppRobotMotionSample& inSample)
{
    attSamples.push_back(inSample);
    attEndTime += attSamplingPeriod;
    attNumerSamples++;
}

double ChppRobotMotion::samplingPeriod()
{
    return attSamplingPeriod;
}

bool ChppRobotMotion::appendMotion(const ChppRobotMotion& innMotion)
{

    ChppRobotMotion& inMotion = const_cast<ChppRobotMotion&>(innMotion);

    if (&(inMotion.robot()) != attRobot || (inMotion.samplingPeriod() != attSamplingPeriod) )
        return false;

    const ChppRobotMotionSample* sample =  inMotion.firstSample();

    while (sample)
    {
        appendSample(*sample);
        sample = inMotion.nextSample();
    }
    return true;
}

void ChppRobotMotion::clear()
{

    attSamples.clear();
    attEndTime = attStartTime;
    attAccessedSample = attSamples.begin();
    attNumerSamples = 0;
}

void  ChppRobotMotion::dumpTo(const char* inFilename, const char* option) const
{
    FILE* dumpFileQ;
    FILE* dumpFileRPY;
    FILE* dumpFileWST;
    FILE* dumpFileZMPwstObs;
    FILE* dumpFileZMPworObs;
    FILE* dumpFileZMPwstPla;
    FILE* dumpFileZMPworPla;

    char fnameQ[256],fnameRPY[256],fnameWST[256];
    sprintf(fnameQ,"%s.pos",inFilename);
    sprintf(fnameRPY,"%s.rpy",inFilename);
    sprintf(fnameWST,"%s.wst",inFilename);
    char fnameZMPwstObs[256],fnameZMPworObs[256],fnameZMPwstPla[256],fnameZMPworPla[256];
    sprintf(fnameZMPwstObs,"%s.zmpobs",inFilename);
    sprintf(fnameZMPwstPla,"%s.zmpref",inFilename);
    sprintf(fnameZMPworObs,"DEBUG%s.zmpobs",inFilename);
    sprintf(fnameZMPworPla,"DEBUG%s.zmpref",inFilename);

    dumpFileQ = fopen (fnameQ, option);
    dumpFileRPY = fopen (fnameRPY, option);
    dumpFileWST = fopen (fnameWST, option);
    dumpFileZMPwstObs = fopen (fnameZMPwstObs, option);
    dumpFileZMPworObs = fopen (fnameZMPworObs, option);
    dumpFileZMPwstPla = fopen (fnameZMPwstPla, option);
    dumpFileZMPworPla = fopen (fnameZMPworPla, option);


    //double time = attStartTime;
    double time = 0.0;

    std::list<ChppRobotMotionSample>::const_iterator iter= attSamples.begin();
    const std::vector<CjrlJoint*> openHRPJoints = attRobot->getActuatedJoints();
    unsigned int sz = openHRPJoints.size();
    CjrlJoint* joint;
    for (unsigned int j=0; iter!=attSamples.end(); j++)
    {
        fprintf(dumpFileQ,"%lf ",time);
        fprintf(dumpFileRPY,"%lf ",time);
        fprintf(dumpFileWST,"%lf ",time);

        for (unsigned int i=0;i<3;i++)
            fprintf (dumpFileWST, "%lf ", (*iter).configuration(i));
        fprintf (dumpFileWST,"\n");

        for (unsigned int i=3;i<6;i++)
            fprintf (dumpFileRPY, "%lf ", (*iter).configuration(i));
        fprintf (dumpFileRPY,"\n");

        for (unsigned int i = 0 ;i < sz; i++)
        {
            joint = openHRPJoints[i];
            fprintf (dumpFileQ, "%lf ", (*iter).configuration(joint->rankInConfiguration()));
        }
        fprintf (dumpFileQ,"\n");


        fprintf(dumpFileZMPwstObs,"%lf ",time);
        fprintf(dumpFileZMPworObs,"%lf ",time);
        fprintf(dumpFileZMPwstPla,"%lf ",time);
        fprintf(dumpFileZMPworPla,"%lf ",time);

        for (unsigned int i=0;i<3;i++)
        {
            fprintf (dumpFileZMPwstObs, "%lf ", V3_I((*iter).ZMPwstObs,i));
            fprintf (dumpFileZMPworObs, "%lf ", V3_I((*iter).ZMPworObs,i));
            fprintf (dumpFileZMPwstPla, "%lf ", V3_I((*iter).ZMPwstPla,i));
            fprintf (dumpFileZMPworPla, "%lf ", V3_I((*iter).ZMPworPla,i));
        }

        fprintf (dumpFileZMPwstObs, "\n");
        fprintf (dumpFileZMPworObs, "\n");
        fprintf (dumpFileZMPwstPla, "\n");
        fprintf (dumpFileZMPworPla, "\n");

        iter++;
        time += attSamplingPeriod;
    }

    fclose (dumpFileQ);
    fclose (dumpFileRPY);
    fclose (dumpFileWST);

    fclose (dumpFileZMPwstObs);
    fclose (dumpFileZMPworObs);
    fclose (dumpFileZMPwstPla);
    fclose (dumpFileZMPworPla);
}


bool ChppRobotMotion::empty() const
{
    return attSamples.size() == 0;
}

const ChppRobotMotionSample* ChppRobotMotion::firstSample()
{
    if (!empty())
    {
        attAccessedSample = attSamples.begin();
        return &(*attAccessedSample);
    }
    else
        return 0;
}

const ChppRobotMotionSample* ChppRobotMotion::previousSample()
{
    if (attAccessedSample != attSamples.begin())
        attAccessedSample--;
    return &(*attAccessedSample);
}

const ChppRobotMotionSample* ChppRobotMotion::nextSample()
{
    attAccessedSample++;
    if (attAccessedSample == attSamples.end())
        return 0;
    return &(*attAccessedSample);
}

const ChppRobotMotionSample* ChppRobotMotion::lastSample()
{
    if (empty())
        return NULL;
    attLastSample = attSamples.end();
    attLastSample--;
    return (&(*attLastSample));
}

unsigned int ChppRobotMotion::numberSamples() const
{
    return attNumerSamples;
}

ChppRobotMotion::~ChppRobotMotion()
{
}
