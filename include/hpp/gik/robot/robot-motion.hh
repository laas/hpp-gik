#ifndef HPP_ROBOT_MOTION_H
#define HPP_ROBOT_MOTION_H

#include <list>
#include "abstract-robot-dynamics/traits/default-pointer.hh"
#include "abstract-robot-dynamics/abstract-robot-dynamics.hh"
#include "abstract-robot-dynamics/humanoid-dynamic-robot.hh"

typedef vectorN CjrlRobotConfiguration;

/**
\brief This is a stored robot motion sample. It contains the full configuration of the robot at some time, as well as the position of the ZMP observed and planned , in the waist frame and in the world frame.
\ingroup robot
*/
class ChppRobotMotionSample
{
    public:
       
        /**
        \brief Root pose
         */
        matrix4d rootpose;
        
        /**
        \brief OpenHRP configuration of the robot
         */
        vectorN configuration;
        
        /**
        \brief Velocity
         */
        vectorN velocity;
        
        /**
        \brief Acceleration
         */
        vectorN acceleration;
        
        /**
        \brief Planned Zero Momentum Point in waist frame
         */
        vector3d ZMPwstPla;
        
        /**
        \brief Planned Zero Momentum Point in world frame
         */
        vector3d ZMPworPla;
        
        /**
        \brief Motion-resulting (Observed) Zero Momentum Point in waist frame
         */
        vector3d ZMPwstObs;
        
        /**
        \brief Motion-resulting (Observed) Zero Momentum Point in world frame
        */
        vector3d ZMPworObs;
        
};


class ChppRobotMotion;
/**
\brief Defines the motion of a robot along time.
The motion is stored as a list of ublas vectors.
\ingroup robot
 */
class ChppRobotMotion
{
public:

    /**
    \brief Constructor.
     */
    ChppRobotMotion(CjrlHumanoidDynamicRobot* inRobot, double inStartTime, double inSamplingPeriod);
    /**
    \brief Destructor.
     */
    ~ChppRobotMotion();

    /**
    \brief Set lower bound of definition interval.
     */
    void startTime(double inStartTime);

    /**
    \brief Get lower bound of definition interval.
     */
    double startTime() const;

    /**
    \brief Get upper bound of defintion interval.
     */
    double endTime() const;

    /**
    \brief Get robot for which motion is defined.
     */
    const CjrlHumanoidDynamicRobot& robot();

    /**
    \brief Get the sampling period of this motion
    */
    double samplingPeriod();
    /**
    \brief Append a new sample 
     */
    void appendSample(const ChppRobotMotionSample& inSample);
    /**
    \brief Append another robot motion. The passed motion is copied sample by sample and added to the end of this motion.
    \return false in case the robot associated to the passed motion is different.
    */
    bool appendMotion(const ChppRobotMotion& inMotion);

    /**
    \brief Delete stored motion
     */
    void clear();

    /**
    \brief Motion dumper:
    
    5 files are produced for openHRP
       joint configuration
       freeflyer position
       freeflyer orientation 
       zmp reference motion in waist frame
       observed zmp motion in waist frame
    2 files are produced for debug:
       zmp reference motion in world frame
       zmp for solution motion in world frame
    
    In all these files, a data sample is output to a line beginning by the sample time, starting from 0.0.
    */
    void dumpTo(const char* inFilename, const char* option="w") const;


    /**
    \brief Tell if this robot motion object is empty
    */
    bool empty() const;
    /**
    \brief This object keeps an iterator on the list of stored samples and it is reset by this method.
    \return a pointer to the first element in the list of stored samples. 0 is returned if the list is empty
    */
    const ChppRobotMotionSample* firstSample();
    /**
    \brief This object keeps an iterator on the list of stored samples and it is incremented by this method to access the samples list sequentially. 
    \return 0 if the end of the list is met.
     */
    const ChppRobotMotionSample* nextSample();
    /**
    \brief This object keeps an iterator on the list of stored samples and it is decremented by this method. 
    \return first sample if current is sample is first, previous otherwise.
     */    
    const ChppRobotMotionSample* previousSample();
    
    /**
    \brief Get a pointer to the last sample stored. The returned pointer is null in case the motion is empty.
    */
    const ChppRobotMotionSample* lastSample();
    
    /**
    \brief Copy the sample corresponding to time \param inTime in \param outSample
    \return false if time out of bounds
    */
    bool getSampleAtTime(double inTime, ChppRobotMotionSample& outSample) const;
    
    /**
    \brief Get the number of samples
     */
    unsigned int numberSamples() const;
    
    
    
    



private:

    bool listGetRange(unsigned int index, unsigned int numberElements, std::vector<ChppRobotMotionSample>& outVectorConfig) const;

    CjrlHumanoidDynamicRobot* attRobot;

    double attStartTime;

    double attEndTime;

    double attSamplingPeriod;

    std::list<ChppRobotMotionSample> attSamples;
    
    std::list<ChppRobotMotionSample>::iterator attAccessedSample;
    
    unsigned int attNumerSamples;
    
    std::list<ChppRobotMotionSample>::iterator attLastSample;
    

};

#endif
