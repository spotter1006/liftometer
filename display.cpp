#include "display.hpp"
#include "encoder.hpp"
#include <chrono>
#include <iostream>
#include "liftometer.hpp"
#include "PCA9685.h"
#include <cstring>
#include <math.h>
using namespace std;

extern Encoder *pEncoder;
extern timed_mutex mtxData;
extern Imu* pImu;
int nSampleSize;
unsigned int nOnVals[16];
unsigned int nOffVals[16];

Display::Display(){
    nSampleSize = 100;
    m_nSlaveAddr = 0x40;
    m_bKeepRunning = true;
    //_PCA9685_DEBUG = 1; // uncomment to show PCA9685 debug info
    m_nFd = PCA9685_openI2C(1, 0x20);
    int nResult = PCA9685_initPWM(m_nFd, m_nSlaveAddr, PWM_FREQUENCY);
}
Display::~Display(){
    PCA9685_setAllPWM(m_nFd, m_nSlaveAddr, _PCA9685_MINVAL, _PCA9685_MINVAL);
}

int Display::updater(Display* pDisplay){
    memset(nOnVals,0,16);
    memset(nOffVals,0,16);

    int nResult = 0;
    chrono::steady_clock::time_point timePt;
    ImuAveragedData average;
    while(pDisplay->isKeepRunning()){
        timePt = chrono::steady_clock::now() + chrono::milliseconds(UPDATE_INTERVAL_MS);
        ImuData latest;
        nSampleSize = pEncoder->getPosition();
        int nSamples = (pEncoder->getSwitchVal() == 0)? 1 : nSampleSize;  // Just latest measurements if switch depressed
        mtxData.lock();  
        pImu->getLatestHrp(&latest);
        pImu->getLatestGyro(&latest);
        pImu->getLatestAccel(&latest);

        pImu->getAverageHeading(nSampleSize, &average);
        mtxData.unlock();
        
        // 16 counts per degree from IMU
        imuAngleToPwm(latest.roll, &nOnVals[0], &nOffVals[0]);       
        imuAngleToPwm(latest.pitch, &nOnVals[1], &nOffVals[1]);      
        imuAngleToPwm(latest.heading, &nOnVals[2], &nOffVals[2]);    
        imuAngleToPwm(average.heading, &nOnVals[3], &nOffVals[3]);    
        
        /************* TODO:***************************
            - gyro.z for yaw rate (shows how the boat is sliding)
            - accel.x,y for performance indicator 
        *********************************************************/

        pDisplay->setPWMVals(nOnVals, nOffVals);

        // printf("\33[2K\rAverage(%d): Accel: %d, gyro: %d, roll: %d pitch: %d", 
        //     nSampleSize, nOffVals[3], nOffVals[2], nOffVals[0], nOffVals[1]);
        // // fflush(stdout); 

        this_thread::sleep_until(timePt);
    }
    return nResult;
}
int Display::setPWMVals(unsigned int* nOnVals, unsigned int* nOffVals){
    return PCA9685_setPWMVals(m_nFd,m_nSlaveAddr,nOnVals, nOffVals);
}
void Display::start(){
    m_bKeepRunning = true;
    thread m_tUpdater(updater, this);
    m_tUpdater.detach();
}
void Display::stop(){
    m_bKeepRunning = false;  
}

void Display::imuAngleToPwm(double angle, unsigned int *on, unsigned int *off, unsigned int phase){
    int nPulseWidth =  PWM_ANGLE_OFFSET + (angle * PWM_ANGLE_SCALE);
    if (nPulseWidth < PWM_MIN)  nPulseWidth = PWM_MIN;
    else if(nPulseWidth > PWM_RANGE) nPulseWidth = PWM_RANGE;

    *on = phase;            // Phase shift
    *off = PWM_MIN + phase + nPulseWidth - 1;
} 
