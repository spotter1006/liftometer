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
    ImuAveragedData averages;
    while(pDisplay->isKeepRunning()){
        timePt = chrono::steady_clock::now() + chrono::milliseconds(UPDATE_INTERVAL_MS);
        
        nSampleSize = pEncoder->getCount();
        int nSamples = (pEncoder->getSwitchVal() == 0)? 1 : nSampleSize;  // Just latest measurements if switch depressed
        mtxData.lock();   
        averages = pImu->getAveragedData(nSamples);
        mtxData.unlock();
        
        double dAccelAngle = atan2(averages.gyroY, averages.gyroX) * 180.0 / M_PI;
        double dAccel = sqrt(averages.accX * averages.accX + averages.accY * averages.accY); 
 

        imuAngleToPwm(averages.roll,    &nOnVals[0], &nOffVals[0]);
        imuAngleToPwm(averages.pitch,   &nOnVals[1], &nOffVals[1]);

        imuAngleToPwm(dAccelAngle, &nOnVals[2], &nOffVals[2]);
        imuAccelToPwm(dAccel, &nOnVals[3], &nOffVals[3]);

        pDisplay->setPWMVals(nOnVals, nOffVals);

        // printf("\33[2K\rAverage(%d): Accel: %d, gyro: %d, roll: %d pitch: %d", 
        //     nSampleSize, nOffVals[3], nOffVals[2], nOffVals[0], nOffVals[1]);
        fflush(stdout); 

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

void Display::imuAngleToPwm(double angle, unsigned int *on, unsigned int *off){
    int nPulseWidth =  PWM_MIN + PWM_ANGLE_OFFSET + (angle * PWM_ANGLE_SCALE);
    if (nPulseWidth < PWM_MIN)
        nPulseWidth = PWM_MIN;
    else if(nPulseWidth > PWM_MAX)
        nPulseWidth = PWM_MAX;

    *on = 0;            // Phase shift = 0
    *off = nPulseWidth - 1;
} 
void Display::imuAccelToPwm(double accel, unsigned int *on, unsigned int *off){
    int nPulseWidth =  PWM_MIN + (accel * PWM_ACCEL_SCALE);
    if (nPulseWidth < PWM_MIN)
        nPulseWidth = PWM_MIN;
    else if(nPulseWidth > PWM_MAX)
        nPulseWidth = PWM_MAX;

    *on = 0;        // Phase shift = 0
    *off = nPulseWidth -1;
}
