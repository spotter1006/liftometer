#include "display.hpp"
#include <chrono>
#include <iostream>
#include "liftometer.hpp"
#include "PCA9685.h"
#include <cstring>
#include <math.h>
using namespace std;

extern timed_mutex mtxData;
extern Imu* pImu;
int nSampleSize;

Display::Display(){
    nSampleSize = 100;
    m_nSlaveAddr = 0x40;
    //_PCA9685_DEBUG = 1; // uncomment to show PCA9685 debug info
    m_nFd = PCA9685_openI2C(1, 0x20);
    int nResult = PCA9685_initPWM(m_nFd, m_nSlaveAddr, PWM_FREQUENCY);
}
Display::~Display(){
    PCA9685_setAllPWM(m_nFd, m_nSlaveAddr, _PCA9685_MINVAL, _PCA9685_MINVAL);
}

int Display::updater(Display* pDisplay){
    unsigned int nOnVals[16];
    unsigned int nOffVals[16];

    memset(nOnVals,0,16);
    memset(nOffVals,0,16);

    int nResult = 0;
    chrono::steady_clock::time_point timePt;

    while(1){
        timePt = chrono::steady_clock::now() + chrono::milliseconds(UPDATE_INTERVAL_MS);
        
        mtxData.lock();   
        double dAccelX = pImu->getAverageAccelX(nSampleSize);
        double dAccelY = pImu->getAverageAccelY(nSampleSize);       
        double dRoll = pImu->getAverageRoll(nSampleSize);
        double dPitch = pImu->getAveragePitch(nSampleSize);
        double dYawRateX = pImu->getAverageYawRateY(nSampleSize);
        double dYawRateY = pImu->getAverageYawRateX(nSampleSize);
        mtxData.unlock();   

        // IMU angle units are 1/16 of a degree
        dRoll /= 16.0;
        dPitch /= 16.0;
        double dYawRate = atan2(dYawRateY, dYawRateX) / 16.0;
        double dAccel = sqrt((double(dAccelX * dAccelX) + (double)(dAccelY * dAccelY))); 

        imuAngleToPwm(dRoll,    &nOnVals[0], &nOffVals[0]);
        imuAngleToPwm(dPitch,   &nOnVals[1], &nOffVals[1]);
        imuAngleToPwm(dYawRate, &nOnVals[2], &nOffVals[2]);
        imuAccelToPwm(dAccel, &nOnVals[3], &nOffVals[3]);

        pDisplay->setPWMVals(nOnVals, nOffVals);

        printf("\33[2K\rAverage(%d): Accel: %d, YawRate: %d, roll: %d pitch: %d", 
        nSampleSize, nOnVals[3], nOnVals[2], nOnVals[0], nOnVals[1]);
        fflush(stdout); 

        this_thread::sleep_until(timePt);
    }
    return nResult;
}
int Display::setPWMVals(unsigned int* nOnVals, unsigned int* nOffVals){
    return PCA9685_setPWMVals(m_nFd,m_nSlaveAddr,nOnVals, nOffVals);
}
int Display::start(){
     thread m_tUpdater(updater, this);
     m_tUpdater.detach();
     return 0;
}

void Display::imuAngleToPwm(double angle, unsigned int *on, unsigned int *off){
    int nOn =  PWM_ANGLE_OFFSET + (angle * PWM_ANGLE_SCALE);
    if (nOn < PWM_MIN)
        nOn = PWM_MIN;
    else if(nOn > PWM_MAX)
        nOn = PWM_MAX;
    int nOff = PWM_FULL_COUNT - nOn;
    *on = nOn;
    *off = nOff;
} 
void Display::imuAccelToPwm(double accel, unsigned int *on, unsigned int *off){
    int nOn =  PWM_ACCEL_OFFSET + (accel * PWM_ACCEL_SCALE);
    if (nOn < PWM_MIN)
        nOn = PWM_MIN;
    else if(nOn > PWM_MAX)
        nOn = PWM_MAX;
    int nOff = PWM_FULL_COUNT - nOn;
    *on = nOn;
    *off = nOff;
}
