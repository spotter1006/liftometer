#include "display.hpp"
#include <chrono>
#include <iostream>
#include "liftometer.hpp"
#include "PCA9685.h"
#include <cstring>
using namespace std;

extern timed_mutex mtxData;
extern Imu* pImu;
int nSampleSize;

Display::Display(){
    nSampleSize = 1000;
    m_nSlaveAddr = 0x40;
    // _PCA9685_DEBUG = 1; // uncomment to show PCA9685 debug info
    m_nFd = PCA9685_openI2C(1, 0x20);
    int nResult = PCA9685_initPWM(m_nFd, m_nSlaveAddr, 476);
}
Display::~Display(){
    pthread_cancel(m_tUpdater.native_handle());
    PCA9685_setAllPWM(m_nFd, m_nSlaveAddr, _PCA9685_MINVAL, _PCA9685_MINVAL);

}

int Display::updater(Display* pDisplay){
    double dRoll;
    double dPitch;
    double dYaw;
    double dAccel;
    unsigned int nOnVals[16];
    unsigned int nOffVals[16];

    memset(nOnVals,0,16);
    memset(nOffVals,0,16);

    int nResult = 0;
    chrono::steady_clock::time_point timePt;
    cout << "update display thread started" << endl;
    while(1){
        timePt = chrono::steady_clock::now() + chrono::milliseconds(500);    // 2 Hz indicator updates
        
        mtxData.lock();   
        dAccel = pImu->getAverageAccel(nSampleSize);
        dRoll = pImu->getAverageRoll(nSampleSize);
        dPitch = pImu->getAveragePitch(nSampleSize);
        dYaw = pImu->getAverageYaw(nSampleSize);
        mtxData.unlock();   
        
        /* Miuzei 9g servos:
            0        90      120 +/- 10      mechanical (degrees)
            900      1500    2100            high pulse width (uS)
            450      750     1050            counts in the PCA2865 "on" register
        
            2100 uS period = 476.19 Hz frequency 
            4096 counts per PWM cycle -> 37.24 - 31.5 counts per degree 
                  34 counts per degree
                  1 count is 2 uS       */     

        // Acceleration
        nOnVals[3] = 450 + dAccel * 100.0;      // TODO: scale for range
        nOffVals[3] - 4095 - nOnVals[3];

        // Roll
        nOnVals[0] = 450 + dRoll * 34.0;
        nOffVals[0] - 4095 - nOnVals[0];

        // Pitch
        nOnVals[1] = 450 + dRoll * 34.0;
        nOffVals[1] - 4095 - nOnVals[1];

        // Yaw
        nOnVals[2] = 450 + dYaw * 34.0;
        nOffVals[2] - 4095 - nOnVals[2];




        pDisplay->setPWMVals(nOnVals, nOffVals);

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

