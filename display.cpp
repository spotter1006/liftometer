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
    int nResult = PCA9685_initPWM(m_nFd, m_nSlaveAddr, 200);
}
Display::~Display(){
    pthread_cancel(m_tUpdater.native_handle());
    PCA9685_setAllPWM(m_nFd, m_nSlaveAddr, _PCA9685_MINVAL, _PCA9685_MINVAL);

}

int Display::updater(Display* pDisplay){
    double dRoll;
    double dPitch;
    double dHeading;
    double dAccel;
    unsigned int nOnVals[16];
    unsigned int nOffVals[16];

    memset(nOnVals,0,16);
    memset(nOffVals,0,16);

    int nResult = 0;
    chrono::steady_clock::time_point timePt;
    cout << "update display thread started" << endl;
    while(1){
        timePt = chrono::steady_clock::now() + chrono::milliseconds(200);    // 5 hz
        
        mtxData.lock();   
        dAccel = pImu->getAverageAccel(nSampleSize);
        dRoll = pImu->getAverageRoll(nSampleSize);
        dPitch = pImu->getAveragePitch(nSampleSize);
        dRoll = pImu->getAverageYaw(nSampleSize);
        mtxData.unlock();   
        
        // Acceleration
        double dPercent = dAccel /10.0;            // TODO: scale for range
        nOnVals[3] = dPercent * _PCA9685_MAXVAL;
        nOffVals[3] - _PCA9685_MAXVAL - nOnVals[3] - 1;
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

