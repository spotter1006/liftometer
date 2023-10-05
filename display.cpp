#include "display.hpp"
#include <chrono>
#include <iostream>
#include "liftometer.hpp"
#include "PCA9685.h"
#include <cstring>
using namespace std;

extern timed_mutex mtxData;

Display::Display(Imu* pImu){
    m_pImu = pImu;
    m_nSampleSize = 32;
    m_nSlaveAddr = 0x40;
    _PCA9685_DEBUG = 1; // uncomment to show PCA9685 debug info
    m_nFd = PCA9685_openI2C(1, 0x20);
    int nResult = PCA9685_initPWM(m_nFd, m_nSlaveAddr, 200);
    memset(m_nOnVals, 0, 16);
    memset(m_nOffVals, 0, 16);
}
Display::~Display(){
    pthread_cancel(m_tUpdater.native_handle());
    PCA9685_setAllPWM(m_nFd, m_nSlaveAddr, _PCA9685_MINVAL, _PCA9685_MINVAL);

}
void Display::setSampleSize(int nSampleSize){
    m_nSampleSize = nSampleSize;
}
int Display::updater(Display* pDisplay){
    double dRoll;
    double dPitch;
    double dHeading;
    double dAccel;

    int nResult = 0;
    chrono::steady_clock::time_point timePt;
    cout << "update display thread started" << endl;
    while(1){
        timePt = chrono::steady_clock::now() + chrono::milliseconds(1000);    // 1 Hz
        
        mtxData.lock();
        
        // Access data here

        
        mtxData.unlock();
        // dAccel = m_pImu->getAverageAccel(m_nSampleSize);
        // dRoll = m_pImu->getAverageRoll(m_nSampleSize);
        // dPitch = m_pImu->getAveragePitch(m_nSampleSize);
        // dRoll = m_pImu->getAverageYaw(m_nSampleSize);
        // PCA9685_setPWMVals(m_nFd, m_nFd, m_nOnVals, m_nOffVals);
        this_thread::sleep_until(timePt);
    }
    return nResult;
}
int Display::start(){
     thread m_tUpdater(updater, this);
     m_tUpdater.detach();
     return 0;
}

