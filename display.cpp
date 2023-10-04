#include "display.hpp"
#include <chrono>
#include <iostream>
#include "liftometer.hpp"
using namespace std;

extern timed_mutex mtxData;

Display::Display(){
    m_nSampleSize = 32;
}
Display::~Display(){
    pthread_cancel(m_tUpdater.native_handle());
}
void Display::setSampleSize(int nSampleSize){
    m_nSampleSize = nSampleSize;
}
int Display::updater(Display* pDisplay){
    int result = 0;
    chrono::steady_clock::time_point timePt;
    cout << "update display thread started" << endl;
    while(1){
        timePt = chrono::steady_clock::now() + chrono::milliseconds(1000);    // 1 Hz
        mtxData.lock();
        // Access data here

        mtxData.unlock();
        
        this_thread::sleep_until(timePt);
    }
    return result;
}
int Display::start(){
     thread m_tUpdater(updater, this);
     m_tUpdater.detach();
     return 0;
}