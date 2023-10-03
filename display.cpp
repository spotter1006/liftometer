#include "display.hpp"
#include <chrono>
#include <iostream>
#include "liftometer.hpp"
using namespace std;

extern timed_mutex mtxData;

Display::Display(){

}
Display::~Display(){
    pthread_cancel(m_tUpdater.native_handle());
}
int Display::updater(Display* pDisplay){
    int result = 0;
    chrono::steady_clock::time_point timePt;
    cout << "update display thread started" << endl;
    while(1){
        timePt = chrono::steady_clock::now() + chrono::milliseconds(500);    // 2 Hz
        mtxData.lock();
        // Access data here
        mtxData.unlock();
        // Update display here
        this_thread::sleep_until(timePt);
    }
    return result;
}
int Display::start(){
     thread m_tUpdater(updater, this);
     m_tUpdater.detach();
     return 0;
}