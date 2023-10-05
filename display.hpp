#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include <pthread.h>
#include <thread>
#include "imu.hpp"

class Display{
    public:
        Display(Imu* pImu);
        ~Display();
        int start();
        void setSampleSize(int nSampleSize);
    private:
        Imu* m_pImu;
        static int updater(Display* pDisplay);
        std::thread m_tUpdater;
        int m_nSampleSize;
        int m_nFd;
        int m_nSlaveAddr;
        int m_nOnVals[16];
        int m_nOffVals[16];
};

#endif