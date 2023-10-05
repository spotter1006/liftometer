#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include <pthread.h>
#include <thread>
#include "imu.hpp"

class Display{
    public:
        Display();
        ~Display();
        int start();
        int setPWMVals(unsigned int * nOnVals, unsigned int *nOffVals);
    private:
        static int updater(Display* pDisplay);
        std::thread m_tUpdater;
        int m_nFd;
        int m_nSlaveAddr;

};

#endif