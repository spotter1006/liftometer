#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include <pthread.h>
#include <thread>

class Display{
    public:
        Display();
        ~Display();
        int start();
        void setSampleSize(int nSampleSize);
    private:
        static int updater(Display* pDisplay);
        std::thread m_tUpdater;
        int m_nSampleSize;
};

#endif