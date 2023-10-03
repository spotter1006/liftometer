#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include <pthread.h>
#include <thread>

class Display{
    public:
        Display();
        ~Display();
        int start();
    private:
        static int updater(Display* pDisplay);
        std::thread m_tUpdater;
};

#endif