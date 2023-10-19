#ifndef _ENCODER_H_
#define __ENCODER_H_

#include <gpiod.hpp>
#include<thread>
#include <mutex>

#define ENCODER_LINE_A (23)
#define ENCODER_LINE_B (24)
#define SWITCH_LINE (25)

 using namespace std;

class Encoder{
    public:
        Encoder();
        ~Encoder();
        int start();
        void stop();
        static int poller(Encoder*); // Main thread
        inline void lock(){m_mtxData.lock();}
        inline void unlock(void){m_mtxData.unlock();}
        inline void setValA(int nValA){m_nValA=nValA;}
        inline void setValB(int nValB){m_nValB=nValB;}
        inline int getValA(){return m_nValA;}
        inline int getValB(){return m_nValB;}
        inline int getSwitchVal(){return m_nSwitchVal;}
        void add(int n);
        int getCount();
        inline bool isKeepRunning(){return m_bKeepRunning;}
    private:
        int waitEdgeEvent(chrono::milliseconds msTimeout);
        int m_nCount;
        mutex m_mtxData;
        thread m_tPoller;
        gpiod::line m_lineA;
        gpiod::line m_lineB;
        gpiod::line m_lineSwitch;
        int m_nValA;
        int m_nValB;
        int m_nSwitchVal;
        bool m_bKeepRunning;
};

#endif