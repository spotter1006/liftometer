#ifndef _ENCODER_H_
#define __ENCODER_H_

#include <gpiod.hpp>
#include<thread>
#include <mutex>

 using namespace std;

class Encoder{
    public:
        Encoder();
        ~Encoder();
        int start();
        static int poller(Encoder*); // Main thread
        inline void lock(){m_mtxData.lock();}
        inline void unlock(void){m_mtxData.unlock();}
        inline void setValA(int nValA){m_nValA=nValA;}
        inline void setValB(int nValB){m_nValB=nValB;}
        inline int getValA(){return m_nValA;}
        inline int getValB(){return m_nValB;}
        inline void add(int n){m_nCount +=n;}
        int getCount();
    private:
        bool waitEdgeEvent(chrono::milliseconds msTimeout);
        int m_nCount;
        mutex m_mtxData;
        thread m_tPoller;
        gpiod::line m_lineA;
        gpiod::line m_lineB;
        int m_nValA;
        int m_nValB;
};

#endif