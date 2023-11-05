#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <gpiod.hpp>
#include<thread>
#include <mutex>

#define ENCODER_LINE_A (23)
#define ENCODER_LINE_B (24)
#define SWITCH_LINE (25)

#define MASS (1.0)
#define FORCE_CONSTANT (10.0)
#define MAX_ACCELERATION (10)
#define VISCOUS_FRICTION_COEFFICIENT (0.7)
#define MOTION_INTERVAL_MS (1000)

 using namespace std;

class Encoder{
    public:
        Encoder();
        int start();
        void stop();
        static void poller(Encoder*); // Main thread
        static void motion(Encoder*); // Thread to calculate dynamic position
        inline void lock(){m_mtxData.lock();}
        inline void unlock(void){m_mtxData.unlock();}
        inline void setValA(int nValA){m_nValA=nValA;}
        inline void setValB(int nValB){m_nValB=nValB;}
        inline int getValA(){return m_nValA;}
        inline int getValB(){return m_nValB;}
        inline int getSwitchVal(){return m_nSwitchVal;}
        inline void setSwitchVal(int nVal){m_nSwitchVal = nVal;}
        void add(int n);
        int getCount();
        void clearCount();
        void calcVelocity(double acceleration);
        void calcPosition();
        inline int getPosition(){return (int)(m_dPosition + 0.5);}
        inline bool isKeepRunning(){return m_bKeepRunning;}
    private:
        int m_nCount;
        mutex m_mtxData;
        int m_nValA;
        int m_nValB;
        int m_nSwitchVal;
        bool m_bKeepRunning;
        double m_dVelocity;
        double m_dPosition;
};

#endif