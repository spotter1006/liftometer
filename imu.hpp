#ifndef _imu_h_
#define _imu_h_

#include "average.hpp"
#include "bno055.h"
#include <limits>
#include <thread>
#include <mutex>
using namespace std;


class Imu{
    public:
        Imu(int);
        ~Imu(void);

        // Thread  functions
        static int imuPoller(Imu*);
        int init();

        inline double getAverageAccel(int nSamples){return m_pAccel->calc((m_pAccel->size() > nSamples)? m_pAccel->size() : nSamples);}
        inline double getAverageRoll(int nSamples){return m_pRoll->calc(nSamples);}
        inline double getAveragePitch(int nSamples){return m_pPitch->calc(nSamples);}
        inline double getAverageYaw(int nSamples){return m_pYaw->calc(nSamples);}

        inline void lock(chrono::_V2 ::steady_clock::time_point tmUntil){m_mtxData.try_lock_until(tmUntil);}
        inline void unlock(void){m_mtxData.unlock();}
 
    private:
        
        int m_nBufferSize;
        Average* m_pRoll;
        Average* m_pPitch;
        Average* m_pYaw;
        Average* m_pAccel;
        timed_mutex m_mtxData;
        std::thread m_tPoller;
};
#endif