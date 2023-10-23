#ifndef _imu_h_
#define _imu_h_

#include "average.hpp"
#include "bno055.h"
#include <limits>
#include <thread>
#include <mutex>

#define SAMPLE_RATE_MS (200)
using namespace std;


class Imu{
    public:
        Imu(int);
        ~Imu();
        static void imuPoller(Imu*); // Main thread
        int start();
        void stop();
        double getAverageRoll(int nSamples);
        double getAveragePitch(int nSamples);
        double getAccelRange(int nSamles);
        double getAverageAccelX(int nSamples);
        double getAverageAccelY(int nSamples);
        double getAverageYawRateX(int nSamples);
        double getAverageYawRateY(int nSamples);
        inline void lock(chrono::_V2 ::steady_clock::time_point tmUntil){m_mtxData.try_lock_until(tmUntil);}
        inline void unlock(void){m_mtxData.unlock();}
        inline bool isKeepRunning(){return m_bKeepRunning;}
    private:        
        int m_nBufferSize;
        Average* m_pRoll;
        Average* m_pPitch;
        Average* m_pYawRateX;
        Average* m_pYawRateY;
        Average* m_pAccelX;
        Average* m_pAccelY;
        timed_mutex m_mtxData;
        bool m_bKeepRunning;
};
#endif