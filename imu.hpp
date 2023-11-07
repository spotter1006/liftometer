#ifndef _imu_h_
#define _imu_h_

#include "bno055.h"
#include <limits>
#include <thread>
#include <mutex>
#include <list>

#define SAMPLE_RATE_MS (20)
#define SAMPLES_PER_SECOND (1000 / SAMPLE_RATE_MS)
#define DATA_SIZE 20000000
using namespace std;

typedef struct IMU_DATA{
    short roll;
    short pitch;
    short heading;
    short gyroX;
    short gyroY;
    short accX;
    short accY;
}ImuData;

typedef struct ACCUMULATOR{
    int size;
    int nSamples;
    long sum;
}accumulator;

class Imu{
    public:
        Imu(int);
        ~Imu();
        void add(ImuData dataPoint);
        static void imuPoller(Imu*); // Main thread
        int start();
        void stop();
        void getData(ImuData *pData);
        int getAverageHeading(int nAverageIndex);
        int getHeadingAverageSamples(int index);
        inline void lock(){m_mtxData.try_lock_for(chrono::milliseconds(1));}
        inline void unlock(void){m_mtxData.unlock();}
        inline bool isKeepRunning(){return m_bKeepRunning;}
    private:        
        int m_nBufferSize;
        timed_mutex m_mtxData;
        bool m_bKeepRunning;
        list<ImuData> *m_pData;
        accumulator m_headingSums[8];  
};
#endif