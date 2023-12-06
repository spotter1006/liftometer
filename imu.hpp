#ifndef _imu_h_
#define _imu_h_

#include "bno055.h"
#include <limits>
#include <thread>
#include <mutex>
#include <list>
#include "average.hpp"

#define SAMPLE_RATE_MS (10)
#define SAMPLES_PER_SECOND (1000 / SAMPLE_RATE_MS)
#define DATA_SIZE 20000000
#define BUCKETS (8)

using namespace std;
typedef struct ACCUMULATOR{
    int size;
    int count;
    long sum;
    short oldestHeading;
}Accumulator;
typedef struct IMU_DATA{
    short roll;
    short pitch;
    short heading;
    short gyroX;
    short gyroY;
    short accX;
    short accY;
}ImuData;

class Imu{
    public:
        Imu(int);
        ~Imu();
        void add(ImuData dataPoint);
        static void imuPoller(Imu*); // Main thread
        int start();
        void stop();
        void getLatestData(ImuData *pData);
        int getHeadingAverageSamples(int i);
        int getAverageHeading(int i);
        inline bool isKeepRunning(){return m_bKeepRunning;}
        inline int getDataSize(){return m_pData->size();}
    private:        
        int m_nBufferSize;
        timed_mutex m_mtxData;
        bool m_bKeepRunning;
        list<ImuData> *m_pData;
        Average *m_pAverages[BUCKETS];      
};
#endif