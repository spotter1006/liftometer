#ifndef _imu_h_
#define _imu_h_

#include "bno055.h"
#include <limits>
#include <thread>
#include <mutex>
#include <list>

#define SAMPLE_RATE_MS (10)
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
typedef struct IMU_AVERAGED_DATA{
    double roll;
    double pitch;
    double heading;
    double gyroX;
    double gyroY;
    double accX;
    double accY;
}ImuAveragedData;
class Imu{
    public:
        Imu(int);
        ~Imu();
        void add(ImuData dataPoint);
        static void imuPoller(Imu*); // Main thread
        int start();
        void stop();
        void getLatestHrp(ImuData *pData);
        void getLatestGyro(ImuData *pData);
        void getLatestAccel(ImuData *pData);
        void getAverageAccel(int nSamples, ImuAveragedData *pData);
        void getAverageHeading(int nSamples, ImuAveragedData *pData);
        inline void lock(chrono::_V2 ::steady_clock::time_point tmUntil){m_mtxData.try_lock_until(tmUntil);}
        inline void unlock(void){m_mtxData.unlock();}
        inline bool isKeepRunning(){return m_bKeepRunning;}
    private:        
        int m_nBufferSize;
        timed_mutex m_mtxData;
        bool m_bKeepRunning;
        list<ImuData> *m_pData;
};
#endif