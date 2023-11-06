#ifndef _imu_h_
#define _imu_h_

#include "bno055.h"
#include <limits>
#include <thread>
#include <mutex>
#include <list>

#define SAMPLE_RATE_MS (10)
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
        long getHeadingSum(int index);
        int getHeadingAverageSamples(int index);
        int getAverageHeading(int nAverageIndex);
        inline void lock(chrono::_V2 ::steady_clock::time_point tmUntil){m_mtxData.try_lock_until(tmUntil);}
        inline void unlock(void){m_mtxData.unlock();}
        inline bool isKeepRunning(){return m_bKeepRunning;}
    private:        
        int m_nBufferSize;
        timed_mutex m_mtxData;
        bool m_bKeepRunning;
        list<ImuData> *m_pData;
        long m_nHeadingSums[8] = {0, 0, 0, 0, 0, 0, 0, 0};  
        int m_nHeadingSamples[8] = {
                                5 * SAMPLES_PER_SECOND, 
                                10 * SAMPLES_PER_SECOND, 
                                20 * SAMPLES_PER_SECOND, 
                                40 * SAMPLES_PER_SECOND, 
                                80 * SAMPLES_PER_SECOND,
                                160 * SAMPLES_PER_SECOND, 
                                320 * SAMPLES_PER_SECOND, 
                                640 * SAMPLES_PER_SECOND };  // 10 minutes, 40 seconds
        void updateHeadingSums(ImuData dataPoint);

};
#endif