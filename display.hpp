
#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include <pthread.h>
#include <thread>
#include "imu.hpp"

#define UPDATE_INTERVAL_MS (500)
#define PWM_FULL_COUNT (4095)       // Count regiters are 12 bits in PCA 9685
#define PWM_FREQUENCY (333)         // 3 mS

// 4095 PWM counts for 3.333 mS -> 1365.3333 counts per mS
// 0.5 mS to 2.5 mS PWM pule width makes the servo go from 0 to 180 degrees 
// (2 mS * 1365.333 counts/ms) / 270 degrees/ms = 10.11 counts per degree

#define PWM_MIN (683)     // .5 mS
#define PWM_MAX (3413)    // 2.5 mS
#define PWM_RANGE (PWM_MAX - PWM_MIN)
#define PWM_MID (PWM_RANGE / 2)   

#define PWM_ANGLE_SCALE (10.11 / 16.0)  // 16 counts per degree from IMU
#define PWM_ANGLE_OFFSET (PWM_MID)

#define PWM_ACCEL_SCALE (10.0)         // Default, to be scaled dynamically
#define PWM_ACCEL_OFFSET (PWM_MIN)

class Display{
    public:
        Display();
        ~Display();
        void start();
        void stop();
        void printData(ImuData imu);
        int setPWMVals(unsigned int * nOnVals, unsigned int *nOffVals);
        inline bool isKeepRunning(){return m_bKeepRunning;}
    private:
        static void imuAngleToPwm(double angle, unsigned int *on, unsigned int *off, unsigned int phase = 0);
        static int updater(Display* pDisplay);

        std::thread m_tUpdater;
        int m_nFd;
        int m_nSlaveAddr;
        bool m_bKeepRunning;
};
#endif