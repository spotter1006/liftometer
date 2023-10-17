
#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include <pthread.h>
#include <thread>
#include "imu.hpp"

#define UPDATE_INTERVAL_MS (10)
#define PWM_FULL_COUNT (4095)       // Count regiters are 12 bits in PCA 9685
#define PWM_FREQUENCY (200)         // 5 mS

// 4095 PWM counts for 5 mS -> 819 counts per mS
// 0.5 mS to 2.5 mS PWM pule width makes the servo go from 0 to 180 degrees
// (1.5 mS * 819 counts/ms) / (90 degrees/ms) = 13.65 counts per degree

#define PWM_ANGLE_SCALE (13.65)    
#define PWM_ANGLE_OFFSET (1024)

#define PWM_MIN (410)         
#define PWM_MAX (2048) 

#define PWM_ACCEL_SCALE (100.0)         // Default, to be scaled dynamically
#define PWM_ACCEL_OFFSET (PWM_MIN)

class Display{
    public:
        Display();
        ~Display();
        void start();
        void stop();
        int setPWMVals(unsigned int * nOnVals, unsigned int *nOffVals);
        inline bool isKeepRunning(){return m_bKeepRunning;}
    private:
        static void imuAngleToPwm(double angle, unsigned int *on, unsigned int *off);
        static void imuAccelToPwm(double accel, unsigned int *on, unsigned int *off);
        static int updater(Display* pDisplay);

        std::thread m_tUpdater;
        int m_nFd;
        int m_nSlaveAddr;
        bool m_bKeepRunning;
};
#endif