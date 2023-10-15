
#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include <pthread.h>
#include <thread>
#include "imu.hpp"

#define UPDATE_INTERVAL_MS (400)
#define PWM_FULL_COUNT (4095)       // Count regiters are 12 bits in PCA 9685
#define PWM_FREQUENCY (200)         // 5 mS

// 1 ms sets 90 degrees on servo
// 4095 PWM counts for 5 mS means 819 PWM counts per mS
// 819 counts/ms / 90 degrees /ms = 9.1 counts per degree
// 90 degrees * 9.1 counts / degree = 819 counts
#define PWM_ANGLE_SCALE (9.1)    
#define PWM_ANGLE_OFFSET (819)

#define PWM_MIN (410)         
#define PWM_MAX (2048) 

#define PWM_ACCEL_SCALE (10.0)         // Default, to be scaled dynamically
#define PWM_ACCEL_OFFSET (PWM_MIN)

class Display{
    public:
        Display();
        ~Display();
        int start();
        int setPWMVals(unsigned int * nOnVals, unsigned int *nOffVals);
    private:
        static void imuAngleToPwm(double angle, unsigned int *on, unsigned int *off);
        static void imuAccelToPwm(double accel, unsigned int *on, unsigned int *off);
        static int updater(Display* pDisplay);


        std::thread m_tUpdater;
        int m_nFd;
        int m_nSlaveAddr;
};
#endif