#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include <pthread.h>
#include <thread>
#include "imu.hpp"


/********************************************************************
 * Miuzei MS18 9G servo with PCA 2685 PWM chip
 * 50 hz = 20 mS
 * 4095 counts / 20 mS = 204.8 counts per mS
 * Min: 0.5ms = 0 degrees 
 * Max: 2.5 mS = 180 degrees 
 ********************************************************************/
#define UPDATE_INTERVAL_MS (400)
#define PWM_FULL_COUNT (4095)       // Count regiters are 12 bits in PCA 9685
#define PWM_FREQUENCY (200)
#define PWM_MS_RANGE (2.0)
#define PWM_ANGLE_RANGE (180.0)

#define PWM_COUNTS_PER_MS (PWM_FULL_COUNT / PWM_FREQUENCY)
#define PWM_MS_PER_DEGREE (PWM_MS_RANGE / PWM_ANGLE_RANGE)
#define IMU_COUNTS_PER_DEGREE (16.0)

#define PWM_ANGLE_SCALE (PWM_COUNTS_PER_MS * PWM_MS_PER_DEGREE / IMU_COUNTS_PER_DEGREE )    
#define PWM_ANGLE_OFFSET (90.0 * PWM_MS_PER_DEGREE)

#define PWM_MIN (0.5 * PWM_COUNTS_PER_MS)         
#define PWM_MAX (2.5 * PWM_COUNTS_PER_MS) 

#define PWM_ACCEL_SCALE (100.0)         // Default, to be scaled dynamically
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