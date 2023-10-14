#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include <pthread.h>
#include <thread>
#include "imu.hpp"

/* Miuzei 9g servos: ******************************************************
*                   THIS IS WRONG
*   0        90      120 +/- 10      mechanical (degrees)
*   900      1500    2100            high pulse width (uS)
*   450      750     1050            counts in the PCA2865 "on" register
* 
*   2100 uS period = 476.19 Hz frequency  
*   4096 counts per PWM cycle -> 31.5 to 37.24 counts per degree 
*     middle: 34 counts per degree
*   1 count is 2 uS       
**************************************************************************/  

/********************************************************************
 * 50 hz = 20 mS
 * 4095 counts / 20 mS = 204.8 counts per mS
 * Min: 1ms = 0 degrees = 204.8 counts
 * Max: 2 mS = 180 degrees = 409.6 counts
 ********************************************************************/
#define UPDATE_INTERVAL_MS (100)
#define PWM_FULL_COUNT (4095)       // Count regiters are 12 bits in PCA 9685
#define PWM_FREQUENCY (50)

#define PWM_COUNTS_PER_MS (204.8)
#define PWM_MS_PER_DEGREE (1.0 / 180.0)
#define IMU_COUNTS_PER_DEGREE (16.0)

#define PWM_ANGLE_SCALE (PWM_COUNTS_PER_MS * PWM_MS_PER_DEGREE/ IMU_COUNTS_PER_DEGREE )    
#define PWM_ANGLE_OFFSET (90.0 * PWM_MS_PER_DEGREE)

#define PWM_MIN (1 * PWM_COUNTS_PER_MS)         
#define PWM_MAX (2 * PWM_COUNTS_PER_MS) 

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