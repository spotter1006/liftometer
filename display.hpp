#ifndef _DISPLAY_H_
#define _DISPLAY_H_
#include <pthread.h>
#include <thread>
#include "imu.hpp"

#define PWM_FULL_COUNT (4095)
#define IMU_PWM_ANGLE_SCALE (34.0 /16.0)    // 16 degrees per LSB, 34 counts per degee
#define IMU_PWM_ANGLE_OFFSET (750)
#define IMU_PWM_ACCEL_SCALE (100.0)         // Default, to be scaled dynamically
#define IMU_PWM_ACCEL_OFFSET (450)
#define PWM_MIN (450)
#define PWM_MAX (1050)

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