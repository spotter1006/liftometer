#include "imu.hpp"
#include <thread>
#include <chrono>
#include <math.h>
#include <termios.h>
#include <iostream>
#include "bno055_support.h"
using namespace std;
#include "liftometer.hpp"
extern "C" struct bno055_t bno055;
timed_mutex mtxData;
Imu::Imu(int nBufferSize){
    m_nBufferSize =nBufferSize;
    m_pRoll = new Average(m_nBufferSize);
    m_pPitch = new Average(m_nBufferSize);
    m_pYaw = new Average(m_nBufferSize);
    m_pAccel = new Average(m_nBufferSize);

}
Imu::~Imu(){
    pthread_cancel(m_tPoller.native_handle());
    delete m_pRoll;
    delete m_pPitch;
    delete m_pYaw;
    delete m_pAccel;
}

int Imu::init(){
    extern int fd;
    s8 stat;
    int nRet;

    fd = BNO055_uart_init(B115200);
    string message = (fd > 0)? "Successfully intialized the UART" : "Error initialing the UART";
    if(fd < 0)  return -1;

    bno055.dev_addr = fd;   
    bno055.bus_write = BNO055_uart_bus_write;
    bno055.bus_read = BNO055_uart_bus_read;
    bno055.delay_msec = BNO055_delay_msek;

    stat = bno055_init(&bno055);    
    if(stat == 0){
        cout << "Successfully intialized the BNO055" << endl;
    }else{
        cout << "Error initializing BNO05. " << endl;
    }

    stat = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    if(stat == 0){
        cout << "Successfully set BNO055 NDOF mode" << endl;
    }else{
        cout << "Failed to se BNO05 NDOF mode. " << endl;
    }

    thread m_tPoller(imuPoller, this);  
    m_tPoller.detach();
    return nRet;
}

int Imu::imuPoller(Imu* pImu){
    BNO055_RETURN_FUNCTION_TYPE ret;
    int nPingPong = 0;
    int result;
    cout << "IMU poller thread started" << endl;
    
    u8 currentMode;
    result = bno055_get_operation_mode(&currentMode);
    cout << "The current operation mode is " << static_cast<int> (currentMode) << endl;
    result = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF); 
    if(result == 0){
        cout << "Successfully set operation mode to NDOF" << endl;
    }else{
        cout << "Failed to set operation mode mode. IMU poller thread exiting." << endl;
        return -1;
    }
    while(1){      
        bno055_euler_t hrp; // heading, roll, pitch
        bno055_linear_accel_t accel;    // Linear acceleration (gravity removed)

        // Calculate interval for the next wake up
        chrono::_V2::steady_clock::time_point timePt = 
            chrono::steady_clock::now() + chrono::milliseconds(500);      
        
        mtxData.lock();
        result = BNO055_read_combined_data(&hrp, &accel);
        mtxData.unlock();
        
        if(result ==0){
            pImu->m_pRoll->add(hrp.r);
            pImu->m_pPitch->add(hrp.p);
            pImu->m_pYaw->add(hrp.h);      
            double dAccel = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
            pImu->m_pAccel->add(dAccel);
        }
 
        
        this_thread::sleep_until(timePt);
    }
    return 0;
}
double Imu::getAverageAccel(int nSamples){
    return m_pAccel->calc( nSamples);
}
double Imu::getAverageRoll(int nSamples){
    return m_pRoll->calc(nSamples);
}
double Imu::getAveragePitch(int nSamples){
    return m_pPitch->calc(nSamples);
}
double Imu::getAverageYaw(int nSamples){
    return m_pYaw->calc(nSamples);
}
d