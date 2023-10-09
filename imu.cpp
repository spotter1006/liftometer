#include "imu.hpp"
#include <thread>
#include <chrono>
#include <math.h>
#include <termios.h>
#include <iostream>
#include <gpiod.hpp>
#include <unistd.h>
#include "bno055_support.h"
#include "liftometer.hpp"

using namespace std;

extern "C" struct bno055_t bno055;
timed_mutex mtxData;

Imu::Imu(int nBufferSize){
    m_nBufferSize =nBufferSize;
    m_pRoll = new Average(m_nBufferSize);
    m_pPitch = new Average(m_nBufferSize);
    m_pYawRateX = new Average(m_nBufferSize);
    m_pYawRateY = new Average(m_nBufferSize);
    m_pAccelX = new Average(m_nBufferSize);
    m_pAccelY = new Average(m_nBufferSize);
}
Imu::~Imu(){
    pthread_cancel(m_tPoller.native_handle());
    delete m_pRoll;
    delete m_pPitch;
    delete m_pYawRateX;
    delete m_pYawRateY;
    delete m_pAccelX;
    delete m_pAccelY;
}

int Imu::start(){
    extern int fd;
    s8 stat;
    int nRet;

    // Reset the BNO055
    gpiod::chip chip("gpiochip0");
    auto line = chip.get_line(18);  
    line.request({"liftometer", gpiod::line_request::DIRECTION_OUTPUT, 0},0);  
    usleep(50);
    line.set_value(1);
    line.release();

    sleep(1);       // Wait for the chip to come back
  
    fd = BNO055_uart_init(B115200);
    string message = (fd > 0)? "Successfully intialized the UART" : "Error initialing the UART";
    if(fd < 0)  return -1;

    bno055.dev_addr = fd;   
    bno055.bus_write = BNO055_uart_bus_write;
    bno055.bus_read = BNO055_uart_bus_read;
    bno055.delay_msec = BNO055_delay_msek;

    stat = bno055_init(&bno055);    
   
    stat = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    if(stat == 0){
        cout << "Successfully set BNO055 NDOF mode" << endl;
        sleep(.02);
    }else{
        cout << "Failed to set BNO05 NDOF mode. " << endl;
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

    result = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF); 
    this_thread::sleep_for(chrono::milliseconds(20));
    if(result == 0){
        cout << "Successfully set operation mode to NDOF" << endl;
    }else{
        cout << "Failed to set operation mode mode. IMU poller thread exiting." << endl;
        return -1;
    }
    while(1){   
        bno055_gyro_t gyro;   
        bno055_euler_t hrp; // heading, roll, pitch
        bno055_linear_accel_t accel;    // Linear acceleration (gravity removed)

        // Calculate interval for the next wake up
        chrono::_V2::steady_clock::time_point timePt = 
            chrono::steady_clock::now() + chrono::milliseconds(20);      // 50 hz
        
        mtxData.lock();
        result = BNO055_read_combined_data(&gyro, &hrp, &accel);
        mtxData.unlock();
        
        if(result ==0){
            pImu->m_pRoll->add(hrp.r);
            pImu->m_pPitch->add(hrp.p);
            pImu->m_pYawRateX->add(gyro.x);
            pImu->m_pYawRateY->add(gyro.y);
            pImu->m_pAccelX->add(accel.x);
            pImu->m_pAccelY->add(accel.y);       
        }
 
        this_thread::sleep_until(timePt);
    }
    return 0;
}
double Imu::getAverageAccelX(int nSamples){
    return m_pAccelX->calc( nSamples);
}
double Imu::getAverageAccelY(int nSamples){
    return m_pAccelY->calc( nSamples);
}
double Imu::getAverageRoll(int nSamples){
    return m_pRoll->calc(nSamples);
}
double Imu::getAveragePitch(int nSamples){
    return m_pPitch->calc(nSamples);
}
double Imu::getAverageYawRateX(int nSamples){
    return m_pYawRateX->calc(nSamples);
}
double Imu::getAverageYawRateY(int nSamples){
    return m_pYawRateY->calc(nSamples);
}

