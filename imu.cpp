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
extern gpiod::chip chip;
timed_mutex mtxData;

Imu::Imu(int nBufferSize){
    m_bKeepRunning = true;
    m_nBufferSize =nBufferSize;
    m_pData = new list<ImuData>();
}
Imu::~Imu(){
    m_pData->clear();
    delete m_pData;
}
void Imu::add(ImuData dataPoint){
    m_pData->push_front(dataPoint);
    if(m_pData->size() > DATA_SIZE){
        m_pData->resize(DATA_SIZE);
    }
}
int Imu::start(){
    extern int fd;
    s8 stat;
    int nRet;

    m_bKeepRunning = true;
    // Reset the BNO055

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
        sleep(.02);
    }else{
        cout << "Failed to set BNO05 NDOF mode. " << endl;
    }

    thread t1(imuPoller, this);  
    t1.detach();
    return nRet;
}
void Imu::stop(){
    m_bKeepRunning = false;
}
void Imu::imuPoller(Imu* pImu){
    BNO055_RETURN_FUNCTION_TYPE ret;
    int result;

    result = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF); 
    this_thread::sleep_for(chrono::milliseconds(20));
    if(result != 0){
        cout << "Failed to set operation mode mode. IMU poller thread exiting." << endl;
        return;
    }
    bno055_gyro_t gyro;   
    bno055_euler_t hrp; // heading, roll, pitch
    bno055_linear_accel_t accel;    // Linear acceleration (gravity removed)
    while(pImu->isKeepRunning()){   

        // Calculate interval for the next wake up
        chrono::_V2::steady_clock::time_point timePt = 
            chrono::steady_clock::now() + chrono::milliseconds(SAMPLE_RATE_MS);  
        
        mtxData.lock();
        result = BNO055_read_combined_data(&gyro, &hrp, &accel);
        mtxData.unlock();
        
        if(result ==0){
            ImuData dataPoint;
            dataPoint.roll = hrp.r;
            dataPoint.pitch = hrp.p;
            dataPoint.yawRateX = gyro.x;
            dataPoint.yawRateY = gyro.y;
            dataPoint.accX = accel.x;
            dataPoint.accY = accel.y;
            pImu->add(dataPoint);    
        }
 
        this_thread::sleep_until(timePt);
    }
}
ImuData sum(ImuAveragedData a, ImuAveragedData b){ 
    ImuData result;
    result.accX = a.accX + b.accX;
    return result;
}

ImuAveragedData Imu::getAveragedData(int nSamples){
    ImuAveragedData result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int size = m_pData->size();
    if(size > 0){
        int n = (nSamples < size)? m_pData->size() : nSamples;
        auto it = m_pData->begin();
        for(int i = 0; i < nSamples; i++){
            ImuData dataPoint = *it;
            result.accX += dataPoint.accX;
            result.accY += dataPoint.accY;
            result.roll += dataPoint.roll;
            result.pitch += dataPoint.pitch;
            result.yawRateX += dataPoint.yawRateX;
            result.yawRateY += dataPoint.yawRateY;
            advance(it,1);
        }
        result.accX /= nSamples;
        result.accY /= nSamples;
        result.roll /= nSamples;
        result.pitch /= nSamples;
        result.yawRateX /= nSamples;
        result.yawRateY /= nSamples;
    }
    return result;
}
