#include "imu.hpp"
#include <thread>
#include <chrono>
#include <math.h>
#include <termios.h>
#include <iostream>
#include <gpiod.hpp>
#include <unistd.h>
#include <cstring>
#include "bno055_support.h"
#include "liftometer.hpp"

using namespace std;

extern "C" struct bno055_t bno055;
extern gpiod::chip chip;


Imu::Imu(int nBufferSize){
    m_bKeepRunning = true;
    m_nBufferSize =nBufferSize;
    m_pData = new list<ImuData>();

    m_pAverages[0] = new Average( 5 * SAMPLES_PER_SECOND);
    m_pAverages[1] = new Average( 10 * SAMPLES_PER_SECOND);
    m_pAverages[2] = new Average( 20 * SAMPLES_PER_SECOND);
    m_pAverages[3] = new Average( 40 * SAMPLES_PER_SECOND);
    // m_pAverages[4] = new Average( 80 * SAMPLES_PER_SECOND);
    // m_pAverages[5] = new Average( 160 * SAMPLES_PER_SECOND);
    // m_pAverages[6] = new Average( 320 * SAMPLES_PER_SECOND);
    // m_pAverages[7] = new Average( 640 * SAMPLES_PER_SECOND);

}
Imu::~Imu(){
    m_pData->clear();
    delete m_pData;
}
// Thread safe
void Imu::add(ImuData dataPoint){
    m_mtxData.lock();
    m_pData->push_front(dataPoint);
    if(m_pData->size() > DATA_SIZE){
        m_pData->resize(DATA_SIZE);
    }
    m_mtxData.unlock();
    for(int i = 0; i < BUCKETS; i++){
        m_pAverages[i]->add(dataPoint.heading);
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
    thread t1(imuPoller, this);  
    t1.detach();
    return nRet;
}
void Imu::stop(){
    m_bKeepRunning = false;
}
void Imu::imuPoller(Imu* pImu){
    BNO055_RETURN_FUNCTION_TYPE ret;
    
    int result = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF); 
    this_thread::sleep_for(chrono::milliseconds(20));
    if(result != 0){
        cout << "Failed to set operation mode mode. IMU poller thread exiting." << endl;
        return;
    }
    bno055_gyro_t gyro;   
    bno055_euler_t hrp; // heading, roll, pitch
    bno055_linear_accel_t accel;    // Linear acceleration (gravity removed)
    ImuData dataPoint;
    while(pImu->isKeepRunning()){  
        
        result = BNO055_read_combined_data(&gyro, &hrp, &accel);
        if(result ==0){
            dataPoint.roll = hrp.r;
            dataPoint.pitch = hrp.p;
            dataPoint.heading = hrp.h;
            dataPoint.gyroX = gyro.x;
            dataPoint.gyroY = gyro.y;
            dataPoint.accX = accel.x;
            dataPoint.accY = accel.y;
            pImu->add(dataPoint);    
        }

        this_thread::sleep_for(chrono::milliseconds(SAMPLE_RATE_MS));
    }
}

void Imu::getLatestData(ImuData *pData){
    m_mtxData.lock();
    list<ImuData>::iterator latest = m_pData->begin();
    pData->heading=latest->heading;
    pData->roll=latest->roll;
    pData->pitch=latest->pitch;
    pData->gyroX=latest->gyroX;
    pData->gyroY=latest->gyroY;
    pData->accX=latest->accX;
    pData->accY=latest->accY;
    m_mtxData.unlock();
}
int Imu::getAverageHeading(int i){
    return m_pAverages[i]->getAverage();
}
int Imu::getOldHeading(int samplesAgo){
    ImuData result;
    m_mtxData.lock();
    list<ImuData>::iterator it = m_pData->begin();
    advance(it, samplesAgo);
    result = *it;
    m_mtxData.unlock();
    return result.heading;
}

