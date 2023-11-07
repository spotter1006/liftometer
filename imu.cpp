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
    memset(m_headingSums, 0, sizeof(accumulator) * 8);
    m_headingSums[0].size = 5 * SAMPLES_PER_SECOND; 
    m_headingSums[1].size = 10 * SAMPLES_PER_SECOND; 
    m_headingSums[2].size = 20 * SAMPLES_PER_SECOND; 
    m_headingSums[3].size = 40 * SAMPLES_PER_SECOND; 
    m_headingSums[4].size = 80 * SAMPLES_PER_SECOND;
    m_headingSums[5].size = 160 * SAMPLES_PER_SECOND; 
    m_headingSums[6].size = 320 * SAMPLES_PER_SECOND; 
    m_headingSums[7].size = 640 * SAMPLES_PER_SECOND;  // 10 minutes, 40 seconds
}
Imu::~Imu(){
    m_pData->clear();
    delete m_pData;
}
void Imu::add(ImuData dataPoint){
    lock();               // This is the longest lock
    m_pData->push_front(dataPoint);

    // Update rolling avereage sums
    for(int i = 0; i < 8; i++){
        m_headingSums[i].sum += dataPoint.heading;
        if(m_headingSums[i].nSamples >= m_headingSums[i].size){
            m_headingSums[i].sum -= m_pData->end()->heading;
        }else{
            m_headingSums[i].nSamples++;
        }
    }
    // TODO: serialize the data to a file on the flash disk and start a new set
    // For now: resize the list, dropping the oldest data
    if(m_pData->size() > DATA_SIZE){
        m_pData->resize(DATA_SIZE);
    }
    unlock();
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
    ImuData dataPoint;
    while(pImu->isKeepRunning()){   

        // Calculate interval for the next wake up
        chrono::_V2::steady_clock::time_point timePt = 
            chrono::steady_clock::now() + chrono::milliseconds(SAMPLE_RATE_MS);        

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
        this_thread::sleep_until(timePt);
    }
}
void Imu::getData(ImuData *pData){
    lock();
    ImuData latest = *(m_pData->begin());
    pData->heading=latest.heading;
    pData->roll=latest.roll;
    pData->pitch=latest.pitch;
    pData->gyroX=latest.gyroX;
    pData->gyroY=latest.gyroY;
    pData->accX=latest.accX;
    pData->accY=latest.accY;
    unlock();
}

int Imu::getAverageHeading(int index){
    lock();
    return m_headingSums[index].sum / m_headingSums[index].nSamples;
    unlock();
}
int Imu::getHeadingAverageSamples(int index){
    return m_headingSums[index].nSamples;   // Read only, does not need mutex lock
}