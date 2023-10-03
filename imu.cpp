#include "imu.hpp"
#include <thread>
#include <chrono>
#include <math.h>
#include <termios.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "bno055_support.h"

using namespace std;

extern "C" struct bno055_t bno055;

atomic_flag flagKeepRunning;

Imu::Imu(int nBufferSize){
    m_nBufferSize =nBufferSize;
    m_pRoll = new Average(m_nBufferSize);
    m_pPitch = new Average(m_nBufferSize);
    m_pYaw = new Average(m_nBufferSize);
    m_pAccel = new Average(m_nBufferSize);
    signal(SIGINT, sigHandler);
}
Imu::~Imu(){
    delete m_pRoll;
    delete m_pPitch;
    delete m_pYaw;
    delete m_pAccel;
}
void Imu::sigHandler(int signum){
    flagKeepRunning.clear();
}
void Imu::runForever(){
    int fd;
    s8 stat;
    u8 mode;
    thread::native_handle_type threadHandles[2];

    // UART setup
    fd = BNO055_uart_init(B115200);
    if(fd > 0){
        cout << "Successfully intialized the UART" << endl;
    }else{
        cout << "Error: " << errno << "Initializing UART. Exiting Imu.runForever()" << endl;
        return;
    }

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

    // Start the IMU and display threads 
    flagKeepRunning.test_and_set();
    thread tImu(imuPoller, this);  
    tImu.detach();
    threadHandles[0] = tImu.native_handle();      
    thread tDisplay(udateDisplay, this);
    tDisplay.detach();
    threadHandles[1] = tDisplay.native_handle();

    // Main loop
    string line;
    cout << "Liftometer - 'h' for a list of commands" << endl; 
    while(flagKeepRunning.test_and_set()){   // Exit on SIGINT
        getline(cin, line);
        if(line.compare("q") == 0){
            cout << "Quit command recieved, exiting..." << endl;
            break;
        }else if(line.compare("h") == 0){
            cout << "Liftometer commands:" << endl;
            cout << "h - diplay this help message" << endl;
            cout << "r -reset the BNO055" << endl;
            cout << "m[mode] - read or set the operating mode of the BNO0055" << endl;
            cout << "q - quit liftometer" << endl;
        }else if(line.compare("r") == 0){
            cout << "Reset command recieved, resetting the BNO055..." << endl;
            bno055_set_sys_rst(1);
        }else if(line.find("m") == 0){
            u8 nMode;
            if(line.size() == 1){
                bno055_get_operation_mode(&nMode);
                cout << "BNO055 operation mode: " << +nMode << endl;
            }else{                int mod = stoi(line.substr(1));
                nMode = static_cast<u8>(mod) ;
                cout << "Changing mode to " << mod << endl;
                bno055_set_operation_mode(nMode);
            }
        }
        this_thread::yield();
    }

    // Clean up and exit
    cout << endl << "Killing  threads..." << endl;
    pthread_cancel(threadHandles[0]);
    pthread_cancel(threadHandles[1]);

exitPoint:
    close(fd);
}

int Imu::imuPoller(Imu* pImu){
    BNO055_RETURN_FUNCTION_TYPE ret1;
    BNO055_RETURN_FUNCTION_TYPE ret2;
    int result;
    cout << "IMU poller thread started" << endl;
    
    // result = bno055_set_sys_rst(1); 
    // result = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    // if(result != 0)
    // {
    //     cout << "Failed to set power mode. IMU thread exiting." << endl;
    //     return -1;

    // }
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
            chrono::steady_clock::now() + chrono::milliseconds(1000);    // 2 Hz    
        
        pImu->m_mtxData.try_lock_until(timePt);
        ret1 = bno055_read_euler_hrp(&hrp);  
        ret2 = bno055_read_linear_accel_xyz(&accel);  
        pImu->unlock();  

        if(ret1 == BNO055_SUCCESS){
            pImu->m_pRoll->add(hrp.r);
            pImu->m_pPitch->add(hrp.p);
            pImu->m_pYaw->add(hrp.h);      // Heading, 'h', is equivelant to yaw
            cout << "hrp: " << hrp.h << "," << hrp.r << ","<< hrp.p << endl;
        }else
            cout << "error reading hrp" << endl;
        
        // if(ret2 == BNO055_SUCCESS){
        //     // Multiplies of shorts to int. Cast to double for the sqrt function
        //     cout << "accel: " << accel.x << ","<< accel.y << ","<< accel.y << endl;
        //     double dAccel = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
        //     pImu->m_pAccel->add(dAccel);
        // }else
        //     cout << "error reading linear accel" << endl;
        
        this_thread::sleep_until(timePt);
    }
     return 0;
}
int Imu::udateDisplay(Imu* pImu){
    int result = 0;

    chrono::steady_clock::time_point timePt;
    cout << "update display thread started" << endl;
    while(1){

        timePt = chrono::steady_clock::now() + chrono::milliseconds(500);    // 2 Hz
        pImu->lock(timePt);
        // Access data here
        pImu->unlock();
        // Update display here
        this_thread::sleep_until(timePt);
    }
    return result;
}

