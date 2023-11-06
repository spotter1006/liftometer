#include <iostream>
#include "imu.hpp"
#include <signal.h>
#include <unistd.h>
#include <cmath>
#include "display.hpp"
#include "encoder.hpp"
#include "imu.hpp"

#define BUFFER_SIZE 10000000    // 10 million samples 
using namespace std;


// Static global variables
atomic_flag flagKeepRunning;
gpiod::chip chip("gpiochip0");
Imu* pImu;
Display *pDisplay;
Encoder *pEncoder;
extern int nSampleSize;     
extern unsigned int nOnVals[16];
extern unsigned int nOffVals[16];

void sigHandler(int signum){
    flagKeepRunning.clear();
}

int main(){
    extern int fd;
    int result =0;

    flagKeepRunning.test_and_set();
    signal(SIGINT, sigHandler);

    pImu = new Imu(BUFFER_SIZE);
    pDisplay = new Display();
    pEncoder = new Encoder();

    pImu->start();
    pDisplay->start();
    pEncoder->start();

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
            cout << "r - reset the BNO055" << endl;
            cout << "q - quit liftometer" << endl;
        }else if(line.compare("r") == 0){
            cout << "Reset command recieved, resetting the BNO055..." << endl;
            bno055_set_sys_rst(1);
        }
    }
    // Clean up and exit
    pDisplay->stop();
    pEncoder->stop();
    pImu->stop();
    delete pDisplay;
    delete pEncoder;
    delete pImu;
    close(fd);
    return result;
}