#include <iostream>
#include "imu.hpp"
#include <signal.h>
#include <unistd.h>
#include "display.hpp"
#define BUFFER_SIZE 1000000
using namespace std;

atomic_flag flagKeepRunning;
Imu* pImu;
extern int nSampleSize;

void sigHandler(int signum){
    flagKeepRunning.clear();
}

int main(){
    extern int fd;
    cout << "Starting liftometer with " << BUFFER_SIZE << " element buffers" << endl;
    int result =0;

    flagKeepRunning.test_and_set();
    signal(SIGINT, sigHandler);

    pImu = new Imu(BUFFER_SIZE);
    Display* display = new Display();
    pImu->start();
    display->start();

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
            cout << "s<integer> - set th number of samples to use for the averages" << endl;
            cout << "d - display the average data" << endl;
            cout << "q - quit liftometer" << endl;
        }else if(line.compare("r") == 0){
            cout << "Reset command recieved, resetting the BNO055..." << endl;
            bno055_set_sys_rst(1);
        }else if(line.find("s") == 0){
            nSampleSize = stoi(line.substr(1));
            cout << "Set the number of sample to average over to " << nSampleSize << endl;
        }else if(line.compare("d") == 0){
            cout << "Differential IMU readings over " << nSampleSize << " samples" << endl;         
            cout << "Acceleration: " << pImu->getAverageAccel(nSampleSize);
            cout << ", Heading: " << pImu->getAverageYawRate(nSampleSize);
            cout << ", Roll: " << pImu->getAverageRoll(nSampleSize);
            cout << ", Pitch: " << pImu->getAveragePitch(nSampleSize) << endl;
        }
        this_thread::yield();
    }
    // Clean up and exit
    cout << endl << "Killing  threads..." << endl;
    delete display;
    delete pImu;
    close(fd);
    return result;
}