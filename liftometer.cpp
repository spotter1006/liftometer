#include <iostream>
#include "imu.hpp"
#include <signal.h>
#include <unistd.h>
#include "display.hpp"
#define BUFFER_SIZE 1024
using namespace std;

atomic_flag flagKeepRunning;
void sigHandler(int signum){
    flagKeepRunning.clear();
}

int main(){
    extern int fd;
    cout << "Starting liftometer with " << BUFFER_SIZE << " element buffers" << endl;
    int result =0;
    int nSamples =32;

    flagKeepRunning.test_and_set();
    signal(SIGINT, sigHandler);

    Imu* imu = new Imu(BUFFER_SIZE);
    imu->start();
    Display* display = new Display();
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
        }else if(line.compare("s") == 0){
            nSamples = stoi(line.substr(1));
            display->setSampleSize(nSamples);
            cout << "Set the number of sample to average over to " << nSamples << endl;
        }else if(line.compare("d") == 0){
            cout << "Acceleration: " << imu->getAverageAccel(nSamples) << endl;
            cout << "Heading: " << imu->getAverageYaw(nSamples) << endl;
            cout << "Roll: " << imu->getAverageRoll(nSamples) << endl;
            cout << "Pich: " << imu->getAveragePitch(nSamples) << endl;
        }
        this_thread::yield();
    }
    // Clean up and exit
    cout << endl << "Killing  threads..." << endl;
    delete display;
    delete imu;
    close(fd);
    return result;
}