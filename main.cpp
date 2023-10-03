#include <iostream>
#include "imu.hpp"
#include <signal.h>
#include <unistd.h>
#define BUFFER_SIZE 1024
using namespace std;

atomic_flag flagKeepRunning;
extern thread::native_handle_type threadHandles[2];
void sigHandler(int signum){
    flagKeepRunning.clear();
}

int main(){
    extern int fd;
    cout << "Starting liftometer with " << BUFFER_SIZE << " element buffers" << endl;
    int result =0;
    
    flagKeepRunning.test_and_set();
    signal(SIGINT, sigHandler);
    Imu* imu = new Imu(BUFFER_SIZE);
    imu->init();
  
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
    delete imu;
    close(fd);
    cout << "Exiting" << endl;
    return result;
}