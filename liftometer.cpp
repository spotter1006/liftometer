#include <iostream>
#include "average.hpp"
#include <iostream>
#include <atomic>
#include "imu.hpp"

#define BUFFER_SIZE 1024

using namespace std;

int main(){
    cout << "Starting liftometer with " << BUFFER_SIZE << " element buffers" << endl;
    int result =0;
    Imu* imu = new Imu(BUFFER_SIZE);
    imu->runForever();
    delete imu;
    cout << "All threads shut down. Exiting." << endl;
    return result;
}