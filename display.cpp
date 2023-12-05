#include "display.hpp"
#include "encoder.hpp"
#include <chrono>
#include <iostream>
#include "liftometer.hpp"
#include "PCA9685.h"
#include <cstring>
#include <math.h>
#include <libu8g2arm/U8g2lib.h>
#include <libu8g2arm/u8g2arm.h>
#include <cstdio>
using namespace std;

extern Encoder *pEncoder;
extern Imu* pImu;
int nSampleSize;
unsigned int nOnVals[16];
unsigned int nOffVals[16];
U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R0, 22, 27, 17); // Rotation, CS, DC, RST

Display::Display(){
    nSampleSize = 100;
    m_nSlaveAddr = 0x40;
    m_bKeepRunning = true;
    //_PCA9685_DEBUG = 1; // uncomment to show PCA9685 debug info
    m_nFd = PCA9685_openI2C(1, 0x20);
    int nResult = PCA9685_initPWM(m_nFd, m_nSlaveAddr, PWM_FREQUENCY);

    if (!u8g2arm_arm_init_hw_spi(u8g2.getU8x8(), 0, 0, 1)  ){ // u8g2 struct, [device name: /dev/spidev0.0], speed (Mhz)
        fprintf(stderr, "could not initialise SPI device");
    }

    u8g2.begin();
    u8g2.clearBuffer();                     // clear the internal memory
    u8g2.sendBuffer();                      // transfer internal memory to the display
    u8g2.setFont(u8g2_font_ncenB08_tr);     // choose a suitable font
}
Display::~Display(){
    PCA9685_setAllPWM(m_nFd, m_nSlaveAddr, _PCA9685_MINVAL, _PCA9685_MINVAL);

}

int Display::updater(Display* pDisplay){
    memset(nOnVals,0,16);
    memset(nOffVals,0,16);

    int nResult = 0;
    chrono::steady_clock::time_point timePt;

    while(pDisplay->isKeepRunning()){
        timePt = chrono::steady_clock::now() + chrono::milliseconds(UPDATE_INTERVAL_MS);
        ImuData latest;
        nSampleSize = pImu->getHeadingAverageSamples(pEncoder->getCount());
      
        pImu->getLatestData(&latest);
        
        imuAngleToPwm(latest.roll, &nOnVals[0], &nOffVals[0]);       
        imuAngleToPwm(latest.pitch, &nOnVals[1], &nOffVals[1]);      
        imuAngleToPwm(latest.heading, &nOnVals[2], &nOffVals[2]);    
        // imuAngleToPwm(average.heading, &nOnVals[3], &nOffVals[3]);    
        
        /************* TODO:***************************
            - gyro.z for yaw rate (shows how the boat is sliding)
            - accel.x,y for performance indicator 
        *********************************************************/

        pDisplay->setPWMVals(nOnVals, nOffVals);

        int nTotalMs = nSampleSize * (SAMPLE_RATE_MS); 
        int nMinutes = nTotalMs / 60000;
        int nSeconds = (nTotalMs / 1000) % 60;

        pDisplay->printData(latest);

        this_thread::sleep_until(timePt);
    }
    return nResult;
}
int Display::setPWMVals(unsigned int* nOnVals, unsigned int* nOffVals){
    return PCA9685_setPWMVals(m_nFd,m_nSlaveAddr,nOnVals, nOffVals);
}
void Display::start(){
    m_bKeepRunning = true;
    thread m_tUpdater(updater, this);
    m_tUpdater.detach();
}
void Display::stop(){
    m_bKeepRunning = false;  
}

void Display::imuAngleToPwm(double angle, unsigned int *on, unsigned int *off, unsigned int phase){
    int nPulseWidth =  PWM_ANGLE_OFFSET + (angle * PWM_ANGLE_SCALE);
    if (nPulseWidth < PWM_MIN)  nPulseWidth = PWM_MIN;
    else if(nPulseWidth > PWM_RANGE) nPulseWidth = PWM_RANGE;

    *on = phase;            // Phase shift
    *off = PWM_MIN + phase + nPulseWidth - 1;
} 

void Display:: printData(ImuData imu){
    char characterBuff[128];
    u8g2.clearBuffer();                     
    u8g2.sendBuffer();         

    sprintf(characterBuff, "Accel: %d, %d", imu.accX, imu.accY);
    u8g2.drawStr(0, 10, characterBuff);   
    sprintf(characterBuff, "HRP: %d, %d, %d", imu.heading, imu.pitch, imu.roll);
    u8g2.drawStr(0, 20, characterBuff); 
    sprintf(characterBuff, "Gyro: %d, %d", imu.gyroX, imu.gyroY);  
    u8g2.drawStr(0, 30, characterBuff); 
    
    sprintf(characterBuff, "Average Headings:");
    u8g2.drawStr(0, 40, characterBuff); 

    double av5, av640;
    pImu->getAverageHeading(1, & av5);
    pImu->getAverageHeading(7, & av640);
    sprintf(characterBuff, "5s: %3.1f, 640s: %3.1f", av5, av640);
    u8g2.drawStr(0, 50, characterBuff); 
    
    u8g2.sendBuffer(); 
                 

}
