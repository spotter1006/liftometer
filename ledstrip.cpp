#include "ledstrip.hpp"
#include <gpiod.hpp>
#include <thread>
#include <math.h>
using namespace std;

Ledstrip::Ledstrip(int elements, int gpio){
    m_line =  chip.get_line(gpio); 
    m_line.request({"liftometer", gpiod::line_request::DIRECTION_OUTPUT, 0},0); 
    m_pData = new RGB[elements];
}

Ledstrip::~Ledstrip(){
    delete(m_pData);
}

void Ledstrip::send(){
    std::thread t1(sender, this);       // Serialize data in a detached thread
    t1.detach();
}

void Ledstrip::sender(Ledstrip* strip){
    // Data bit order: green red, blue - high bit first:
    // G7 G6 G5 G4 G3 G2 G1 G0 R7 R6 R5 R4 R3 R2 R1 R0 B7 B6 B5 B4 B3 B2 B1 B0 
 
    unsigned char mask = 0x80;
    for(int j = 0; j < strip->getSize(); j++){       
        RGB pixel = strip->getPixel(j);
        serialize(pixel.getGreen());
        serialize(pixel.getRed());
        serialize(pixel.getBlue());
    }
    m_line.set_value(0);    // Begin marking the frame reset. Must wait 50 uS before sending again to complete frame reset
}

void Ledstrip::serialize(char data){
    unsigned char mask = 0x80;
    for(int i=0; i < 8; i++){
        m_line.set_value(1);
        this_thread::sleep_for(std::chrono::nanoseconds((data & mask)? LONG_PERIOD_NS  : SHORT_PERIOD_NS));
        m_line.set_value(0);
        this_thread::sleep_for(std::chrono::nanoseconds((data & mask)? SHORT_PERIOD_NS : LONG_PERIOD_NS));
        mask = mask >> 1;
    }
}

void Ledstrip::bar(double mag, RGB color){
    double fractpart, intpart;
    fractpart = modf (mag , &intpart);
    
    RGB dark = RGB(0, 0, 0);      
    for(int i = 0; i < m_nElements; i ++){
        m_pData[i] = (i < intpart)? color : dark;
    }
    // The last bar will be dimmed to the magnitude of the frational part
    RGB fractionPixel = RGB(color);
    fractionPixel.dim(fractpart);
    m_pData[(int)intpart] = fractionPixel;
    send();
}