#include "encoder.hpp"
#include <math.h> 
extern gpiod::chip chip;
/******************************************************************************
   Quadrature encoder makes two waveforms that are 90° out of phase:
                           _______         _______         __
                  PinA ___|       |_______|       |_______|   PinA
          CCW <--              _______         _______
                  PinB _______|       |_______|       |______ PinB

                               _______         _______
                  PinA _______|       |_______|       |______ PinA
          CW  -->          _______         _______         __
                  PinB ___|       |_______|       |_______|   PinB


          The half of the pulses from top to bottom create full state array:  

          prev.A+B   cur.A+B   (prev.AB+cur.AB)  Array   Encoder State
          -------   ---------   --------------   -----   -------------
            00         00            0000          0     stop/idle
            00         01            0001          1     CW,  0x01
            00         10            0010         -1     CCW, 0x02
            00         11            0011          0     invalid state
            01         00            0100         -1     CCW, 0x04
            01         01            0101          0     stop/idle
            01         10            0110          0     invalid state
            01         11            0111          1     CW, 0x07
            10         00            1000          1     CW, 0x08
            10         01            1001          0     invalid state
            10         10            1010          0     stop/idle
            10         11            1011         -1     CCW, 0x0B
            11         00            1100          0     invalid state
            11         01            1101         -1     CCW, 0x0D 
            11         10            1110          1     CW,  0x0E
            11         11            1111          0     stop/idle

          - CW  states 0b0001, 0b0111, 0b1000, 0b1110
          - CCW states 0b0010, 0b0100, 0b1011, 0b1101
******************************************************************************/
int states[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

Encoder::Encoder(){
    m_bKeepRunning = true;
    m_nValA = 0;
    m_nValB = 0;
    m_nSwitchVal =1;
    m_nCount = 0;
}

int Encoder::start(){
    m_bKeepRunning = true;
    std::thread t1(poller, this);
    t1.detach();
    return(0);
}
void Encoder::stop(){
    m_bKeepRunning = false;
}
void Encoder::add(int n){
    m_nCount += n;
    if(m_nCount >= 15) 
        m_nCount =0;
    else if(m_nCount < 0) m_nCount = 15;
}

/*
    poller thread
    uses a state machine to count the edges on the encoder A and B lines
*/
void Encoder::poller(Encoder* pEncoder){
    gpiod::line lineA = chip.get_line(ENCODER_LINE_A); 
    gpiod::line lineB = chip.get_line(ENCODER_LINE_B);
    gpiod::line lineSwitch = chip.get_line(SWITCH_LINE);
    lineA.request({"liftometer", gpiod::line_request::EVENT_BOTH_EDGES, 0},0);  
    lineB.request({"liftometer", gpiod::line_request::EVENT_BOTH_EDGES, 0},0);
    
    // TODO: libgpiod V1.6 has the version that allows setting the pullup. The highest version available in buster is libgpio 1.2
    // lineSwitch.request({"liftometer", gpiod::line_request::EVENT_BOTH_EDGES, gpiod::line_request::FLAG_BIAS_PULL_UP},0);   
    lineSwitch.request({"liftometer", gpiod::line_request::EVENT_BOTH_EDGES, 0},0);
    
    gpiod::line_event event;
    auto timeout = chrono::milliseconds(1);
    vector<gpiod::line> lines = {lineA, lineB, lineSwitch};
    gpiod::line_bulk lineBulk = gpiod::line_bulk(lines);
    while(pEncoder->isKeepRunning()){
        int nEvents = 0;     
        gpiod::line_bulk eventLines = lineBulk.event_wait(timeout);
        for(gpiod::line line : eventLines){
            if(lineA == line){
                event = lineA.event_read();
                nEvents++;
            }
            if( lineB == line){
                event = lineB.event_read();
                nEvents++;
            }
            if(lineSwitch == line){
                event = lineSwitch.event_read();
                pEncoder->setSwitchVal(lineSwitch.get_value());
            }
        }
        if(nEvents > 0){
            int nValA = lineA.get_value();
            int nValB = lineB.get_value();
            int nState =  pEncoder->getValA() << 3 | pEncoder->getValB() << 2 | nValA << 1 | nValB;
            int nCount = states[nState];

            pEncoder->lock();
            pEncoder->add(nCount);
            pEncoder->unlock();
        
            // Save current line states (will be the previous state on the next pass)
            pEncoder->setValA(nValA);
            pEncoder->setValB(nValB);
        }
    }
    lineA.release();
    lineB.release();
    lineSwitch.release();
}

int Encoder::getCount(){
    int nCount;
    lock();
    nCount = m_nCount / 2;
    unlock();
    return nCount;
}
void Encoder::clearCount(){
    lock();
    m_nCount = 0;
    unlock();
}


