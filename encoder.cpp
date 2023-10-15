#include "encoder.hpp"

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
    m_nCount = 0;
    gpiod::chip chip("gpiochip0");
    m_lineA = chip.get_line(18); 
    m_lineB = chip.get_line(19);
    m_lineA.request({"liftometer", gpiod::line_request::DIRECTION_INPUT, 0},0);  
    m_lineB.request({"liftometer", gpiod::line_request::DIRECTION_INPUT, 0},0);
}
Encoder::~Encoder(){
    pthread_cancel(m_tPoller.native_handle());
    m_lineA.release();
    m_lineB.release();
}
int Encoder::start(){
    thread m_tPoller(poller, this);
    m_tPoller.detach();
    return 0;
}
int Encoder::poller(Encoder* pEncoder){
    chrono::steady_clock::time_point timePt;
    while(1){
        timePt = chrono::steady_clock::now() + chrono::milliseconds(1);

        int nValA = pEncoder->m_lineA.get_value();
        int nValB = pEncoder->m_lineB.get_value();

        int nState =  pEncoder->getValA() << 4 | pEncoder->getValB() << 3 | nValA << 2 | nValB;
        int nCount = states[nCount];

        pEncoder->lock();
        pEncoder->add(nCount);
        pEncoder->unlock();

        pEncoder->setValA(nValA);
        pEncoder->setValB(nValB);

        this_thread::sleep_until(timePt);
    }
    return 0;
}
int Encoder::getCount(){
    int nCount;
    lock();
    nCount = getCount();
    unlock();
    return nCount;
}