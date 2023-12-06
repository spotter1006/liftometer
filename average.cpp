#include "average.hpp"
#include <math.h>
using namespace std;

Average::Average(int nSize){
    m_nSize = nSize;
    m_nMax = 4096;              // BNO055 12 bits
    m_pAverage = new atomic_int(0);
    m_pCount = new atomic_int(0);

    m_dSmallConstant = 1.0 / m_nSize;
    m_dLargeConstant = 1.0 - m_dSmallConstant;
}
Average::~Average(){

}
void Average::add(short nPoint){

    double currentAverage = m_pAverage->load();    
    double dSmall, dLarge;
    double dRelativeChange = abs(nPoint - currentAverage);

    if(dRelativeChange > RESET_THRESHOLD){           // Seed average if large change       
        dLarge = 0.0;
        dSmall = 1.0;
    }else if(m_pCount->load() == m_nSize){
        dSmall = m_dSmallConstant;
        dLarge = m_dLargeConstant;
    }else{                                                  // Bucket not yet filled, go with current count
        dSmall = *m_pCount / m_nSize;
        dLarge = 1.0 - dSmall;
        m_pCount->fetch_add(1);
    }
    int nResult = currentAverage * dLarge + nPoint * dSmall;
    m_pAverage->exchange(nResult);
}

