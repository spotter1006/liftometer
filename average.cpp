#include "average.hpp"
#include <math.h>
using namespace std;

Average::Average(int nSize){
    m_nSize = nSize;

    m_pAverage = new atomic_int(0);
    m_partialSum = 0;
    m_pPoints = new list<short>();
}
Average::~Average(){

}
void Average::add(int nPoint){   
    m_pPoints->push_front(nPoint);
    if(m_pPoints->size() > m_nSize)
        m_pPoints->resize(m_nSize);
    
    // Force the upcast to double
    double dAverage = 0;  
    double dSize = (double) m_pPoints->size();
    double dPoint = (double) nPoint;
    
    if(m_pPoints->size() < m_nSize){                 
        m_partialSum += dPoint;
        dAverage = m_partialSum / dSize;      
    }else{   
        double current =  m_pAverage->load();   
        double oldest = m_pPoints->back();                            
        dAverage =  current + ((dPoint - oldest) / dSize);
    }
    
    m_pAverage->exchange((short) dAverage);
}


