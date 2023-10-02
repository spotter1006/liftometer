#include "average.hpp"
#include <bits/stdc++.h>

using namespace std;

Average::Average(int nSize){
    m_nSize = nSize;
    reset();
}
Average::~Average(){
    m_dData.clear();
};

void Average::reset(void){
    m_dData.clear();
}
void Average::add(int data){
    m_dData.push_front(data);
    if(m_dData.size() > m_nSize){
        m_dData.resize(m_nSize);
    }
}
double Average::calc(int nSamples){
    list<int> result;
    list<int>::iterator it;
    if(nSamples <= m_dData.size()){
        it = m_dData.begin();
        advance(it, nSamples);
    }else{
        it = m_dData.end();
    }
    partial_sum(m_dData.begin(), it, result.begin());
    return *result.begin() / (double)m_dData.size();
}
void Average::print(void){
    auto print = [](const int& n) { cout <<  n << ' '; };
    cout << m_dData.size() << " elements: ";
    for_each(m_dData.begin(), m_dData.end(), print);
    cout << endl;
}
