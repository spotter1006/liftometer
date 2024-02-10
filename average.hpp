#ifndef _AVERAGE_H_
#define _AVERAGE_H_

#include <atomic>
#include <list>
#define RESET_THRESHOLD (20 * 16) // 20 degrees- BN0055 16 counts per degree (12 out of 16 bits)

using namespace std;

/// @brief Class for thread-safe exponential rolling averages
class Average{
public:
    Average(int nSize);
    ~Average();
    inline int getAverage(){return m_pAverage->load();}
    void add(int nPoint);
private:
    atomic_int *m_pAverage;
    int m_nSize;
    double m_partialSum;
    list<short> *m_pPoints;
};

#endif