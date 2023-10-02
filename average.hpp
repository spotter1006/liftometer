#ifndef _average_h_
#define _average_h_
#include <list>
class Average{
    public:
        Average(int);
        ~Average();
        inline double size(){return m_dData.size();} 
        double calc(int);
        void resize(int);
        void reset(void);
        void add(int);
        void print(void);
    private:
        int getDataSize(void);
        std::list<int> m_dData;
        int m_nSize;
};
#endif