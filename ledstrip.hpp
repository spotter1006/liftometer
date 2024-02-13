#define BIT_PERIOD_NS 1250
#define SHORT_PERIOD_NS 450
#define LONG_PERIOD_NS 800

#include <gpiod.hpp>
#include "rgb.hpp"
extern gpiod::chip chip;

class Ledstrip{
    private:
        int m_nElements;
        RGB* m_pData;
        static gpiod::line m_line;
        static void sender(Ledstrip* strip);
        void send();
        static void serialize(char data);
    public:
        Ledstrip(int elements, int gpio);
        ~Ledstrip();
        void bar(double pixels, RGB color);
        inline int getSize(){return m_nElements;}
        inline RGB getPixel(int i){return m_pData[i];}
};