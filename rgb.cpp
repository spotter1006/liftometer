 #include "rgb.hpp"
RGB::RGB(){
    red = 0;
    green = 0;
    blue = 0;
 }
RGB::RGB(short red, short blue, short green){
    this->red = red;
    this->blue = blue;
    this->green = green;
}
RGB::RGB(const RGB &t){
    this->red = t.red;
    this->blue = t.blue;
    this->green = t.green;
}

void RGB::dim(double scale){
    red *= scale;
    green *= scale;
    blue *= scale;
}