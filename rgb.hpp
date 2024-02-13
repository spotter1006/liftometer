 class RGB{
        private:
            unsigned char red;
            unsigned char green;
            unsigned char blue;
        public:
            RGB();
            RGB(short red, short blue, short green);
            RGB(const RGB &t);

            inline unsigned char getRed(){return red;}
            inline unsigned char getBlue(){return blue;}
            inline unsigned char getGreen(){return green;}
            void dim(double scale);
    };