# liftometer
C and C++ project for the  sailing device, "liftometer". 
- Runs on any linux target
- Hardware interfaces specfic to Raspberry Pi zero and 4
- Interfaces to a Bosch BNO055 IMU in UART communicatios mode 
- Uses MakerFocus PWM servo drivers for for the indicators
- Uses a rotary encoder connected to GPIO n,m. Pusbutton connected to GPIOx
- A simple user shell is accessible through the standard console
## Dependencies
gpiod:
	sudo apt install gpiod
 
   sudo apt install libgpiod-dev
## Build
This commandline is for a debug build comatible with GDB and VSCode:

/usr/bin/g++ -I/home/pi/swprojects/liftometer/liftometer../ -pthread -fdiagnostics-color=always -g /home/pi/swprojects/liftometer/liftometer/*.c* -o /home/pi/swprojects/liftometer/liftometer/build/debug/liftometer -lgpiodcxx
