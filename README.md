# liftometer
C and C++ project for the  sailing device, "liftometer". 
- Runs on any linux target 
- Interfaces to a BNO055 IMU in UART communicatios mode 
- Uses PWM servo drivers specific to the RasberryPi series of boards for the indicators
- A simple user shell is accessible through the standard console
## Dependencies
The kernel mode GPIO daemon, installed with:
sudo apt install gpiod
sudo apt install libgpiod-dev
## Build
/usr/bin/g++ -I/home/pi/swprojects/liftometer/liftometer../ -pthread -fdiagnostics-color=always -g /home/pi/swprojects/liftometer/liftometer/*.c* -o /home/pi/swprojects/liftometer/liftometer/build/debug/liftometer
