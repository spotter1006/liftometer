# liftometer
C and C++ project for the  sailing device, "liftometer". 
- Raspberryi Pi zero or 4 running Raspian (Bullseye)
- Interfaces to a BNO055 IMU in UART communicatios mode 
- Uses MakerFocus PWM servo drivers specific to the RasberryPi series of boards for the indicators
- A simple user shell is accessible through the standard console
## Dependencies
### gpiod
  sudo apt install gpiod
  libgpiod-dev
  
## Build
/usr/bin/g++ -I/home/pi/swprojects/liftometer/liftometer../ -pthread -fdiagnostics-color=always -g /home/pi/swprojects/liftometer/liftometer/*.c* -o /home/pi/swprojects/liftometer/liftometer/build/debug/liftometer "-lgpiodcxx"
