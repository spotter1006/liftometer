# liftometer
C and C++ project for the  sailing device, "liftometer". 
- Raspberryi Pi zero or 4 running Raspian (Bullseye)
- Uses a Bosch BNO055 IMU in UART communicatios mode connected to ttyS0. HW reset connected to GPIO18.
- Uses 4 of 16 MakerFocus PWM servo drivers conncted to the Raspberry Pi IO headers
- A simple user shell is accessible through the standard console
## Dependencies
### gpiod
  sudo apt install gpiod
  libgpiod-dev
  
## Build
/usr/bin/g++ -I/home/pi/swprojects/liftometer/liftometer../ -pthread -fdiagnostics-color=always -g /home/pi/swprojects/liftometer/liftometer/*.c* -o /home/pi/swprojects/liftometer/liftometer/build/debug/liftometer "-lgpiodcxx"
