# liftometer
C and C++ project for the  sailing device, "liftometer". 

- Runs on any linux target
- Hardware interfaces specfic to Raspberry Pi zero and 4
- Uses a Bosch BNO055 IMU in UART communicatios mode 
- Uses MakerFocus PWM servo drivers for for the indicators
- Uses a rotary encoder connected to GPIO 23, 24. Push button connected to GPIO 25
- A simple user shell is accessible through the standard console 
- Raspberryi Pi zero or 4 running Raspian (Bullseye)
- Uses a Bosch BNO055 IMU in UART communicatios mode connected to ttyS0. HW reset connected to GPIO 18.
- Uses 4 of 16 MakerFocus PWM servo drivers conncted to the Raspberry Pi IO headers
- A simple user shell is accessible through the standard console
- 
## Dependencies

```
sudo apt install build-essential
sudo apt install gpiod libgpiod-dev
````

## Monitoring & Debug Tools
```
sudo apt install nmon
sudo apt install code
sudo apt install nmon
```
## Build
### Directory Structure
* Liftometer
  * Liftometer (application sources)
  * Test (unit test case sources)
  
### Debug Bulid
This commandline is for a debug build comatible with GDB and VSCode:
```
/usr/bin/g++ -I/home/pi/swprojects/liftometer/liftometer../ -pthread -fdiagnostics-color=always -g /home/pi/swprojects/liftometer/liftometer/*.c* -o /home/pi/swprojects/liftometer/liftometer/build/debug/liftometer -lgpiodcxx
```
### Release Build
No debug symbols
```
/usr/bin/g++ -I/home/pi/swprojects/liftometer/liftometer../ -pthread /home/pi/swprojects/liftometer/liftometer/*.c* -o /home/pi/swprojects/liftometer/liftometer/build/release/liftometer -lgpiodcxx
```

## Usage
```
liftometer
```
* Type "h" for a list of commands
* Hit enter to display the data


