# liftometer
C and C++ project for the  sailing device, "liftometer". 

- Runs on any linux target
- Hardware interfaces specfic to Raspberry Pi Zero 2 W or 4
- Uses MakerFocus PWM servo drivers for for the indicators
- Uses a rotary encoder connected to GPIO 23, 24. Push button connected to GPIO 25
- Uses a Bosch BNO055 IMU in UART communicatios mode connected to ttyS0. HW reset connected to GPIO 18.
- A simple user shell is accessible through the standard console
## Zram
The raspberry pi 0 has 500M of RAM, about half of which is used by the OS. This is enough to run liftometer, but if you want to use MS Code with the C/C++ extensions, the performance is inadequate. ZRAM makes a compressed swap drive.
```
git clone https://github.com/foundObjects/zram-swap
cd zram-swap/
sudo ./install.sh
```
### performance tweaks in /etc/sysctl.conf:
```
vm.vfs_cache_pressure=500
vm.swappiness=100
vm.dirty_background_ratio=1
vm.dirty_ratio=50
```
## Dependencies
BCM2835 libray
http://www.airspayce.com/mikem/bcm2835/
```
sudo apt install build-essential
sudo apt install gpiod libgpiod-dev
````
## Monitoring & Debug Tools
```
sudo apt install git
sudo apt install code
sudo apt install nmon
```
## Build
### Directory Structure
* liftometer
  * liftometer (application sources)
  * test (unit test case sources)
  * build
     * debug
     * relese
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
Launch release build with normal prority (niceness level 0:
```
/home/pi/swprojects/liftometer/liftometer/build/release/liftometer
```
Launch release build with high prority (niceness level -10):
```
sudo nice --10 /home/pi/swprojects/liftometer/liftometer/build/release/liftometer
```
* Type "h" for a list of commands
* Hit enter to display the data


