#include "bno055.h"
#include "bno055_support.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <cstdio>

struct bno055_t bno055;
int fd;     // Handle to serial port

#define I2C_BUFFER_LEN 8
#define I2C0           5

u8 txBuff[16];
u8 rxBuff[32];

// u8 len;
s8 nStatus;
const char* ackMessages[11] = {
    "",
    "WRITE_SUCCESS",
    "",
    "WRITE_FAIL",
    "REGMAP_INVALID_ADDRESS",
    "REGMAP_WRITE_DISABLED",
    "WRONG_START_BYTE",
    "BUS_OVER_RUN_ERROR",
    "MAX_LENGTH_ERROR",
    "MIN_LENGTH_ERROR",
    "RECEIVE_CHARACTER_TIMEOUT"
};
const char* responseMessages[11] = {
    "",
    "",
    "READ_FAIL",
    "",
    "REGMAP_INVALID_ADDRESS",
    "REGMAP_WRITE_DISABLED",
    "WRONG_START_BYTE",
    "BUS_OVER_RUN_ERROR",
    "MAX_LENGTH_ERROR",
    "MIN_LENGTH_ERROR",
    "RECEIVE_CHARACTER_TIMEOUT"
};

// UART functions
int BNO055_uart_init(int speed){
    char* port_name = PORT_NAME;
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    
    usleep(100000);         // Junk building up in the buffer. 100 ms delay
    tcflush(fd,TCIOFLUSH);  // Start with n empty serial port buffer
    
    struct termios tty;
    if (tcgetattr (fd, &tty) != 0)
    {
        printf("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 10;            // read blocks
    tty.c_cc[VTIME] = 2;            //  se1conds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable readingB
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    // tty.c_cflag |= parity; no parity
    tty.c_cflag &= ~CSTOPB;
    // tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf("error %d from tcsetattr", errno);
        return -1;
    }
    return fd;
}
// Dev_addr is the serial port file descriptor
s8 BNO055_uart_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){
    int nResult;
    int nRet;
    int nRead;
    int nLen;
    u8 data[128];

    txBuff[0] = COMMAND_START_BYTE;
    txBuff[1] = 1;          // Boolean read = true
    txBuff[2] = reg_addr;
    txBuff[3] = cnt;
    
    nResult = write(dev_addr, txBuff, 4);
    if(nResult == 4){
        nRead = read(dev_addr, rxBuff, 2);
        if(nRead == 2){
            if(rxBuff[0] == 0xBB){
                nLen = rxBuff[1];
                nResult = read(dev_addr, rxBuff, nLen);
                memcpy(reg_data, data, nLen);
                nRet = 0;
            }else if(rxBuff[0] == 0xEE){
                nStatus = rxBuff[1];
                printf("BNO055_uart_bus_read: 0x%x: %s\n", nStatus, responseMessages[nStatus]);
                nRet = -1;
            }else{
                printf("BNO055_uart_bus_read: Invalid response start byte: 0x%02hhX\n");
                nRet = -1;
            }          
        }else{
            printf("BNO055_uart_bus_read: error 0x%X reading 2 bytes from the UART\n", errno); 
        }
    }else{
        printf("BNO055_uart_bus_read: error 0x%X writing 4 bytes to the UART\n", errno);     
    }
    return nRet;
}

s8 BNO055_uart_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){
    int nResult;
    int nRetries = 1;

    txBuff[0] = COMMAND_START_BYTE;
    txBuff[1] = 0;          // write operation
    txBuff[2] = reg_addr;
    txBuff[3] = cnt;
    for(int i=0; i < cnt; i++){
        txBuff[i +4] = reg_data[i];
    }
    do{
        nResult = write(dev_addr, txBuff, cnt + 4);
        if(nResult > 0){
        nResult = read(dev_addr, rxBuff, 2);        // Get the response
            if(nResult ==2){
                if(rxBuff[0] == 0xEE){
                    if(rxBuff[0] == 7){     // Buffer overrun
                        nRetries--;         // retry per Bosch "Uart Interface" document
                    }
                    else if(rxBuff[1] != 1){
                        nStatus = rxBuff[1];
                        printf("BNO055_uart_bus_write: 0x%x: %s error received\n", nStatus, ackMessages[nStatus]);
                        nRetries = -1;      // Dont retry on any other codes
                    }else{                  // Success
                        nStatus = 0;
                        nRetries = 0;
                    }
                }else
                    printf("BNO055_uart_bus_write: Invalid start byte in command response: 0x%02hhX\n", rxBuff[0]);                
            }else
                printf("BNO055_uart_bus_write error 0x%hhu reading 2 bytes from the UART\n", errno); 
        }else
            printf("BNO055_uart_bus_write error 0x%hhu writing %hhu, bytes to the UART\n", errno, cnt + 4);
    } while(nRetries > 0);

    return nRetries > 0;
}

int BNO055_read_combined_data(bno055_euler_t* hrp, bno055_linear_accel_t* accel ){
    txBuff[0] = COMMAND_START_BYTE;  
    txBuff[1] = 0x01,   // Read operation
    txBuff[2] = BNO055_EULER_H_LSB_ADDR;   
    txBuff[3] = 20;     // Read 20 bytes (hrp, quaternion,linear acceleration)

    if(write(fd, txBuff, 4) != 4) 
        return -1;
    if(read(fd, rxBuff, 22) != 22) 
        return -2;
    if(rxBuff[0] != RESPONSE_START_BYTE) 
        return -3;
    if(rxBuff[1] != 20) 
        return -4;

    hrp->h = rxBuff[2] | (rxBuff[3] << 8);
    hrp->r = rxBuff[4] | (rxBuff[5] << 8);
    hrp->p = rxBuff[6] | (rxBuff[7] << 8);
    //...quaternion data
    accel->x = rxBuff[16] | (rxBuff[17] << 8);
    accel->y = rxBuff[18] | (rxBuff[19] << 8);
    accel->z = rxBuff[20] | (rxBuff[21] << 8);
    return 0;
}
void BNO055_delay_msek(u32 msek){ 
    usleep(msek * 1000);
}