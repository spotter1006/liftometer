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
u8 rxBuff[16];

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
    tty.c_cc[VMIN]  = 5;            // read blocks
    tty.c_cc[VTIME] = 2;            // 0.1 seconds read timeout
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
                printf("BNO055_uart_bus_read: 0x%x: %s received\n", nStatus, responseMessages[nStatus]);
                nRet = -1;
            }else{
                printf("BNO055_uart_bus_read: Invalid start byte in response: 0x%X\n");
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
    int nRet;

    txBuff[0] = COMMAND_START_BYTE;
    txBuff[1] = 0;          // write operation
    txBuff[2] = reg_addr;
    txBuff[3] = cnt;
    for(int i=0; i < cnt; i++){
        txBuff[i +4] = reg_data[i];
    }
    nResult = write(dev_addr, txBuff, cnt + 4);
    if(nResult > 0){
        nResult = read(dev_addr, rxBuff, 2);        //Get the response
        if(nResult ==2){
            if(rxBuff[0] == 0xEE){
                if(rxBuff[1] != 1){
                    nStatus = rxBuff[1];
                    printf("BNO055_uart_bus_write: 0x%x: %s error received\n", nStatus, ackMessages[nStatus]);
                    nRet =-1;
                }else{
                    nStatus = 0;
                    nRet =0;
                }
            }else{
                printf("BNO055_uart_bus_write: Invalid start byte in command response: 0x%x\n", rxBuff[0]);
                nRet = -1;
            }        
        }else{
            printf("BNO055_uart_bus_write error 0x%hhu reading 2 bytes from the UART\n", errno); 
        }

    }else{
        printf("BNO055_uart_bus_write error 0x%hhu writing %hhu, bytes to the UART\n", errno, cnt + 4);
        nRet = -1;
    }

    return nRet;
}



void BNO055_delay_msek(u32 msek){ 
    sleep(msek);
}