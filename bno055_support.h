#ifndef _i2c_h_
#define _i2c_h_

#include "bno055.h"

#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)
// UART status bytes
#define COMMAND_START_BYTE 0xAA
#define RESPONSE_START_BYTE 0xBB
#define ERROR_START_BYTE 0xEE

#define PORT_NAME (char*)"/dev/ttyS0"

#ifdef __cplusplus 
extern "C" {
#endif

int BNO055_uart_init(int speed);
s8 BNO055_uart_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_uart_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msek);
int BNO055_getResponse(u8 dev_addr);

#endif
#ifdef __cplusplus
}
#endif
