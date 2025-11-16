#ifndef __I2C_H
#define __I2C_H

#include "RM.h"   

void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(unsigned char data);
uint8_t I2C_ReceiveAck(void);
void I2C_SendData(unsigned char data);

#endif
