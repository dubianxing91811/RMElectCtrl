#include "i2c.h"
#include "RM.h"  

static void I2C_Delay(void)
{
   for(int i = 0; i < 50; i++); //一个循环3至5个周期左右，这里用72mHz，则一个周期13.8ns，大概3/4us
}

void I2C_Start(void)
{
    SDA_High();
    SCL_High();
    I2C_Delay();
    SDA_Low();
    I2C_Delay();
    SCL_Low();
}

void I2C_Stop(void)
{
    SDA_Low();
    SCL_High();
    I2C_Delay();
    SDA_High();
    I2C_Delay();
}

void I2C_SendByte(uint8_t data)
{
    int i;
    for(i = 0; i < 8; i++)
    {
        if(data & 0x80){
            SDA_High();
        }else{
            SDA_Low();
}
        SCL_High();
        I2C_Delay();
        SCL_Low();
        I2C_Delay();

        data <<= 1;
    }
}

uint8_t I2C_ReceiveAck(void)
{
    SDA_High();  
    SCL_High();
    I2C_Delay();
   uint8_t ack = SDA_Read(); 
    SCL_Low();
    I2C_Delay();
    return ack;
}

void I2C_SendData(uint8_t data)
{
    I2C_Start();
    I2C_SendByte(data);
    I2C_ReceiveAck();
    I2C_Stop();
}

