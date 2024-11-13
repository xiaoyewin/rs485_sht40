#ifndef _I2C_ANALOG_H_
#define _I2C_ANALOG_H_

#include <stdint.h>

void IIC_GPIOInit(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(uint8_t ack);

uint8_t i2c_read_ack(void);
void i2c_send_ack();
void IIC_NAck(void);
void IIC_Ack(void);
void IIC_Stop(void);
void IIC_Start(void);

#endif