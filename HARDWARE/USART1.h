
#ifndef _USART1_H
#define _USART1_H

#include "stm32f0xx.h"
#include <stdio.h>

typedef void (*uart_recv_data)(uint8_t );



void USART1_change(uint16_t baud,uint16_t stop_bit,uint16_t check_bit);


void USART1_Init(uint16_t baud,uint16_t stop_bit,uint16_t check_bit);
void USART1_SENDDATA(uint8_t *buf, uint8_t len);

//注册接收的回调函数
void registerRecvFunc(uart_recv_data func);

void uart_exti_int(void);
#endif
