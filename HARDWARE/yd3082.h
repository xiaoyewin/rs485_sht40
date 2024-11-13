#ifndef __YD3082_H
#define __YD3082_H

#include "stm32f0xx.h"

#define LED_OFF                       GPIOB->BSRR = 0x02
#define LED_ON                        GPIOB->BRR = 0x02 
#define LED_TURN                      GPIOB->ODR ^= 0x02

#define RS485_CONTROL                  

void yd3082_init(void);
void yd3082_setinout(uint8_t is_input);

#endif
