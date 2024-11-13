
#include "yd3082.h"

#define YD3082_INPUT     GPIO_ResetBits(GPIOA, GPIO_Pin_6);
#define YD3082_OUTPUT    GPIO_SetBits(GPIOA, GPIO_Pin_6);

void yd3082_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* 使能GPIOB时钟 */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* 配置LED相应引脚PB1*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

}


void yd3082_setinout(uint8_t is_input)
{
    if(is_input==0){
        YD3082_OUTPUT
    }
    else{
        YD3082_INPUT
    }
}