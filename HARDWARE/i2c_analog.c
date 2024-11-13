
#include "i2c_analog.h"
#include "stm32f0xx.h"
#include "delay.h"


#define SHT30I2C_SCL   GPIO_Pin_4
#define SHT30I2C_SDA   GPIO_Pin_3
#define GPIO_I2C  GPIOA



#define I2C_SCL_H GPIO_SetBits(GPIO_I2C,SHT30I2C_SCL)
#define I2C_SCL_L GPIO_ResetBits(GPIO_I2C,SHT30I2C_SCL)

#define I2C_SDA_H GPIO_SetBits(GPIO_I2C,SHT30I2C_SDA)
#define I2C_SDA_L GPIO_ResetBits(GPIO_I2C,SHT30I2C_SDA) 

//use  analog io 
/*
PA3--------SDA
PA4--------SCLK
*/


#define BIT32_SET(reg,pin,value) (reg = ((reg & ~(0x00000003 << (pin * 2))) | (value << (pin * 2))))//reg:寄存器  pin：管教号  value：参考手册上的设定值

#define DS18B20_IO_IN()  				BIT32_SET(GPIOA->MODER,3,0x00)//GPIOA_3模式：输人											00: 输入模式( 复位状态)	01: 通用输出模式	10: 复用功能模式	11: 模拟模式
#define DS18B20_IO_OUT() 				BIT32_SET(GPIOA->MODER,3,0x01)//GPIOA_3模式：输出	


static void I2C_SDA_OUT(void)
{
//		GPIO_InitTypeDef GPIO_InitStructure;	
//		
//		GPIO_InitStructure.GPIO_Pin=SHT30I2C_SDA;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_Init(GPIOA,&GPIO_InitStructure);	
    DS18B20_IO_OUT();
}

static void I2C_SDA_IN(void)
{
//		GPIO_InitTypeDef GPIO_InitStructure;	
//		
//		GPIO_InitStructure.GPIO_Pin = SHT30I2C_SDA;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_Init(GPIOA,&GPIO_InitStructure);
    DS18B20_IO_IN();
}



void IIC_GPIOInit(void)
{					     
      GPIO_InitTypeDef  GPIO_InitStructure;
      /* 使能GPIOA时钟 */
      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

      /* 配置LED相应引脚PB1*/
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  //  GPIO_OType_PP
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  //GPIO_PuPd_NOPULL 
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);
       
        
      I2C_SCL_H;
      I2C_SDA_H;  
      //GPIO_SetBits(GPIOA,GPIO_Pin_3);    //输出A3
     // GPIO_SetBits(GPIOA,GPIO_Pin_4);    //输出A3
}




#define I2C_DELAY_CLK  10//3

//产生IIC起始信号:
void IIC_Start(void)
{
  I2C_SDA_OUT();
	
	I2C_SDA_H;
	I2C_SCL_H;
	delay_us(10);
	I2C_SDA_L;
	delay_us(I2C_DELAY_CLK);
	I2C_SCL_L;
	delay_us(I2C_DELAY_CLK);
}	  

//产生IIC停止信号
void IIC_Stop(void)
{
   I2C_SDA_OUT();

   I2C_SCL_L;
   I2C_SDA_L;
   I2C_SCL_H;
   delay_us(I2C_DELAY_CLK);
   I2C_SDA_H;
   delay_us(I2C_DELAY_CLK);						   	
}


//产生ACK应答
void IIC_Ack(void)
{
   I2C_SCL_L;
   I2C_SDA_OUT();
   I2C_SDA_L;
   delay_us(2);
   I2C_SCL_H;
   delay_us(I2C_DELAY_CLK);
   I2C_SCL_L;
}

//不产生ACK应答		    
void IIC_NAck(void)
{
   I2C_SCL_L;
   I2C_SDA_OUT();
   I2C_SDA_H;
   delay_us(2);
   I2C_SCL_H;
   delay_us(I2C_DELAY_CLK);
   I2C_SCL_L;
}	




void i2c_send_ack()  
{
   I2C_SCL_L;	
	  delay_us(I2C_DELAY_CLK);
   I2C_SDA_H;
   I2C_SDA_OUT();    
	 delay_us(I2C_DELAY_CLK);
   I2C_SCL_H;	
	 delay_us(I2C_DELAY_CLK);
   I2C_SCL_L;	
	 delay_us(I2C_DELAY_CLK);
}



uint8_t i2c_read_ack(void)  
{
	  I2C_SDA_IN();  //SDA设置为输入 
    I2C_SCL_L;
    delay_us(I2C_DELAY_CLK);
    I2C_SCL_H;	
    delay_us(I2C_DELAY_CLK);
    if(GPIO_ReadInputDataBit(GPIO_I2C, SHT30I2C_SDA))
    {
        I2C_SCL_L;
        //delay_us(I2C_DELAY_CLK);
        return 1;
    }
    else
    {
        I2C_SCL_L;
        //delay_us(I2C_DELAY_CLK);
        return 0;			
    }
}




//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t txd)
{                        
		uint8_t i=0;

		I2C_SDA_OUT();
		
		delay_us(I2C_DELAY_CLK); //发送数据
	  I2C_SCL_L;//拉低时钟开始数据传输
		for(i=0;i<8;i++)
		{
			  
				if((txd&0x80)>0) //0x80  1000 0000
						I2C_SDA_H;
				else
						I2C_SDA_L;

				txd<<=1;
				delay_us(I2C_DELAY_CLK); //发送数据
				I2C_SCL_H;
				delay_us(I2C_DELAY_CLK*3);
				I2C_SCL_L;
		}
//	i2c_read_ack();
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(uint8_t ack)
{
		uint8_t i=0,receive=0;

		I2C_SDA_IN();
		for(i=0;i<8;i++)
		{
				I2C_SCL_L;
				delay_us(I2C_DELAY_CLK);  //origin  4
				I2C_SCL_H;
				receive<<=1;
				if(GPIO_ReadInputDataBit(GPIO_I2C,SHT30I2C_SDA))
				receive++;
			 	delay_us(I2C_DELAY_CLK);	
		}

		if(ack==0)
				IIC_NAck();
		else
				IIC_Ack();

		return receive;
}


