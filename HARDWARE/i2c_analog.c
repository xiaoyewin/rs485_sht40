
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


#define BIT32_SET(reg,pin,value) (reg = ((reg & ~(0x00000003 << (pin * 2))) | (value << (pin * 2))))//reg:�Ĵ���  pin���̺ܽ�  value���ο��ֲ��ϵ��趨ֵ

#define DS18B20_IO_IN()  				BIT32_SET(GPIOA->MODER,3,0x00)//GPIOA_3ģʽ������											00: ����ģʽ( ��λ״̬)	01: ͨ�����ģʽ	10: ���ù���ģʽ	11: ģ��ģʽ
#define DS18B20_IO_OUT() 				BIT32_SET(GPIOA->MODER,3,0x01)//GPIOA_3ģʽ�����	


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
      /* ʹ��GPIOAʱ�� */
      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

      /* ����LED��Ӧ����PB1*/
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  //  GPIO_OType_PP
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  //GPIO_PuPd_NOPULL 
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);
       
        
      I2C_SCL_H;
      I2C_SDA_H;  
      //GPIO_SetBits(GPIOA,GPIO_Pin_3);    //���A3
     // GPIO_SetBits(GPIOA,GPIO_Pin_4);    //���A3
}




#define I2C_DELAY_CLK  10//3

//����IIC��ʼ�ź�:
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

//����IICֹͣ�ź�
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


//����ACKӦ��
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

//������ACKӦ��		    
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
	  I2C_SDA_IN();  //SDA����Ϊ���� 
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




//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
		uint8_t i=0;

		I2C_SDA_OUT();
		
		delay_us(I2C_DELAY_CLK); //��������
	  I2C_SCL_L;//����ʱ�ӿ�ʼ���ݴ���
		for(i=0;i<8;i++)
		{
			  
				if((txd&0x80)>0) //0x80  1000 0000
						I2C_SDA_H;
				else
						I2C_SDA_L;

				txd<<=1;
				delay_us(I2C_DELAY_CLK); //��������
				I2C_SCL_H;
				delay_us(I2C_DELAY_CLK*3);
				I2C_SCL_L;
		}
//	i2c_read_ack();
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
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


