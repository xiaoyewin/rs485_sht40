
#include "bh1750.h"

#include "i2c_analog.h"
#include "modbus_rtu.h"
#include "delay.h"
extern  T_MODBUS_REG gt_modbus_reg;



/****向BH1750单次写命令****/
void Single_Write_bh1750(uint8_t Reg_Address)
{
	IIC_Start();
	IIC_Send_Byte(0x46);  //发送器件地址0100 0110   最后一位0，表示写
	//IIC_Wait_Ack();
	IIC_Send_Byte(Reg_Address);  
	//IIC_Wait_Ack();
	IIC_Stop();
}




/****从BH1750连续读Lux****/
/****
模式选择：H-Resolution Mode,分辨率1lux
连续读Opecode：0001 0000
器件地址：0100011，ADDR接GND
****/

uint8_t BUF[8];

void  bh1750_ReadContinuous1(void)
{
//	u16 temp=0,temp1=0;
	IIC_Start();
    IIC_Send_Byte(0x47);  
	uint8_t errdata=i2c_read_ack();
   	if(errdata != 0) 
	{
			IIC_Stop();
			return ;
	}
 	BUF[0]=IIC_Read_Byte(1);
    modbusPoll();
	BUF[1]=IIC_Read_Byte(0);	
	//temp=bh1750_recv_byte(1);
	//temp1=bh1750_recv_byte(0);
	IIC_Stop();
	
	//temp2=temp1+(temp<<8);
	//lux=(float)temp2/1.2;    //lux是float型


}



/****BH1750初始化****/
void bh1750_Init(void)
{
    
	char dir=0;
	
    Single_Write_bh1750(0x01);
    Single_Write_bh1750(0x10);
    delay_ms(100);
    
    
    IIC_Start();
    IIC_Send_Byte(0x46);  //发送器件地址0100 0110   最后一位0，表示写
    uint8_t errdata=i2c_read_ack();
   	if(errdata != 0) 
	{
			IIC_Stop();
			return ;
	}
	IIC_Send_Byte(0x10);  //发送Continuous H-Resolution Mode的Opecode 0001 0000，分辨率为1lux
    errdata=i2c_read_ack();
   	if(errdata != 0) 
	{
			IIC_Stop();
			return ;
	}
	IIC_Stop();
    
    delay_ms(100);
    
}


void Get_Sunlight_Value()//获取光照强度值
{
    unsigned char value=0;
    float lux=0;
    bh1750_ReadContinuous1();
    uint16_t temp2=0;
    temp2=BUF[0];
	temp2=(temp2<<8)+BUF[1];
    
    float  rate = gt_modbus_reg.sunLight_correct_reg/100;
    uint32_t sun_max_value=  (uint32_t)temp2*rate;
    
    if(sun_max_value>=0xffff){
        gt_modbus_reg.sunLight_reg=0xffff;
    }
    else{
        gt_modbus_reg.sunLight_reg=(uint16_t)sun_max_value;
    }
    
		//delay_ms(1000);
	
}