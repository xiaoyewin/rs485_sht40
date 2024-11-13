
#include "i2c_analog.h"
#include "sht40.h"
#include "modbus_rtu.h"
extern  T_MODBUS_REG gt_modbus_reg;

#define I2C_ADDR   0x44    //sht30 

uint16_t Check_falg = 0;
uint16_t Check_falg2 = 0;
uint8_t sht40_init()
{

        IIC_GPIOInit();
    
        uint8_t errdata = 0x00;

	/*
		IIC_Start();
		IIC_Send_Byte(I2C_ADDR << 1);//Ð´¼Ä´æÆ÷
		errdata = i2c_read_ack();//¶ÁÈ¡ACK
		if( errdata != 0) 
		{
				IIC_Stop();
				return errdata;
		}
		Check_falg2 = 1;
		IIC_Send_Byte(0x21); //·¢ËÍ¸ßÎ»
		errdata = i2c_read_ack();
		if( errdata != 0) 
		{
				IIC_Stop();
				return errdata;
		}
		
		Check_falg2 = 2;
		IIC_Send_Byte(0x30);  //·¢ËÍµÍÎ»
		errdata = i2c_read_ack();
		if( errdata != 0) 
		{
				IIC_Stop();
				return errdata;
		}
	
		Check_falg2 = 3;
		IIC_Stop();
		
		*/
		return 0;	
    
}




static void sht40_Write_Byte(uint8_t data)
{
   uint8_t errdata = 0x00;
	IIC_Start();
	
	IIC_Send_Byte(I2C_ADDR<<1);
	errdata=i2c_read_ack();
  if(errdata != 0) {
			IIC_Stop();
			return;
	}

	
	IIC_Send_Byte(data);
    errdata=i2c_read_ack();
   	if(errdata != 0) 
	{
			IIC_Stop();
			return;
	}
	IIC_Stop();
}




uint8_t I2C_SHT40_read_nodelay(uint16_t reg)
{
	uint8_t errdata = 0x00;
	uint8_t hum_h = 0,hum_l,hum_crc = 0;
	uint8_t temp_h = 0,temp_l,temp_crc = 0;
    uint8_t  TempratureValue,HumiValue;
    uint16_t Temp_Value,Hum_Value;
    uint16_t temp,hum = 0;
    int   Temprature,Humi;
    
	   sht40_Write_Byte(0xfd);
	  IIC_Start();
    IIC_Send_Byte((I2C_ADDR << 1) | 0x01);//¶Á¼Ä´æÆ÷

//	IIC_Send_Byte((I2C_ADDR << 1) | 0x01);//¶Á¼Ä´æÆ÷
	errdata = i2c_read_ack();
	if( errdata != 0) 
	{
			IIC_Stop();
			return errdata;
	}
	

	temp_h = IIC_Read_Byte(1);
	temp_l = IIC_Read_Byte(1);
	temp_crc = IIC_Read_Byte(1);
	hum_h = IIC_Read_Byte(1);
	hum_l = IIC_Read_Byte(1);
	hum_crc = IIC_Read_Byte(0);	
	hum = hum_h << 8 | hum_l;
	temp = temp_h << 8 | temp_l;
	
	
//	
//	    *temperature = ((21875 * (int32_t)words[0]) >> 13) - 45000;
//    *humidity = ((15625 * (int32_t)words[1]) >> 13) - 6000;
//		

	Temprature= (int)((175.0*(float)temp/65535.0-45.0)*100);
	
	Humi = (int)((125.0*(float)hum/65535.0-6.0)*100);
	
	if(Humi<0){
		Humi=0;
	}
	if(Humi>10000){
		Humi=10000;
	}
	  IIC_Stop();
    gt_modbus_reg.temperature_reg= (int16_t)Temprature+gt_modbus_reg.temperature_correct_reg;
    gt_modbus_reg.humidity_reg= (int16_t)Humi+gt_modbus_reg.humidity_correct_reg;
	return 0;	
}





//¶Á¼Ä´æÆ÷
uint8_t I2C_SHT40_read(uint16_t reg)
{
	uint8_t errdata = 0x00;
	uint8_t hum_h = 0,hum_l,hum_crc = 0;
	uint8_t temp_h = 0,temp_l,temp_crc = 0;
    uint8_t  TempratureValue,HumiValue;
    uint16_t Temp_Value,Hum_Value;
    uint16_t temp,hum = 0;
    int   Temprature,Humi;
    
	sht40_Write_Byte(0xfd);
	 delay_ms(10);
	   modbusPoll(); 
	
	  IIC_Start();
    IIC_Send_Byte((I2C_ADDR << 1) | 0x01);//¶Á¼Ä´æÆ÷

//	IIC_Send_Byte((I2C_ADDR << 1) | 0x01);//¶Á¼Ä´æÆ÷
	errdata = i2c_read_ack();
	if( errdata != 0) {
			IIC_Stop();
			return errdata;
	}
	
//	 IIC_Send_Byte(0x6);
//		errdata = i2c_read_ack();
//	if( errdata != 0) 
//	{
//			IIC_Stop();
//			return errdata;
//	}
	



	temp_h = IIC_Read_Byte(1);
    modbusPoll(); 
	temp_l = IIC_Read_Byte(1);
    modbusPoll(); 
	temp_crc = IIC_Read_Byte(1);
    modbusPoll(); 
	hum_h = IIC_Read_Byte(1);
    modbusPoll(); 
	hum_l = IIC_Read_Byte(1);
    modbusPoll(); 
	hum_crc = IIC_Read_Byte(0);	
    modbusPoll(); 


	hum = hum_h << 8 | hum_l;
	temp = temp_h << 8 | temp_l;
	
	
//	
//	    *temperature = ((21875 * (int32_t)words[0]) >> 13) - 45000;
//    *humidity = ((15625 * (int32_t)words[1]) >> 13) - 6000;
//		

	Temprature= (int)((175.0*(float)temp/65535.0-45.0)*100);
	
	Humi = (int)((125.0*(float)hum/65535.0-6.0)*100);
	
	if(Humi<0){
		Humi=0;
	}
	if(Humi>10000){
		Humi=10000;
	}
	
	  IIC_Stop();
    gt_modbus_reg.temperature_reg= (int16_t)Temprature+gt_modbus_reg.temperature_correct_reg;
    gt_modbus_reg.humidity_reg= (int16_t)Humi+gt_modbus_reg.humidity_correct_reg;
	return 0;	
}