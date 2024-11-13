#include "bmp280.h"

#include "i2c_analog.h"
#include "modbus_rtu.h"
extern  T_MODBUS_REG gt_modbus_reg;



#define BMP280_ADDRESS						0x76		//从设备地址
#define BMP280_RESET_VALUE					0xB6		//复位寄存器写入值





/*calibration parameters */
#define BMP280_DIG_T1_LSB_REG                0x88
#define BMP280_DIG_T1_MSB_REG                0x89
#define BMP280_DIG_T2_LSB_REG                0x8A
#define BMP280_DIG_T2_MSB_REG                0x8B
#define BMP280_DIG_T3_LSB_REG                0x8C
#define BMP280_DIG_T3_MSB_REG                0x8D
#define BMP280_DIG_P1_LSB_REG                0x8E
#define BMP280_DIG_P1_MSB_REG                0x8F
#define BMP280_DIG_P2_LSB_REG                0x90
#define BMP280_DIG_P2_MSB_REG                0x91
#define BMP280_DIG_P3_LSB_REG                0x92
#define BMP280_DIG_P3_MSB_REG                0x93
#define BMP280_DIG_P4_LSB_REG                0x94
#define BMP280_DIG_P4_MSB_REG                0x95
#define BMP280_DIG_P5_LSB_REG                0x96
#define BMP280_DIG_P5_MSB_REG                0x97
#define BMP280_DIG_P6_LSB_REG                0x98
#define BMP280_DIG_P6_MSB_REG                0x99
#define BMP280_DIG_P7_LSB_REG                0x9A
#define BMP280_DIG_P7_MSB_REG                0x9B
#define BMP280_DIG_P8_LSB_REG                0x9C
#define BMP280_DIG_P8_MSB_REG                0x9D
#define BMP280_DIG_P9_LSB_REG                0x9E
#define BMP280_DIG_P9_MSB_REG                0x9F




BMP280  bmp280;


uint8_t BMP280_Read_Byte(uint8_t reg)
{
    uint8_t errdata = 0x00;
	uint8_t rec_data;
	IIC_Start();
	IIC_Send_Byte(BMP280_ADDRESS<<1|0);
    
	errdata=i2c_read_ack();
    
   	if(errdata != 0) 
	{
			IIC_Stop();
			return errdata;
	}
    
	IIC_Send_Byte(reg);
    errdata=i2c_read_ack();
    
   	if(errdata != 0) 
	{
			IIC_Stop();
			return errdata;
	}
    
	IIC_Start();
	IIC_Send_Byte(BMP280_ADDRESS<<1|1);
    
	errdata=i2c_read_ack();
   	if(errdata != 0) 
	{
			IIC_Stop();
			return errdata;
	}
	rec_data = IIC_Read_Byte(0);	//不应答
	IIC_Stop();
	return rec_data;
}

static void BMP280_Write_Byte(uint8_t reg,uint8_t data)
{
    uint8_t errdata = 0x00;
	IIC_Start();
    
	IIC_Send_Byte(BMP280_ADDRESS<<1);
	errdata=i2c_read_ack();
   	if(errdata != 0) 
	{
			IIC_Stop();
			return;
	}
	IIC_Send_Byte(reg);
	errdata=i2c_read_ack();
   	if(errdata != 0) 
	{
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


//设置BMP过采样因子 MODE 
//BMP280_SLEEP_MODE||BMP280_FORCED_MODE||BMP280_NORMAL_MODE
void BMP280_Set_TemOversamp(BMP_OVERSAMPLE_MODE * Oversample_Mode)
{
	uint8_t Regtmp;
	Regtmp = ((Oversample_Mode->T_Osample)<<5)|
			 ((Oversample_Mode->P_Osample)<<2)|
			 ((Oversample_Mode)->WORKMODE);
	
	BMP280_Write_Byte(BMP280_CTRLMEAS_REG,Regtmp);
}


//设置保持时间和滤波器分频因子
void BMP280_Set_Standby_FILTER(BMP_CONFIG * BMP_Config)
{
	uint8_t Regtmp;
	Regtmp = ((BMP_Config->T_SB)<<5)|
			 ((BMP_Config->FILTER_COEFFICIENT)<<2)|
			 ((BMP_Config->SPI_EN));
	
	BMP280_Write_Byte(BMP280_CONFIG_REG,Regtmp);
}


// bmp280 的初始化
void Bmp_Init(void)
{
	uint8_t Lsb,Msb;
	/********************接下来读出矫正参数*********************/
	//温度传感器的矫正值
	Lsb = BMP280_Read_Byte(BMP280_DIG_T1_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_T1_MSB_REG);
	bmp280.T1 = (((uint16_t)Msb)<<8) + Lsb;			//高位加低位
	Lsb = BMP280_Read_Byte(BMP280_DIG_T2_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_T2_MSB_REG);
	bmp280.T2 = (((uint16_t)Msb)<<8) + Lsb;		
	Lsb = BMP280_Read_Byte(BMP280_DIG_T3_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_T3_MSB_REG);
	bmp280.T3 = (((uint16_t)Msb)<<8) + Lsb;		
	
	//大气压传感器的矫正值
	Lsb = BMP280_Read_Byte(BMP280_DIG_P1_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P1_MSB_REG);
	bmp280.P1 = (((uint16_t)Msb)<<8) + Lsb;		
	Lsb = BMP280_Read_Byte(BMP280_DIG_P2_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P2_MSB_REG);
	bmp280.P2 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P3_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P3_MSB_REG);
	bmp280.P3 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P4_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P4_MSB_REG);
	bmp280.P4 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P5_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P5_MSB_REG);
	bmp280.P5 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P6_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P6_MSB_REG);
	bmp280.P6 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P7_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P7_MSB_REG);
	bmp280.P7 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P8_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P8_MSB_REG);
	bmp280.P8 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P9_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P9_MSB_REG);
	bmp280.P9 = (((uint16_t)Msb)<<8) + Lsb;	
	/******************************************************/
	BMP280_Write_Byte(BMP280_RESET_REG,BMP280_RESET_VALUE);	//往复位寄存器写入给定值
	
	BMP_OVERSAMPLE_MODE			BMP_OVERSAMPLE_MODEStructure;
	BMP_OVERSAMPLE_MODEStructure.P_Osample = BMP280_P_MODE_3;
	BMP_OVERSAMPLE_MODEStructure.T_Osample = BMP280_T_MODE_1;
	BMP_OVERSAMPLE_MODEStructure.WORKMODE  = BMP280_NORMAL_MODE;
	BMP280_Set_TemOversamp(&BMP_OVERSAMPLE_MODEStructure);
	
	BMP_CONFIG					BMP_CONFIGStructure;
	BMP_CONFIGStructure.T_SB = BMP280_T_SB1;
	BMP_CONFIGStructure.FILTER_COEFFICIENT = BMP280_FILTER_MODE_4;
	BMP_CONFIGStructure.SPI_EN = DISABLE;
	
	BMP280_Set_Standby_FILTER(&BMP_CONFIGStructure);
}




//获取BMP当前状态
//status_flag = BMP280_MEASURING ||
//			 	BMP280_IM_UPDATE
uint8_t  BMP280_GetStatus(uint8_t status_flag)
{
	uint8_t flag;
	flag = BMP280_Read_Byte(BMP280_STATUS_REG);
	if(flag&status_flag)	return SET;
	else return RESET;
}




/*******************************下面是用来计算补偿值相关**********************************/
typedef			long signed int				BMP280_S32_t;	//有符号 64位！
typedef			long unsigned int			BMP280_U32_t;	//无符号 32位！
typedef			long long signed int		BMP280_S64_t;

#define	dig_T1			bmp280.T1	
#define	dig_T2			bmp280.T2	
#define	dig_T3			bmp280.T3	

#define	dig_P1			bmp280.P1
#define	dig_P2			bmp280.P2
#define	dig_P3			bmp280.P3
#define	dig_P4			bmp280.P4
#define	dig_P5			bmp280.P5
#define	dig_P6			bmp280.P6
#define	dig_P7			bmp280.P7
#define	dig_P8			bmp280.P8
#define	dig_P9			bmp280.P9
/************************************************CUT****************************************/
/**************************传感器值转定点值*************************************/
BMP280_S32_t t_fine;			//用于计算补偿
//我用浮点补偿
#ifdef USE_FIXED_POINT_COMPENSATE
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
// t_fine carries fine temperature as global value
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
	BMP280_S32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)dig_T1))) >> 12) * 
	((BMP280_S32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P)
{
	BMP280_S64_t var1, var2, p;
	var1 = ((BMP280_S64_t)t_fine) - 128000;
	var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
	var2 = var2 + ((var1*(BMP280_S64_t)dig_P5)<<17);
	var2 = var2 + (((BMP280_S64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3)>>8) + ((var1 * (BMP280_S64_t)dig_P2)<<12);
	var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig_P1)>>33;
	if (var1 == 0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((BMP280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);
	return (BMP280_U32_t)p;
}


/***********************************CUT*************************************/
#else
/**************************传感器值转定点值*************************************/
// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
// t_fine carries fine temperature as global value
double bmp280_compensate_T_double(BMP280_S32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (BMP280_S32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}

// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double bmp280_compensate_P_double(BMP280_S32_t adc_P)
{
    
    
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}
#endif



//温度值-℃
int BMP280_Get_Temperature(void)
{
	uint8_t XLsb,Lsb, Msb;
	long  Bit32;
	int  temperature;
	XLsb = BMP280_Read_Byte(BMP280_TEMPERATURE_XLSB_REG);
	Lsb	 = BMP280_Read_Byte(BMP280_TEMPERATURE_LSB_REG);
    modbusPoll();
	Msb	 = BMP280_Read_Byte(BMP280_TEMPERATURE_MSB_REG);
	Bit32 = ((long)(Msb << 12))|((long)(Lsb << 4))|(XLsb>>4);	//寄存器的值,组成一个浮点数
	temperature = bmp280_compensate_T_double(Bit32);
	return temperature;
}


//大气压值-Pa
//大气压值-Pa
double BMP280_Get_Pressure(void)
{
    BMP280_Get_Temperature();
	uint8_t XLsb,Lsb, Msb;
	long  Bit32;
	double pressure;
	XLsb = BMP280_Read_Byte(BMP280_PRESSURE_XLSB_REG);
	Lsb	 = BMP280_Read_Byte(BMP280_PRESSURE_LSB_REG);
    modbusPoll();
	Msb	 = BMP280_Read_Byte(BMP280_PRESSURE_MSB_REG);
	Bit32 = ((long)(Msb << 12))|((long)(Lsb << 4))|(XLsb>>4);	//寄存器的值,组成一个浮点数
	pressure = bmp280_compensate_P_double(Bit32)/10;
    gt_modbus_reg.gassPressure_reg= (int16_t)pressure+gt_modbus_reg.gassPressure_correct_reg;

	return pressure;
}







