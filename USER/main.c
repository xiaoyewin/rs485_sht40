
#include "stm32f0xx.h"
#include "delay.h"
#include "USART1.h"
#include "modbus_rtu.h"
#include <stdio.h>
#include <math.h>
#include "flash.h"

#include "sht40.h"
#include "bmp280.h"
#include "bh1750.h"


extern  T_MODBUS_REG gt_modbus_reg;
extern uint8_t need_save_reg_data;
extern uint16_t need_update_data ;

//int delay_self(int delay_ms)
//{
//    int value=0;
//    for(int i=0;i<delay_ms*100;i++){
//        value++;
//    }
//    return value;
//    
//}


void ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    
    /* GPIOC Periph clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;    //模拟输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;    
    GPIO_Init(GPIOA, &GPIO_InitStructure);     
}



//const int8_t ADC_order[4]={ADC_INX1,ADC_INX2,ADC_INX3,ADC_INX4};
//unsigned short ad_result;

void ADC_JZ(void)
{
	ADC1->CR&=~(1<<0);
	ADC1->CR|=0X80000000;    //set ADCAL ADC 校准
	while(ADC1->CR&0X80000000);//ADCAL 等待校准完成  0: 校准完成
}


void ADC_Chenel(int16_t ch)
{
	ADC1->CR|=1<<0;	//ADC 使能命令
	while(!(ADC1->ISR&0X00000001));//ADRDY  等待ADC准备好  1: ADC 已准备好开始转换
	ADC1->SMPR|=1<<1;	//13.5 ADC 时钟周期
	ADC1->CHSELR=1<<ch;	// 输入通道 x 被选为转换通道	
}



static uint16_t temperature_valuye=0;
int16_t Get_Adc_Average(int8_t ch)
{
	int32_t temp_val=0;
    int32_t times=100;
	int8_t t;
	ADC_Chenel(ch);
	//ADC1->CHSELR=1<<ch;	// 输入通道 x 被选为转换通道
    //去掉最大值和最小值
    uint16_t max_value=0x0;
    uint16_t min_value=0xffff;
    uint16_t temp_value;
	for(t=0;t<times;t++)
	{
		//Delay_10ms(1);
        ADC_StartOfConversion(ADC1); //软件启动ADC转换
        while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET); //等待ADC完成
        temp_value=ADC_GetConversionValue(ADC1);
        if(temp_value>max_value){
            max_value=temp_value;
        }
        if(temp_value<min_value){
            min_value=temp_value;
        }
        
		temp_val+=temp_value;		 
	}
    temp_val=temp_val-max_value-min_value;
	//ADC1->CR&=~(1<<0);
	//Delay_10ms(2);
	return temp_val/(times-2);
} 

double pt_r, pt_t;


void pt100_init()
{
    ADC_GPIO_Config();
     /* ADC1 Init ****************************************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能AHB预分频器到外设ADC1的开关
    RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4) ; //时钟分频48M/4=12M 最大时钟不超过14M

    ADC_DeInit(ADC1); //ADC复位
    ADC_DMACmd(ADC1, DISABLE); //禁止DMA

    ADC_InitTypeDef ADC_InitStuctrue; //声明ADC结构变量
    ADC_StructInit(&ADC_InitStuctrue); //根据ADC_InitStuctrue中指定参数初始化ADC1的寄存器
    ADC_InitStuctrue.ADC_Resolution=ADC_Resolution_12b; //采集设为12位精度即4095
    ADC_InitStuctrue.ADC_ContinuousConvMode=DISABLE; //禁止持续ADC，设为单次ADC采集
    ADC_InitStuctrue.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;

    ADC_InitStuctrue.ADC_DataAlign=ADC_DataAlign_Right; //数据右对齐
    ADC_InitStuctrue.ADC_ScanDirection=ADC_ScanDirection_Backward; //数据覆盖/浏览方向
    ADC_Init(ADC1,&ADC_InitStuctrue); //按以上参数设置ADC1

    ADC_ChannelConfig(ADC1,ADC_Channel_0,ADC_SampleTime_239_5Cycles);  //通道2 
    ADC_ChannelConfig(ADC1,ADC_Channel_1,ADC_SampleTime_239_5Cycles); //配置ADC1通道1即PA1，
    //ADC_ChannelConfig(ADC1,ADC_Channel_2,ADC_SampleTime_239_5Cycles); //配置ADC1通道1即PA1，


    //ADC_Chenel(1);
   
    ADC_GetCalibrationFactor(ADC1); //校准ADC1
    ADC_Cmd(ADC1,ENABLE); //使能ADC1
    while(ADC_GetFlagStatus(ADC1,ADC_FLAG_ADEN)==RESET); //等待ADC准备
    
}




 float  RTD_TAB_PT100[211] =   // 表格是以5度为一步，即-200, -195, - 190.....
 {
 18.52,20.68,22.83,24.97,27.10,29.22,31.34,33.44,35.54,37.64,                  // -200 ~ -155   10
 
 39.72,41.80,43.88,45.94,48.00,50.06,52.11,54.15,56.19,58.23,                  // -150 ~ -105
 
 60.26,62.28,64.30,66.31,68.33,70.33,72.33,74.33,76.33,78.32,                  // -100 ~ -55
 
 80.31,82.29,84.27,86.25,88.22,90.19,92.16,94.12,96.09,98.04,                  //        -50 ~ -5
 
 100.00,101.95,103.90,105.85,107.79,109.73,111.67,113.61,115.54,117.47,        // 0   ~ 45
 
 119.40,121.32,123.24,125.16,127.08,128.99,130.90,132.80,134.71,136.61,        // 50  ~ 95
 
 138.51,140.40,142.29,144.18,146.07,147.95,149.83,151.71,153.58,155.46,        // 100 ~ 145
 
 157.33,159.19,161.05,162.91,164.77,166.63,168.48,170.33,172.17,174.02,        // 150 ~ 195
 
 175.86,177.69,179.53,181.36,183.19,185.01,186.84,188.66,190.47,192.29,        // 200 ~ 245
 
 194.10,195.91,197.71,199.51,201.31,203.11,204.90,206.70,208.48,210.27,        // 250 ~ 295
 
 212.05,213.83,215.61,217.38,219.15,220.92,222.68,224.45,226.21,227.96,        // 300 ~ 345
 
 229.72,231.47,233.21,234.96,236.70,238.44,240.18,241.91,243.64,245.37,        // 350 ~ 395
 
 247.09,248.81,250.53,252.25,253.96,255.67,257.38,259.08,260.78,262.48,        // 400 ~ 445
 
 264.18,265.87,267.56,269.25,270.93,272.61,274.29,275.97,277.64,279.31,        // 450 ~ 495
 
 280.98,282.64,284.30,285.96,287.62,289.27,290.92,292.56,294.21,295.85,        // 500 ~ 545
 
 297.49,299.12,300.75,302.38,304.01,305.63,307.25,308.87,310.49,312.10,        // 550 ~ 595
 
 313.71,315.31,316.92,318.52,320.12,321.71,323.30,324.89,326.48,328.06,        // 600 ~ 645
 
 329.64,331.22,332.79,334.36,335.93,337.50,339.06,340.62,342.18,343.73,        // 650 ~ 695
 
 345.28,346.83,348.38,349.92,351.46,353.00,354.53,356.06,357.59,359.12,        // 700 ~ 745
 
 360.64,362.16,363.67,365.19,366.70,368.21,369.71,371.21,372.71,374.21,        // 750 ~ 795
 
 375.70,377.19,378.68,380.17,381.65,383.13,384.60,386.08,387.55,389.02,        // 800 ~ 845
 
 390.48        // 850
 };

 
//测量的范围是0-200度
void get_pt100_temperature()
{
        temperature_valuye=Get_Adc_Average(1);//这个是1通道
        //先根据电压算出电阻的阻值
        //需要矫正
       // temperature_valuye=4095;
    
        int32_t temp_adc;
        temperature_valuye-=185; //先算172
    
        temp_adc=(temperature_valuye*3299)/4095;  //3288、
        
        //每次采集出来的电压47mv v 这段下对应的值  为77  矫正值就改这个
        //还有一个对应的系数  //77  140   31  
    
   
       pt_r=-(200000*temperature_valuye + 708931363)/(temperature_valuye - 86421);
       //pt_r =-(200000*temperature_valuye + 733377272)/(temperature_valuye - 89401);
        //pt_r= -(200000*temperature_valuye + 757823181)/(temperature_valuye - 92381);
        //pt_r=-(200000*temperature_valuye + 777305454)/(temperature_valuye - 86079);
 
        //电阻值需要扩大100倍 
        
        //根据这个电压值进行纠正
    
        //pt_r=-(200000* + 669600000)/(temp_adc - 74152);
    
       // pt_r=-(200000*temp_adc + 648000000)/(temp_adc - 71760);
       // pt_r=-(200000*temp_adc + 648000000)/(temp_adc - 56760);
    
    
        //pt_r=-(22000*temperature_valuye + 88452000)/(11*temperature_valuye - 774774);
   
    
       // pt_r=-(200000*temperature_valuye + 1068795000)/(temperature_valuye - 117506);
 
      
        //先暂时的倍数是32倍
        //pt_r = (4091 * temp_adc )/(50100-temp_adc);
        pt_t = 3367.8214408824 - sqrt(439835508010000 - 5802000000 * pt_r) * 0.00017235436056;
        
        //pt_t = 3367.8214408824 - sqrt(439835508010000 - 580200000000 * pt_r) * 0.00017235436056;

       //保存温度值
       //先保存的温度通道1，到时候保存通道二
       
       
        int16_t temp_temperature=(int16_t)(pt_t*100);

        gt_modbus_reg.temperature_reg= temp_temperature+gt_modbus_reg.temperature_correct_reg;
        

        
        
        //在这里进行比例压缩
       // gt_modbus_reg.temperature_reg1=get_temp_correct_value(temp_temperature);
        //gt_modbus_reg.temperature_reg2=get_temp_correct_value(temp_temperature);


        //下面是CH2
        temperature_valuye=Get_Adc_Average(0);//这个是1通道

        temperature_valuye-=185; //先算172
    
        temp_adc=(temperature_valuye*3299)/4095;  //3288、
        

   
       pt_r=-(200000*temperature_valuye + 708931363)/(temperature_valuye - 86421);

        pt_t = 3367.8214408824 - sqrt(439835508010000 - 5802000000 * pt_r) * 0.00017235436056;
        

         temp_temperature=(int16_t)(pt_t*100);

         //gt_modbus_reg.temperature_reg2= temp_temperature+gt_modbus_reg.temperature_correct_reg2;
}

float CalculateTemperature(float fR)
 {
         float fTem;
 
        float fLowRValue;
 
        float fHighRValue;        
 
        int   iTem;
 
        uint8_t i;
 
        uint8_t cLimite = 0xFF;
 
        uint8_t cBottom, cTop;
 
         if (fR < RTD_TAB_PT100[0])                // 电阻值小于表格最值低于量程下限。
         {
                 return 2;
       }
        if (fR > RTD_TAB_PT100[210])        // 电阻值大于表格最大值，超出量程上限 。
        {
                 return 3;
        }
 
        cBottom = 0; 
        cTop    = 210;
 
        for (i=105; (cTop-cBottom)!=1; )        // 2分法查表。
        {
 
                if (fR < RTD_TAB_PT100[i])
                {
                       cTop = i;
                       i = (cTop + cBottom) / 2;
               }
                else if (fR > RTD_TAB_PT100[i])
               {
                         cBottom = i;
                         i = (cTop + cBottom) / 2;
                 }
                else
                {
                        iTem = (uint32_t)i * 5 - 200;
                        fTem = (float)iTem;
                       return fTem;
                }
        }
 
         iTem = (uint32_t)i * 5 - 200;
 
        fLowRValue  = RTD_TAB_PT100[cBottom];
 
         fHighRValue = RTD_TAB_PT100[cTop];
 
         fTem = ( ((fR - fLowRValue)*5) / (fHighRValue - fLowRValue) ) + iTem;        // 表格是以5度为一步的。
                                                                                       // 两点内插进行运算。
         return fTem;
 
 }

void flash_judge_reg()
{
    T_MODBUS_REG  temp_modbus_reg;
    readPackedMessageFromFlash((uint8_t *)&temp_modbus_reg,sizeof(T_MODBUS_REG));
	  need_save_reg_data=0;
    if(temp_modbus_reg.temperature_reg==0x1234){
        gt_modbus_reg=temp_modbus_reg;
    }
    else{
        uint16_t g_device_id=MODBUS_DEVICE_ID;
        
        //下面一些寄存器的初始化函数
        
        gt_modbus_reg.temperature_reg=0x0;
        gt_modbus_reg.humidity_reg=0x0;
        gt_modbus_reg.gassPressure_reg=0x0;
        gt_modbus_reg.model_number_reg = MODBUS_MODEL;
        
        gt_modbus_reg.testPoint_number_reg=0x1;//测试点默认1个
        gt_modbus_reg.device_addr_reg=g_device_id;//设备地为1
        gt_modbus_reg.baudrate_reg=3; //默认是3
        gt_modbus_reg.comm_mode_reg=0x1;//0x68
        gt_modbus_reg.protocol_reg=0x1;//0x69
        gt_modbus_reg.upload_time_reg=0x1;//0x6a
        gt_modbus_reg.checkbit_reg=0x1;//0x6b
        gt_modbus_reg.stopbit_reg=0x1;//0x6c
        gt_modbus_reg.temperature_correct_reg=0x0;//0x6d
        gt_modbus_reg.humidity_correct_reg=0x0;//0x6e
        gt_modbus_reg.gassPressure_correct_reg=0x0;//0x6e
        gt_modbus_reg.sunLight_correct_reg=100;
				
				 need_save_reg_data=1;

    }
    
    //这个会从FLASH 中读取数据，然后然后进行数据的校准
    //他会有个比例  

}


#define TEM_HUM_ADDR        0x88



//SHT30 CRC校验
unsigned char crc8_checksum(unsigned char *ptr,unsigned char len)
{
	unsigned char bit;        // bit mask
  unsigned char crc = 0xFF; // calculated checksum
  unsigned char byteCtr;    // byte counter
  
  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < len; byteCtr++)
  {
    crc ^= (ptr[byteCtr]);
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ 0x131;
      else           crc = (crc << 1);
    }
  }
	
	return crc;
}




//读取SHT30温湿度值
//unsigned char tem_hum_read(unsigned char *data)
//{
//    IIC_Start();
//    IIC_Send_Byte(TEM_HUM_ADDR + 1);


//    if(IIC_Wait_Ack() == 1)
//    {
//        return 1;
//    }

//    //前五次读取都要发送ack信号，最后一次就不用发了。
//    data[0] = IIC_Read_Byte(1);
//    data[1] = IIC_Read_Byte(1);
//    data[2] = IIC_Read_Byte(1);
//    data[3] = IIC_Read_Byte(1);
//    data[4] = IIC_Read_Byte(1);
//    data[5] = IIC_Read_Byte(0);
//    
//    if(data[2] != crc8_checksum(data,2))
//    {
//        return 1;
//    }
//    
//    if(data[5] != crc8_checksum(data + 3,2))
//    {
//        return 1;
//    }
//    
//    IIC_Stop();

//    return 0;
//}














////需要在这里定义一个寄存器大小的全局
////可以使用停机模式
//// An highlighted block
void system_enter_stopmode(void)
{
	//进入停止模式之前，设置以下IO口状态；停止模式保持IO状态不变
//    yd3082_setinout(1);  //配置为接收状态
//	uart_exti_int();  
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);	//使能PWR外设时钟
	//PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}


void close_all_pin()
{

    

  ADC_Cmd(ADC1,DISABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
    
  /* 使能GPIOB时钟 */

//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
    
    
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* 使能GPIOB时钟 */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* 配置LED相应引脚PB1*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

}









void TIM15_Config(void)
{
	
	
		GPIO_InitTypeDef  GPIO_InitStructure;
		
		/* 使能GPIOB时钟 */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);     //打开定时器15的时钟    

	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);     //打开复用功能
		/* 配置LED相应引脚PB1*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*
	
	int temp_0=0;
	
	while(1){
		if(temp_0==0){
				temp_0=1;
			GPIO_SetBits(GPIOA,GPIO_Pin_7);
				
		}
		else{
			temp_0=0;
				 GPIO_ResetBits(GPIOA,GPIO_Pin_7);
		}


	}
	*/
	
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;    


   
    TIM_DeInit(TIM15);                                                        //定时器15 整体复位
    TIM_TimeBaseStructure.TIM_Period = 47999;                //计数值
    TIM_TimeBaseStructure.TIM_Prescaler = 0;                                //预分频值
    TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;            //向上计数模式
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                    //这个值的作用暂时不清楚，配置那个参数都可以
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;    
    TIM_TimeBaseInit(TIM15, &TIM_TimeBaseStructure);
    TIM_PrescalerConfig(TIM15,0,TIM_PSCReloadMode_Immediate); //预分频系数(0+1)，预分频值即时装入
    TIM_ARRPreloadConfig(TIM15, ENABLE);//使能ARR预装载缓冲器

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWMI1模式
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//输出比较状态使能
    TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;//初始电平为高
    TIM_OCInitStructure.TIM_Pulse = 10000;                //占空比的多少
    TIM_OC2Init(TIM15, &TIM_OCInitStructure);  //不同的定时器通道是不一样的
    TIM_OC2PreloadConfig(TIM15, TIM_OCPreload_Enable);//使能外设预装载寄存器在CCR4

    TIM_CtrlPWMOutputs(TIM15, ENABLE);//使能TIM3外设的主输出
    TIM_Cmd(TIM15, ENABLE);    

}


//#define TEMP_N40_P80
//#define TEMP_N20_P80

#define TEMP_N30_P80

//#define TEMP_N40_P120

//#define TEMP_N0_P70

//#define TEMP_N0_P100


//#define TEMP_N40_P120
//0-70 
//-30-80 







//设置温度的PWM 值  
void set_temp_pwm(int16_t  temp){
	//-40 +80   -4000 -8000   //-20-+20    MAX 10000
	int16_t  pwm_value=0;
#ifdef TEMP_N40_P80
		pwm_value=(temp+4000)*0.8333;
#endif
	
#ifdef TEMP_N20_P80  
		pwm_value=temp+2000; //多40%
#endif	
	
#ifdef TEMP_N0_P100
  	 pwm_value=temp; //多40%
#endif	
	
#ifdef TEMP_N0_P70
		 pwm_value=temp*1.429;
#endif	
	
#ifdef TEMP_N30_P80
		 pwm_value=(temp+3000)*0.91;
#endif	
	
#ifdef TEMP_N40_P120
		 pwm_value=(temp+4000)*0.625;
#endif
	
	if(pwm_value<0){
		pwm_value=0;
	}
	else if(pwm_value>9999){
		pwm_value=9999;
	}
	
	pwm_value=9999-pwm_value;
		
	
	uint32_t  ccr_value=(uint32_t)pwm_value;
	
	  /* Set the Capture Compare Register value */
  TIM3->CCR2 = ccr_value;
  
}

//设置湿度的PWM 值
void set_humi_pwm(int16_t  humi){
	
	uint32_t  pwm_value=(uint32_t)humi;
	if(pwm_value>9999){
			pwm_value=9999;
	}
	pwm_value=9999-pwm_value;
	
	  /* Set the Capture Compare Register value */
  TIM3->CCR4 = pwm_value;
}


void TIM3_PWM_Init(uint16_t period, uint16_t prescaler)
{

	
	  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* 使能GPIOB时钟 */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* 配置LED相应引脚PB1*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);

  /* 使能GPIOA时钟 */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* 配置PWM相应引脚PA6，PA7*/
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);
	
	
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能定时器3时钟
    
		TIM_TimeBaseStructure.TIM_Period        = period;// 设置自动重装周期值
    TIM_TimeBaseStructure.TIM_Prescaler     = prescaler;//设置预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;//设置时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;//向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);    //初始化定时器3

    //TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;// PWM2模式
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
    //TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;//输出高
    //TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    
		//TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;//使能频道1配置
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		
		TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	
  /* 频道1，2，3，4的PWM 模式设置 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OCInitStructure.TIM_Pulse = 100;//使能频道1配置

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	
	
	
		TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能预装载寄存器
		TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能预装载寄存器
	
		TIM_Cmd(TIM3, ENABLE);                          //  使能定时器3
}

	

//自动上传和PWM冲突
//PB1 湿度   PA7 温度 PWM 输出
int main()
{
    //RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); // PLL 做系统时钟
    SystemInit();
    close_all_pin();
    delay_init();
    
    //设置一个定时器
    //使用高低电平看下，启动没有
    //先判断有flash 中有没有数据
    flash_judge_reg();
		//固话参数
		//固化参数
		
	/*
	gt_modbus_reg.baudrate_reg=4;
	gt_modbus_reg.checkbit_reg=1;
	gt_modbus_reg.device_addr_reg=5;
	
	*/

    modbusInit();
    uint8_t return_value;
    return_value=sht40_init();
	
		//TIM15_Config();
	  TIM3_PWM_Init(9999, 0);  //频率暂定为100K
	
	 I2C_SHT40_read_nodelay(0x44);
	

    
  uint8_t repeat_num=0;
	int16_t before_temp=gt_modbus_reg.temperature_reg;
	int16_t before_humi=gt_modbus_reg.humidity_reg;
	
	set_temp_pwm(before_temp);
	set_humi_pwm(before_humi);
	
	  static uint8_t send_num=0;
	int value=100;
	
   while(1)
	{
       // system_enter_stopmode();
        //这边抑制在查询
        //读取温度
        //startADC();
        //get_pt100_temperature();
        modbusPoll();  //进行轮询查询
        repeat_num++;
        
        if(((need_update_data&UPDATE_TEMP_DATA)==UPDATE_TEMP_DATA)||((need_update_data&UPDATE_HUMIDITY_DATA)==UPDATE_HUMIDITY_DATA)){
            I2C_SHT40_read(0x44);

            need_update_data&=~UPDATE_TEMP_DATA;
            need_update_data&=~UPDATE_HUMIDITY_DATA;
        }
				else{
					I2C_SHT40_read(0x44);
				}
				

				if(gt_modbus_reg.temperature_reg!=before_temp){
 					before_temp=gt_modbus_reg.temperature_reg;
					set_temp_pwm(before_temp);
				}
//				
				if(gt_modbus_reg.humidity_reg!=before_humi){
						before_humi=gt_modbus_reg.humidity_reg;
	
						set_humi_pwm(before_humi);
				}

       delay_ms(12); //大概10MS  

        if(need_save_reg_data){
            need_save_reg_data=0;
            gt_modbus_reg.temperature_reg=0x1234;
            writeMessageToFlash(&gt_modbus_reg,sizeof(T_MODBUS_REG));
        }

	}
}

