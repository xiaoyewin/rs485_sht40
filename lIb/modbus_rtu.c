#include "modbus_rtu.h"
#include "string.h"

#include "sht40.h"
#include "bmp280.h"
#include "bh1750.h"







/*
*********************************************************************************************************
*	函 数 名: MODS_Poll
*	功能说明: 解析数据包. 在主程序中轮流调用。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/

//如果处于接收的状态中，是不能发送数据的

uint16_t need_update_data ;  //需要更新哪些数据

uint8_t need_save_reg_data ;  //需要保存数据


uint8_t start_recv_data ;  // 表示可以接收数据
uint8_t recv_data_complete; //表示接收数据完成

T_MODBUS gt_modbus;
T_MODBUS_REG gt_modbus_reg;

uint16_t  g_device_id=MODBUS_DEVICE_ID;








#define TIMER_NOW 

void TIM14_Init(int second)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0; //这个优先级要低点
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM_TimeBaseStructure.TIM_Period = 1000*second;  // 自动重装载寄存器周期的值(计数值)   50
  TIM_TimeBaseStructure.TIM_Prescaler = (1000 - 1);	//时钟预分频数 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//向上计数模式
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);			        // 清除溢出中断标志 
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  
  if(gt_modbus_reg.comm_mode_reg==0x2){
     // TIM_Cmd(TIM3, ENABLE);
  }
  else{
     // TIM_Cmd(TIM3, DISABLE);
  }
  TIM_SetCounter(TIM3,0);//清除计时
  
  
 
}

void change_TIM14_fre(int second){
    
  TIM_Cmd(TIM3, DISABLE);
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 1000*second;  // 自动重装载寄存器周期的值(计数值)   50
  TIM_TimeBaseStructure.TIM_Prescaler = (1000 - 1);	//时钟预分频数 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//向上计数模式
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;


  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);			        // 清除溢出中断标志 
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  
  if(gt_modbus_reg.comm_mode_reg==0x2){
     // TIM_Cmd(TIM3, ENABLE);
  }
  else{
     // TIM_Cmd(TIM3, DISABLE);
  } 
  
  TIM_SetCounter(TIM3,0);//清除计时
}


uint8_t  g_is_active_upload=0;
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
      TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update);
     // TIM_Cmd(TIM14, ENABLE);
       //这边需要主动上传数据
      
      
      
      g_is_active_upload=1;
            
  }
}




// CRC 高位字节值表
static const uint8_t s_CRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
// CRC 低位字节值表
const uint8_t s_CRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
/*
*********************************************************************************************************
*	函 数 名: CRC16_Modbus
*	功能说明: 计算CRC。 用于Modbus协议。
*	形    参: _pBuf : 参与校验的数据
*			  _usLen : 数据长度
*	返 回 值: 16位整数值。 对于Modbus ，此结果高字节先传送，低字节后传送。
*
*   所有可能的CRC值都被预装在两个数组当中，当计算报文内容时可以简单的索引即可；
*   一个数组包含有16位CRC域的所有256个可能的高位字节，另一个数组含有低位字节的值；
*   这种索引访问CRC的方式提供了比对报文缓冲区的每一个新字符都计算新的CRC更快的方法；
*
*  注意：此程序内部执行高/低CRC字节的交换。此函数返回的是已经经过交换的CRC值；也就是说，该函数的返回值可以直接放置
*        于报文用于发送；
*********************************************************************************************************
*/
uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen)
{
	uint8_t ucCRCHi = 0xFF; /* 高CRC字节初始化 */
	uint8_t ucCRCLo = 0xFF; /* 低CRC 字节初始化 */
	uint16_t usIndex;  /* CRC循环中的索引 */

  while (_usLen--)
  {
      usIndex = ucCRCHi ^ *_pBuf++; /* 计算CRC */
      ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
      ucCRCLo = s_CRCLo[usIndex];
  }
  return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

/*
*********************************************************************************************************
*	函 数 名: MODS_ReadRegValue
*	功能说明: 读取保持寄存器的值
*	形    参: reg_addr 寄存器地址
*			  reg_value 存放寄存器结果
*	返 回 值: 1表示OK 0表示错误
*********************************************************************************************************
*/


static uint8_t modbusReadRegValue(uint16_t reg_addr, uint8_t *reg_value)
{
	uint16_t value;
    
    if(reg_addr<=0x72){
        
        if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->temperature_reg)){
            //这个表示需要读取温度,那么需要采集一下
            need_update_data|=UPDATE_TEMP_DATA;
        }
        
        if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->humidity_reg)){
            //这个表示需要读取温度,那么需要采集一下
            need_update_data|=UPDATE_HUMIDITY_DATA;
        }
        
        if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->gassPressure_reg)){
               need_update_data|=UPDATE_GASSPRESSURE_DATA;
        }
        
        if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->sunLight_reg)){
            need_update_data|=UPDATE_SUNLIGHT_DATA;
        }
            
    
        uint16_t * temp_reg  =(uint16_t *)&gt_modbus_reg;
        //value = gt_modbus_reg.temperature_reg1;
        value = temp_reg[reg_addr];

        reg_value[0] = value >> 8;
        reg_value[1] = value;
        return 1;
    }
    else{
       return 0;
    }									/* 读取成功 */
}

/*
*********************************************************************************************************
*	函 数 名: modbusWriteRegValue
*	功能说明: 读取保持寄存器的值
*	形    参: reg_addr 寄存器地址
*			  reg_value 寄存器值
*	返 回 值: 1表示OK 0表示错误
*********************************************************************************************************
*/
static uint8_t modbusWriteRegValue(uint16_t reg_addr, int16_t reg_value)
{
    //不可修改
    if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->model_number_reg)){
        return 1;
    }
    if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->baudrate_reg)){
        if((reg_value>6)||(reg_value<1)){
            return 1;
        }
    }
    if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->checkbit_reg)){
        if((reg_value>3)||(reg_value<1)){
            return 1;
        }
    }
    
    if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->stopbit_reg)){
        if((reg_value>3)||(reg_value<1)){
            return 1;
        }
    }
    
    if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->temperature_correct_reg)){
        if((reg_value>1000)||(reg_value<-1000)){
            return 1;
        }
    }
    
     if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->humidity_correct_reg)){
        if((reg_value>1000)||(reg_value<-1000)){
            return 1;
        }
    }
     
    if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->gassPressure_correct_reg)){
        if((reg_value>1000)||(reg_value<-1000)){
            return 1;
        }
    }
    
   if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->sunLight_correct_reg)){
        if((reg_value>2000)||(reg_value<0)){
            return 1;
        }
    }
   
   if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->upload_time_reg)){
        if((reg_value<1)||(reg_value>3600)){
            return 1;
        }
       // change_TIM14_fre(reg_value);
    }
   
   if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->comm_mode_reg)){

       if(reg_value==0x2){
           gt_modbus_reg.comm_mode_reg=reg_value;
          // change_TIM14_fre(gt_modbus_reg.upload_time_reg);
       }
        
    }
   
   
    if(reg_addr<=0x72){
        int16_t * temp_reg  =(int16_t *)&gt_modbus_reg;
        temp_reg[reg_addr]=reg_value;
        return 1;
    }
    else{  //如果大于0xff00 
       
       return 0;
    }
 
}




/*
*********************************************************************************************************
*	函 数 名: BEBufToUint16
*	功能说明: 将2字节数组(大端Big Endian次序，高字节在前)转换为16位整数
*	形    参: _pBuf : 数组
*	返 回 值: 16位整数值
*
*   大端(Big Endian)与小端(Little Endian)
*********************************************************************************************************
*/
uint16_t BEBufToUint16(uint8_t *_pBuf)
{
    return (((uint16_t)_pBuf[0] << 8) | _pBuf[1]);
}



/*
*********************************************************************************************************
*	函 数 名: MODS_SendWithCRC
*	功能说明: 发送一串数据, 自动追加2字节CRC
*	形    参: _pBuf 数据；
*			  _ucLen 数据长度（不带CRC）
*	返 回 值: 无
*********************************************************************************************************
*/
static void modbusSendWithCRC(uint8_t *_pBuf, uint8_t _ucLen)
{
	uint16_t crc;
	uint8_t buf[S_TX_BUF_SIZE];

	memcpy(buf, _pBuf, _ucLen);
	crc = CRC16_Modbus(_pBuf, _ucLen);
	buf[_ucLen++] = crc >> 8;
	buf[_ucLen++] = crc;
    
    //如果串口不忙的话，就可以发送
    
    yd3082_setinout(0);
    USART1_SENDDATA(buf,_ucLen);
    yd3082_setinout(1);

}
/*
*********************************************************************************************************
*	函 数 名: MODS_SendAckErr
*	功能说明: 发送错误应答
*	形    参: _ucErrCode : 错误代码
*	返 回 值: 无
*********************************************************************************************************
*/
static void modbusSendAckErr(uint8_t _ucErrCode)
{
	uint8_t txbuf[3];

	txbuf[0] = gt_modbus.rx_buf[0];					/* 485地址 */
	txbuf[1] = gt_modbus.rx_buf[1] | 0x80;				/* 异常的功能码 */
	txbuf[2] = _ucErrCode;							/* 错误代码(01,02,03,04) */

	modbusSendWithCRC(txbuf, 3);
}


/*
这个需要放到中断函数中 
*/
void modbusReciveData(uint8_t byte)
{
	/*
		3.5个字符的时间间隔，只是用在RTU模式下面，因为RTU模式没有开始符和结束符，
		两个数据包之间只能靠时间间隔来区分，Modbus定义在不同的波特率下，间隔时间是不一样的，
		所以就是3.5个字符的时间，波特率高，这个时间间隔就小，波特率低，这个时间间隔相应就大

		4800  = 7.297ms
		9600  = 3.646ms
		19200  = 1.771ms
		38400  = 0.885ms
	*/
    if(start_recv_data>0){
        //表示已经超时,需要判断第一个字符
        if(((gt_modbus_reg.device_addr_reg & 0xff)==byte)||(MODBUS_DEVICE_QUERY==byte)){  //那就说明数据有效
            start_recv_data=0;//清除掉超时标记
            gt_modbus.rx_count=0; //开始接收数据
            recv_data_complete=0;//数据接收开始
            gt_modbus.rx_buf[gt_modbus.rx_count++] = byte;
        }
    }
    else if(recv_data_complete==0){
        if (gt_modbus.rx_count < S_RX_BUF_SIZE)
        {
            gt_modbus.rx_buf[gt_modbus.rx_count++] = byte;
        }
    }
	
    //在这里需要重新设置定时器
}


/*
*********************************************************************************************************
*	函 数 名: MODS_01H
*	功能说明: 读取线圈状态（对应远程开关D01/D02/D03）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void modbus_01_func(void)
{
	/*
	 举例：
		主机发送:
			11 从机地址
			01 功能码
			00 寄存器起始地址高字节
			13 寄存器起始地址低字节
			00 寄存器数量高字节
			25 寄存器数量低字节
			0E CRC校验高字节
			84 CRC校验低字节

		从机应答: 	1代表ON，0代表OFF。若返回的线圈数不为8的倍数，则在最后数据字节未尾使用0代替. BIT0对应第1个
			11 从机地址
			01 功能码
			05 返回字节数
			CD 数据1(线圈0013H-线圈001AH)
			6B 数据2(线圈001BH-线圈0022H)
			B2 数据3(线圈0023H-线圈002AH)
			0E 数据4(线圈0032H-线圈002BH)
			1B 数据5(线圈0037H-线圈0033H)
			45 CRC校验高字节
			E6 CRC校验低字节

		例子:
			01 01 10 01 00 03   29 0B	--- 查询D01开始的3个继电器状态
			01 01 10 03 00 01   09 0A   --- 查询D03继电器的状态
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint16_t m;
	uint8_t status[10];
	
	gt_modbus.rsp_code = RSP_OK;

	/* 没有外部继电器，直接应答错误 */
	if (gt_modbus.rx_count != 8)
	{
		gt_modbus.rsp_code = RSP_ERR_VALUE;				/* 数据值域错误 */
		return;
	}

	reg = BEBufToUint16(&gt_modbus.rx_buf[2]); 			/* 寄存器号 */
	num = BEBufToUint16(&gt_modbus.rx_buf[4]);				/* 寄存器个数 */

	m = (num + 7) / 8;
	
	if ((reg >= REG_D01) && (num > 0) && (reg + num <= REG_DXX + 1))
	{
		for (i = 0; i < m; i++)
		{
			status[i] = 0;
		}
		for (i = 0; i < num; i++)
		{

		}
	}
	else
	{
		gt_modbus.rsp_code = RSP_ERR_REG_ADDR;				/* 寄存器地址错误 */
	}

	if (gt_modbus.rsp_code == RSP_OK)						/* 正确应答 */
	{
		gt_modbus.tx_count = 0;
		gt_modbus.tx_buf[gt_modbus.tx_count++] = gt_modbus.rx_buf[0];
		gt_modbus.tx_buf[gt_modbus.tx_count++] = gt_modbus.rx_buf[1];
		gt_modbus.tx_buf[gt_modbus.tx_count++] = m;			/* 返回字节数 */

		for (i = 0; i < m; i++)
		{
			gt_modbus.tx_buf[gt_modbus.tx_count++] = status[i];	/* 继电器状态 */
		}
		modbusSendWithCRC(gt_modbus.tx_buf, gt_modbus.tx_count);
	}
	else
	{
		modbusSendAckErr(gt_modbus.rsp_code);				/* 告诉主机命令错误 */
	}
}


/*
*********************************************************************************************************
*	函 数 名: MODS_03H
*	功能说明: 读取保持寄存器 在一个或多个保持寄存器中取得当前的二进制值
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void modbus_03_func(void)
{
	/*
		从机地址为11H。保持寄存器的起始地址为006BH，结束地址为006DH。该次查询总共访问3个保持寄存器。

		主机发送:
			11 从机地址
			03 功能码
			00 寄存器地址高字节
			6B 寄存器地址低字节
			00 寄存器数量高字节
			03 寄存器数量低字节
			76 CRC高字节
			87 CRC低字节

		从机应答: 	保持寄存器的长度为2个字节。对于单个保持寄存器而言，寄存器高字节数据先被传输，
					低字节数据后被传输。保持寄存器之间，低地址寄存器先被传输，高地址寄存器后被传输。
			11 从机地址
			03 功能码
			06 字节数
			00 数据1高字节(006BH)
			6B 数据1低字节(006BH)
			00 数据2高字节(006CH)
			13 数据2 低字节(006CH)
			00 数据3高字节(006DH)
			00 数据3低字节(006DH)
			38 CRC高字节
			B9 CRC低字节

		例子:
			01 03 30 06 00 01  6B0B      ---- 读 3006H, 触发电流
			01 03 4000 0010 51C6         ---- 读 4000H 倒数第1条浪涌记录 32字节
			01 03 4001 0010 0006         ---- 读 4001H 倒数第1条浪涌记录 32字节

			01 03 F000 0008 770C         ---- 读 F000H 倒数第1条告警记录 16字节
			01 03 F001 0008 26CC         ---- 读 F001H 倒数第2条告警记录 16字节

			01 03 7000 0020 5ED2         ---- 读 7000H 倒数第1条波形记录第1段 64字节
			01 03 7001 0020 0F12         ---- 读 7001H 倒数第1条波形记录第2段 64字节

			01 03 7040 0020 5F06         ---- 读 7040H 倒数第2条波形记录第1段 64字节
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[64];
    uint8_t addr;

	gt_modbus.rsp_code = RSP_OK;

	if (gt_modbus.rx_count != 8)								/* 03H命令必须是8个字节 */
	{
		gt_modbus.rsp_code = RSP_ERR_VALUE;					/* 数据值域错误 */
		goto err_ret;
	}

    addr=gt_modbus.rx_buf[0];
	reg = BEBufToUint16(&gt_modbus.rx_buf[2]); 				/* 寄存器号 */
	num = BEBufToUint16(&gt_modbus.rx_buf[4]); //需要读取的数量					/* 寄存器个数 */
	if (num > sizeof(reg_value) / 2)  //不能读取全部寄存器的
	{
		gt_modbus.rsp_code = RSP_ERR_VALUE;					/* 数据值域错误 */
		goto err_ret;
	}

	for (i = 0; i < num; i++)
	{
		if (modbusReadRegValue(reg, &reg_value[2 * i]) == 0)	/* 读出寄存器值放入reg_value */
		{
			gt_modbus.rsp_code = RSP_ERR_REG_ADDR;				/* 寄存器地址错误 */
			break;
		}
		reg++;
	}

err_ret:
	if (gt_modbus.rsp_code == RSP_OK)							/* 正确应答 */
	{
		gt_modbus.tx_count = 0;
		gt_modbus.tx_buf[gt_modbus.tx_count++] = (gt_modbus_reg.device_addr_reg&0xff);
		gt_modbus.tx_buf[gt_modbus.tx_count++] = gt_modbus.rx_buf[1];
		gt_modbus.tx_buf[gt_modbus.tx_count++] = num * 2;			/* 返回字节数 */

		for (i = 0; i < num; i++)
		{
			gt_modbus.tx_buf[gt_modbus.tx_count++] = reg_value[2*i];
			gt_modbus.tx_buf[gt_modbus.tx_count++] = reg_value[2*i+1];
		}
		modbusSendWithCRC(gt_modbus.tx_buf, gt_modbus.tx_count);	/* 发送正确应答 */
	}
	else
	{
		modbusSendAckErr(gt_modbus.rsp_code);					/* 发送错误应答 */
	}
}


//如果为0,直接写0.00

int int_convert_ascii(int reg,uint8_t* buf,int point){
    //最大的值为65535 也就  100000
    
    uint8_t * p_buf= (uint8_t *)buf;
    int max_rate=10000;
    int buf_len=0;
    uint8_t is_start=0;
    //一般需要在那里加小数点
    int point_value=0;
    switch(point){
        case 0:
            point_value=0;
            break;
        case 1:
            point_value=10;
            break;
        case 2:
            point_value=100;
            break;
        case 3:
            point_value=1000;
            break;
        case 4:
            point_value=10000;
            break;
        default:
            point_value=0;
            break;
    }
    
    while(max_rate>0){
        uint8_t temp=reg/max_rate;
        if(temp==0){
           if(is_start){
                p_buf[buf_len++]=temp+48;
            }
        }
        else{
            p_buf[buf_len++]=temp+48;
            is_start=1;
            reg-=(max_rate*temp);
        }
        if(point_value==max_rate){
            if(is_start==0){
               p_buf[buf_len++]=48;  
            }
            is_start=1;
            p_buf[buf_len++]=46; 
        }
        max_rate/=10;
    }
    
    if(!is_start){
        p_buf[buf_len++]=48;  
    }
    
    return buf_len;
}

void active_upload(){
    
    uint8_t buf[30];
    gt_modbus.tx_count = 0;
    
    gt_modbus.tx_buf[gt_modbus.tx_count++] = (gt_modbus_reg.device_addr_reg&0xff);
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 0x3; 
    gt_modbus.tx_count++;  
    
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 123; //{
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 34 ; //"
	
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 100; //d
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 97; //a
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 116; //t
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 97; //a
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 34 ; //"
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 58 ; //:
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 34 ; //"
    
    // gt_modbus.tx_count  
    
    int16_t reg= gt_modbus_reg.temperature_reg;
    
    gt_modbus.tx_count+=int_convert_ascii(gt_modbus_reg.temperature_reg,&gt_modbus.tx_buf[gt_modbus.tx_count],2);
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 44 ; //,
    gt_modbus.tx_count+=int_convert_ascii(gt_modbus_reg.humidity_reg,&gt_modbus.tx_buf[gt_modbus.tx_count],2);
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 44 ; //,
    gt_modbus.tx_count+=int_convert_ascii(gt_modbus_reg.gassPressure_reg,&gt_modbus.tx_buf[gt_modbus.tx_count],2);
    gt_modbus.tx_buf[gt_modbus.tx_count++] = 44 ; //,
    gt_modbus.tx_count+=int_convert_ascii(gt_modbus_reg.sunLight_reg,&gt_modbus.tx_buf[gt_modbus.tx_count],0);
    
    //然后需要采集的数据自己采集下                    
     gt_modbus.tx_buf[gt_modbus.tx_count++] = 34 ; // "   
     gt_modbus.tx_buf[gt_modbus.tx_count++] = 125; // }
    
     gt_modbus.tx_buf[2] = 0x3; 
     
     modbusSendWithCRC(gt_modbus.tx_buf, gt_modbus.tx_count);	/* 发送正确应答 */

            //这个表示需要读取温度,那么需要采集一下
      need_update_data|=UPDATE_TEMP_DATA;

            //这个表示需要读取温度,那么需要采集一下
      need_update_data|=UPDATE_HUMIDITY_DATA;

      need_update_data|=UPDATE_GASSPRESSURE_DATA;

      need_update_data|=UPDATE_SUNLIGHT_DATA;

}



/*
*********************************************************************************************************
*	函 数 名: MODS_SendAckOk
*	功能说明: 发送正确的应答.
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void modbusSendAckOk(void)
{
	uint8_t txbuf[6];
	uint8_t i;

	for (i = 0; i < 6; i++)
	{
		txbuf[i] = gt_modbus.rx_buf[i];
	}
	modbusSendWithCRC(txbuf, 6);
}




/*
*********************************************************************************************************
*	函 数 名: MODS_06H
*	功能说明: 读取保持寄存器 在一个或多个保持寄存器中取得当前的二进制值
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void modbus_06_func(void)
{

/*
		写保持寄存器。注意06指令只能操作单个保持寄存器，16指令可以设置单个或多个保持寄存器
		主机发送:
			11 从机地址
			06 功能码
			00 寄存器地址高字节
			01 寄存器地址低字节
			00 数据1高字节
			01 数据1低字节
			9A CRC校验高字节
			9B CRC校验低字节

		从机响应:
			11 从机地址
			06 功能码
			00 寄存器地址高字节
			01 寄存器地址低字节
			00 数据1高字节
			01 数据1低字节
			1B CRC校验高字节
			5A	CRC校验低字节

		例子:
			01 06 30 06 00 25  A710    ---- 触发电流设置为 2.5
			01 06 30 06 00 10  6707    ---- 触发电流设置为 1.0


			01 06 30 1B 00 00  F6CD    ---- SMA 滤波系数 = 0 关闭滤波
			01 06 30 1B 00 01  370D    ---- SMA 滤波系数 = 1
			01 06 30 1B 00 02  770C    ---- SMA 滤波系数 = 2
			01 06 30 1B 00 05  36CE    ---- SMA 滤波系数 = 5

			01 06 30 07 00 01  F6CB    ---- 测试模式修改为 T1
			01 06 30 07 00 02  B6CA    ---- 测试模式修改为 T2

			01 06 31 00 00 00  8736    ---- 擦除浪涌记录区
			01 06 31 01 00 00  D6F6    ---- 擦除告警记录区

*/

	uint16_t reg;
	uint16_t value;

	gt_modbus.rsp_code = RSP_OK;

	if(gt_modbus.rx_count != 8)
	{
		gt_modbus.rsp_code = RSP_ERR_VALUE;		/* 数据值域错误 */
		goto err_ret;
	}

	reg = BEBufToUint16(&gt_modbus.rx_buf[2]); 	/* 寄存器号 */
	value = BEBufToUint16(&gt_modbus.rx_buf[4]);	/* 寄存器值 */

	if (modbusWriteRegValue(reg, (int16_t)value) == 1)	/* 该函数会把写入的值存入寄存器 */
	{
        //需要判断值有没有区别 有区别需要修改了
        
        if(reg*2 == (uint16_t)(&((T_MODBUS_REG *)0)->baudrate_reg))
        {
            USART1_change(gt_modbus_reg.baudrate_reg,gt_modbus_reg.stopbit_reg,gt_modbus_reg.checkbit_reg); 
        }
        else if(reg*2 == (uint16_t)(&((T_MODBUS_REG *)0)->checkbit_reg))
        {
            USART1_change(gt_modbus_reg.baudrate_reg,gt_modbus_reg.stopbit_reg,gt_modbus_reg.checkbit_reg); 
        }
        else if(reg*2 == (uint16_t)(&((T_MODBUS_REG *)0)->stopbit_reg))
        {
            USART1_change(gt_modbus_reg.baudrate_reg,gt_modbus_reg.stopbit_reg,gt_modbus_reg.checkbit_reg); 
        }
        else if(reg*2 == (uint16_t)(&((T_MODBUS_REG *)0)->comm_mode_reg))
        { //需要修改通讯协议模式 
           // USART1_change(gt_modbus_reg.baudrate_reg,gt_modbus_reg.stopbit_reg,gt_modbus_reg.checkbit_reg); 
        }
        else if(reg*2 == (uint16_t)(&((T_MODBUS_REG *)0)->temperature_correct_reg)){
            //进行数据的校准
            //下暂时不用
           
        }
        need_save_reg_data=1;
	}
	else
	{
		gt_modbus.rsp_code = RSP_ERR_REG_ADDR;		/* 寄存器地址错误 */
	}

err_ret:
	if (gt_modbus.rsp_code == RSP_OK)				/* 正确应答 */
	{

		gt_modbus.rx_buf[0] = (gt_modbus_reg.device_addr_reg&0xff);
		modbusSendAckOk();
	}
	else
	{
		modbusSendAckErr(gt_modbus.rsp_code);		/* 告诉主机命令错误 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODS_AnalyzeApp
*	功能说明: 分析应用层协议
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void modbusAnalyzeApp(void)
{
	switch (gt_modbus.rx_buf[1])				/* 第2个字节 功能码 */
	{
		case 0x01:							/* 读取线圈状态 */
			modbus_01_func();
			break;
		case 0x02:							/* 读取输入状态 */
			//MODS_02H();
			break;		
		case 0x03:							/* 读取保持寄存器 ）*/
			modbus_03_func();
			break;		
		case 0x04:							/* 读取输入寄存器 */
			//MODS_04H();
			break;		
		case 0x05:							/* 强制单线圈 */
			//MODS_05H();
			break;		
		case 0x06:							/* 写单个保存寄存器 */
			modbus_06_func();	
			break;			
		case 0x10:							/* 写多个保存寄存器 */
			//MODS_10H();
			break;
		default:
			gt_modbus.rsp_code = RSP_ERR_CMD;
			modbusSendAckErr(gt_modbus.rsp_code);	/* 告诉主机命令错误 */
			break;
	}
}









void modbusInit(){
    start_recv_data = 1;  // 表示可以接收数据
    recv_data_complete=0; //表示接收数据完成
    
    
    gt_modbus_reg.temperature_reg=0x0;
    //为了程序的移植，最好多层封装
    USART1_Init(gt_modbus_reg.baudrate_reg,gt_modbus_reg.stopbit_reg,gt_modbus_reg.checkbit_reg);  //默认9600
   
    
    yd3082_init();    
    yd3082_setinout(1);
    //需要注册回调函数
    registerRecvFunc(modbusReciveData);
    need_update_data=0;
    
  //  TIM14_Init(gt_modbus_reg.upload_time_reg);

}
//表明modbus 是空闲
uint8_t modbus_is_free(void){
    if(recv_data_complete != 0){
        return 0;
    }
    
     if(gt_modbus.rx_count != 0){
        return 0;
    }

    return 1;
    
}


//进行上传 宁
void modbusPoll()
{
	uint16_t addr;
	uint16_t crc1;
	/* 超过3.5个字符时间后执行MODH_RxTimeOut()函数。全局变量 g_modbus_timeout = 1; 通知主程序开始解码 */
	
   if (recv_data_complete == 0)	 //表明没有接收完成  
	{
        if(gt_modbus.rx_count==0){
            if(g_is_active_upload){
               // active_upload();
                g_is_active_upload=0;
            }
        }
		return;								/* 没有超时，继续接收。不要清零 g_tModS.RxCount */
	}
	
	if (gt_modbus.rx_count < 4)				/* 接收到的数据小于4个字节就认为错误 */
	{
        
		goto err_ret;
	}

	/* 计算CRC校验和 */
	crc1 = CRC16_Modbus(gt_modbus.rx_buf, gt_modbus.rx_count);
	if (crc1 != 0){
		goto err_ret;
	}

	/* 站地址 (1字节） */
	addr = gt_modbus.rx_buf[0];				/* 第1字节 站号 */
    
	if ((addr != (gt_modbus_reg.device_addr_reg&0xff))&&(addr != MODBUS_DEVICE_QUERY))		 			/* 判断主机发送的命令地址是否符合 */
	{

		goto err_ret;
	}

	/* 分析应用层协议 */
	modbusAnalyzeApp();						
	
err_ret:
    //中间最好有个停顿
    start_recv_data=1;
    recv_data_complete=0;
 	  gt_modbus.rx_count = 0;					/* 必须清零计数器，方便下次帧同步 */
}


// 插入 实际的温度值
//void insert_temp_correct_value(uint16_t correct_value, uint16_t show_value)
//{
//    //因为值是从小到大排序的 
//    //先寻找下一个无效的地方;
//    int seek_right_pos=-1;
//    uint8_t is_find=0; 
//    int i=0;
//    
//    while(i<MAX_CORRECT_SIZE){
//        if(gt_modbus_reg.temp_value[i].flag==TEMP_VAILD_FLAG){ //先找有没有一样的
//            if(gt_modbus_reg.temp_value[i].real_temp==correct_value){
//                gt_modbus_reg.temp_value[i].offset=correct_value-show_value;
//                return ;  // 已经弄好
//            }
//            if(is_find==0){  //比较难   
//                if(gt_modbus_reg.temp_value[i].real_temp>correct_value){
//                    is_find=1;  
//                    seek_right_pos=i;
//                }
//            }
//        }  
//        i++;
//    }
//   
//    //  
//    if(is_find==1){  //说明找到插入的位置
//        //之后的要往后移动  //那么已经满了就不需要移动了
//        if(i>=MAX_CORRECT_SIZE){
//            gt_modbus_reg.temp_value[seek_right_pos].real_temp=correct_value;
//            gt_modbus_reg.temp_value[seek_right_pos].offset=correct_value-show_value;
//        }
//        else{
//            //都需要往后空一个
//            for(int j=i-1;j>=seek_right_pos;j--){
//                gt_modbus_reg.temp_value[j+1].flag=gt_modbus_reg.temp_value[j].flag;
//                gt_modbus_reg.temp_value[j+1].offset=gt_modbus_reg.temp_value[j].offset;
//                gt_modbus_reg.temp_value[j+1].real_temp=gt_modbus_reg.temp_value[j].real_temp;
//            }
//            gt_modbus_reg.temp_value[seek_right_pos].flag=TEMP_VAILD_FLAG;
//            gt_modbus_reg.temp_value[seek_right_pos].real_temp=correct_value;
//            gt_modbus_reg.temp_value[seek_right_pos].offset=correct_value-show_value;
//        }
//    }
//    else {//如果没有找到、
//        //
//        if(i>=MAX_CORRECT_SIZE){
//            gt_modbus_reg.temp_value[MAX_CORRECT_SIZE-1].real_temp=correct_value;
//            gt_modbus_reg.temp_value[MAX_CORRECT_SIZE-1].offset=correct_value-show_value;
//        }
//    }
//    
//    //得到那个位置是空的.

//}

//uint16_t get_temp_correct_value(uint16_t real_temp){
//    //先判断超过那个显示值
//   uint16_t show_temp=real_temp;
//   int i=0;
//   while(i<MAX_CORRECT_SIZE){
//       if(gt_modbus_reg.temp_value[i].real_temp>real_temp){
//            //表示已经找到;
//          i--;
//          break;
//       }
//       i++;
//   }

//   if(i>=0){
//       //先折算出比率;
//       int next_i=i+1;
//      double rate=(double)(gt_modbus_reg.temp_value[next_i].show_temp-gt_modbus_reg.temp_value[i].show_temp)      \
//                                /(double)(gt_modbus_reg.temp_value[next_i].real_temp-gt_modbus_reg.temp_value[i].real_temp);
//       //得到号
//        real_temp+=(real_temp-gt_modbus_reg.temp_value[i].real_temp)*rate;
//        real_temp+=gt_modbus_reg.temp_value[i].offset;
//   }
//   else{
//       return show_temp;
//   }
//   
//   
//}














