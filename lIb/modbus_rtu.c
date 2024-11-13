#include "modbus_rtu.h"
#include "string.h"

#include "sht40.h"
#include "bmp280.h"
#include "bh1750.h"







/*
*********************************************************************************************************
*	�� �� ��: MODS_Poll
*	����˵��: �������ݰ�. �����������������á�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/

//������ڽ��յ�״̬�У��ǲ��ܷ������ݵ�

uint16_t need_update_data ;  //��Ҫ������Щ����

uint8_t need_save_reg_data ;  //��Ҫ��������


uint8_t start_recv_data ;  // ��ʾ���Խ�������
uint8_t recv_data_complete; //��ʾ�����������

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
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0; //������ȼ�Ҫ�͵�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM_TimeBaseStructure.TIM_Period = 1000*second;  // �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ)   50
  TIM_TimeBaseStructure.TIM_Prescaler = (1000 - 1);	//ʱ��Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);			        // �������жϱ�־ 
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  
  if(gt_modbus_reg.comm_mode_reg==0x2){
     // TIM_Cmd(TIM3, ENABLE);
  }
  else{
     // TIM_Cmd(TIM3, DISABLE);
  }
  TIM_SetCounter(TIM3,0);//�����ʱ
  
  
 
}

void change_TIM14_fre(int second){
    
  TIM_Cmd(TIM3, DISABLE);
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 1000*second;  // �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ)   50
  TIM_TimeBaseStructure.TIM_Prescaler = (1000 - 1);	//ʱ��Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;


  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);			        // �������жϱ�־ 
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  
  if(gt_modbus_reg.comm_mode_reg==0x2){
     // TIM_Cmd(TIM3, ENABLE);
  }
  else{
     // TIM_Cmd(TIM3, DISABLE);
  } 
  
  TIM_SetCounter(TIM3,0);//�����ʱ
}


uint8_t  g_is_active_upload=0;
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
      TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update);
     // TIM_Cmd(TIM14, ENABLE);
       //�����Ҫ�����ϴ�����
      
      
      
      g_is_active_upload=1;
            
  }
}




// CRC ��λ�ֽ�ֵ��
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
// CRC ��λ�ֽ�ֵ��
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
*	�� �� ��: CRC16_Modbus
*	����˵��: ����CRC�� ����ModbusЭ�顣
*	��    ��: _pBuf : ����У�������
*			  _usLen : ���ݳ���
*	�� �� ֵ: 16λ����ֵ�� ����Modbus ���˽�����ֽ��ȴ��ͣ����ֽں��͡�
*
*   ���п��ܵ�CRCֵ����Ԥװ���������鵱�У������㱨������ʱ���Լ򵥵��������ɣ�
*   һ�����������16λCRC�������256�����ܵĸ�λ�ֽڣ���һ�����麬�е�λ�ֽڵ�ֵ��
*   ������������CRC�ķ�ʽ�ṩ�˱ȶԱ��Ļ�������ÿһ�����ַ��������µ�CRC����ķ�����
*
*  ע�⣺�˳����ڲ�ִ�и�/��CRC�ֽڵĽ������˺������ص����Ѿ�����������CRCֵ��Ҳ����˵���ú����ķ���ֵ����ֱ�ӷ���
*        �ڱ������ڷ��ͣ�
*********************************************************************************************************
*/
uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen)
{
	uint8_t ucCRCHi = 0xFF; /* ��CRC�ֽڳ�ʼ�� */
	uint8_t ucCRCLo = 0xFF; /* ��CRC �ֽڳ�ʼ�� */
	uint16_t usIndex;  /* CRCѭ���е����� */

  while (_usLen--)
  {
      usIndex = ucCRCHi ^ *_pBuf++; /* ����CRC */
      ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
      ucCRCLo = s_CRCLo[usIndex];
  }
  return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

/*
*********************************************************************************************************
*	�� �� ��: MODS_ReadRegValue
*	����˵��: ��ȡ���ּĴ�����ֵ
*	��    ��: reg_addr �Ĵ�����ַ
*			  reg_value ��żĴ������
*	�� �� ֵ: 1��ʾOK 0��ʾ����
*********************************************************************************************************
*/


static uint8_t modbusReadRegValue(uint16_t reg_addr, uint8_t *reg_value)
{
	uint16_t value;
    
    if(reg_addr<=0x72){
        
        if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->temperature_reg)){
            //�����ʾ��Ҫ��ȡ�¶�,��ô��Ҫ�ɼ�һ��
            need_update_data|=UPDATE_TEMP_DATA;
        }
        
        if(reg_addr*2==(uint16_t)(&((T_MODBUS_REG *)0)->humidity_reg)){
            //�����ʾ��Ҫ��ȡ�¶�,��ô��Ҫ�ɼ�һ��
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
    }									/* ��ȡ�ɹ� */
}

/*
*********************************************************************************************************
*	�� �� ��: modbusWriteRegValue
*	����˵��: ��ȡ���ּĴ�����ֵ
*	��    ��: reg_addr �Ĵ�����ַ
*			  reg_value �Ĵ���ֵ
*	�� �� ֵ: 1��ʾOK 0��ʾ����
*********************************************************************************************************
*/
static uint8_t modbusWriteRegValue(uint16_t reg_addr, int16_t reg_value)
{
    //�����޸�
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
    else{  //�������0xff00 
       
       return 0;
    }
 
}




/*
*********************************************************************************************************
*	�� �� ��: BEBufToUint16
*	����˵��: ��2�ֽ�����(���Big Endian���򣬸��ֽ���ǰ)ת��Ϊ16λ����
*	��    ��: _pBuf : ����
*	�� �� ֵ: 16λ����ֵ
*
*   ���(Big Endian)��С��(Little Endian)
*********************************************************************************************************
*/
uint16_t BEBufToUint16(uint8_t *_pBuf)
{
    return (((uint16_t)_pBuf[0] << 8) | _pBuf[1]);
}



/*
*********************************************************************************************************
*	�� �� ��: MODS_SendWithCRC
*	����˵��: ����һ������, �Զ�׷��2�ֽ�CRC
*	��    ��: _pBuf ���ݣ�
*			  _ucLen ���ݳ��ȣ�����CRC��
*	�� �� ֵ: ��
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
    
    //������ڲ�æ�Ļ����Ϳ��Է���
    
    yd3082_setinout(0);
    USART1_SENDDATA(buf,_ucLen);
    yd3082_setinout(1);

}
/*
*********************************************************************************************************
*	�� �� ��: MODS_SendAckErr
*	����˵��: ���ʹ���Ӧ��
*	��    ��: _ucErrCode : �������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void modbusSendAckErr(uint8_t _ucErrCode)
{
	uint8_t txbuf[3];

	txbuf[0] = gt_modbus.rx_buf[0];					/* 485��ַ */
	txbuf[1] = gt_modbus.rx_buf[1] | 0x80;				/* �쳣�Ĺ����� */
	txbuf[2] = _ucErrCode;							/* �������(01,02,03,04) */

	modbusSendWithCRC(txbuf, 3);
}


/*
�����Ҫ�ŵ��жϺ����� 
*/
void modbusReciveData(uint8_t byte)
{
	/*
		3.5���ַ���ʱ������ֻ������RTUģʽ���棬��ΪRTUģʽû�п�ʼ���ͽ�������
		�������ݰ�֮��ֻ�ܿ�ʱ���������֣�Modbus�����ڲ�ͬ�Ĳ������£����ʱ���ǲ�һ���ģ�
		���Ծ���3.5���ַ���ʱ�䣬�����ʸߣ����ʱ������С�������ʵͣ����ʱ������Ӧ�ʹ�

		4800  = 7.297ms
		9600  = 3.646ms
		19200  = 1.771ms
		38400  = 0.885ms
	*/
    if(start_recv_data>0){
        //��ʾ�Ѿ���ʱ,��Ҫ�жϵ�һ���ַ�
        if(((gt_modbus_reg.device_addr_reg & 0xff)==byte)||(MODBUS_DEVICE_QUERY==byte)){  //�Ǿ�˵��������Ч
            start_recv_data=0;//�������ʱ���
            gt_modbus.rx_count=0; //��ʼ��������
            recv_data_complete=0;//���ݽ��տ�ʼ
            gt_modbus.rx_buf[gt_modbus.rx_count++] = byte;
        }
    }
    else if(recv_data_complete==0){
        if (gt_modbus.rx_count < S_RX_BUF_SIZE)
        {
            gt_modbus.rx_buf[gt_modbus.rx_count++] = byte;
        }
    }
	
    //��������Ҫ�������ö�ʱ��
}


/*
*********************************************************************************************************
*	�� �� ��: MODS_01H
*	����˵��: ��ȡ��Ȧ״̬����ӦԶ�̿���D01/D02/D03��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void modbus_01_func(void)
{
	/*
	 ������
		��������:
			11 �ӻ���ַ
			01 ������
			00 �Ĵ�����ʼ��ַ���ֽ�
			13 �Ĵ�����ʼ��ַ���ֽ�
			00 �Ĵ����������ֽ�
			25 �Ĵ����������ֽ�
			0E CRCУ����ֽ�
			84 CRCУ����ֽ�

		�ӻ�Ӧ��: 	1����ON��0����OFF�������ص���Ȧ����Ϊ8�ı�����������������ֽ�δβʹ��0����. BIT0��Ӧ��1��
			11 �ӻ���ַ
			01 ������
			05 �����ֽ���
			CD ����1(��Ȧ0013H-��Ȧ001AH)
			6B ����2(��Ȧ001BH-��Ȧ0022H)
			B2 ����3(��Ȧ0023H-��Ȧ002AH)
			0E ����4(��Ȧ0032H-��Ȧ002BH)
			1B ����5(��Ȧ0037H-��Ȧ0033H)
			45 CRCУ����ֽ�
			E6 CRCУ����ֽ�

		����:
			01 01 10 01 00 03   29 0B	--- ��ѯD01��ʼ��3���̵���״̬
			01 01 10 03 00 01   09 0A   --- ��ѯD03�̵�����״̬
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint16_t m;
	uint8_t status[10];
	
	gt_modbus.rsp_code = RSP_OK;

	/* û���ⲿ�̵�����ֱ��Ӧ����� */
	if (gt_modbus.rx_count != 8)
	{
		gt_modbus.rsp_code = RSP_ERR_VALUE;				/* ����ֵ����� */
		return;
	}

	reg = BEBufToUint16(&gt_modbus.rx_buf[2]); 			/* �Ĵ����� */
	num = BEBufToUint16(&gt_modbus.rx_buf[4]);				/* �Ĵ������� */

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
		gt_modbus.rsp_code = RSP_ERR_REG_ADDR;				/* �Ĵ�����ַ���� */
	}

	if (gt_modbus.rsp_code == RSP_OK)						/* ��ȷӦ�� */
	{
		gt_modbus.tx_count = 0;
		gt_modbus.tx_buf[gt_modbus.tx_count++] = gt_modbus.rx_buf[0];
		gt_modbus.tx_buf[gt_modbus.tx_count++] = gt_modbus.rx_buf[1];
		gt_modbus.tx_buf[gt_modbus.tx_count++] = m;			/* �����ֽ��� */

		for (i = 0; i < m; i++)
		{
			gt_modbus.tx_buf[gt_modbus.tx_count++] = status[i];	/* �̵���״̬ */
		}
		modbusSendWithCRC(gt_modbus.tx_buf, gt_modbus.tx_count);
	}
	else
	{
		modbusSendAckErr(gt_modbus.rsp_code);				/* ��������������� */
	}
}


/*
*********************************************************************************************************
*	�� �� ��: MODS_03H
*	����˵��: ��ȡ���ּĴ��� ��һ���������ּĴ�����ȡ�õ�ǰ�Ķ�����ֵ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void modbus_03_func(void)
{
	/*
		�ӻ���ַΪ11H�����ּĴ�������ʼ��ַΪ006BH��������ַΪ006DH���ôβ�ѯ�ܹ�����3�����ּĴ�����

		��������:
			11 �ӻ���ַ
			03 ������
			00 �Ĵ�����ַ���ֽ�
			6B �Ĵ�����ַ���ֽ�
			00 �Ĵ����������ֽ�
			03 �Ĵ����������ֽ�
			76 CRC���ֽ�
			87 CRC���ֽ�

		�ӻ�Ӧ��: 	���ּĴ����ĳ���Ϊ2���ֽڡ����ڵ������ּĴ������ԣ��Ĵ������ֽ������ȱ����䣬
					���ֽ����ݺ󱻴��䡣���ּĴ���֮�䣬�͵�ַ�Ĵ����ȱ����䣬�ߵ�ַ�Ĵ����󱻴��䡣
			11 �ӻ���ַ
			03 ������
			06 �ֽ���
			00 ����1���ֽ�(006BH)
			6B ����1���ֽ�(006BH)
			00 ����2���ֽ�(006CH)
			13 ����2 ���ֽ�(006CH)
			00 ����3���ֽ�(006DH)
			00 ����3���ֽ�(006DH)
			38 CRC���ֽ�
			B9 CRC���ֽ�

		����:
			01 03 30 06 00 01  6B0B      ---- �� 3006H, ��������
			01 03 4000 0010 51C6         ---- �� 4000H ������1����ӿ��¼ 32�ֽ�
			01 03 4001 0010 0006         ---- �� 4001H ������1����ӿ��¼ 32�ֽ�

			01 03 F000 0008 770C         ---- �� F000H ������1���澯��¼ 16�ֽ�
			01 03 F001 0008 26CC         ---- �� F001H ������2���澯��¼ 16�ֽ�

			01 03 7000 0020 5ED2         ---- �� 7000H ������1�����μ�¼��1�� 64�ֽ�
			01 03 7001 0020 0F12         ---- �� 7001H ������1�����μ�¼��2�� 64�ֽ�

			01 03 7040 0020 5F06         ---- �� 7040H ������2�����μ�¼��1�� 64�ֽ�
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[64];
    uint8_t addr;

	gt_modbus.rsp_code = RSP_OK;

	if (gt_modbus.rx_count != 8)								/* 03H���������8���ֽ� */
	{
		gt_modbus.rsp_code = RSP_ERR_VALUE;					/* ����ֵ����� */
		goto err_ret;
	}

    addr=gt_modbus.rx_buf[0];
	reg = BEBufToUint16(&gt_modbus.rx_buf[2]); 				/* �Ĵ����� */
	num = BEBufToUint16(&gt_modbus.rx_buf[4]); //��Ҫ��ȡ������					/* �Ĵ������� */
	if (num > sizeof(reg_value) / 2)  //���ܶ�ȡȫ���Ĵ�����
	{
		gt_modbus.rsp_code = RSP_ERR_VALUE;					/* ����ֵ����� */
		goto err_ret;
	}

	for (i = 0; i < num; i++)
	{
		if (modbusReadRegValue(reg, &reg_value[2 * i]) == 0)	/* �����Ĵ���ֵ����reg_value */
		{
			gt_modbus.rsp_code = RSP_ERR_REG_ADDR;				/* �Ĵ�����ַ���� */
			break;
		}
		reg++;
	}

err_ret:
	if (gt_modbus.rsp_code == RSP_OK)							/* ��ȷӦ�� */
	{
		gt_modbus.tx_count = 0;
		gt_modbus.tx_buf[gt_modbus.tx_count++] = (gt_modbus_reg.device_addr_reg&0xff);
		gt_modbus.tx_buf[gt_modbus.tx_count++] = gt_modbus.rx_buf[1];
		gt_modbus.tx_buf[gt_modbus.tx_count++] = num * 2;			/* �����ֽ��� */

		for (i = 0; i < num; i++)
		{
			gt_modbus.tx_buf[gt_modbus.tx_count++] = reg_value[2*i];
			gt_modbus.tx_buf[gt_modbus.tx_count++] = reg_value[2*i+1];
		}
		modbusSendWithCRC(gt_modbus.tx_buf, gt_modbus.tx_count);	/* ������ȷӦ�� */
	}
	else
	{
		modbusSendAckErr(gt_modbus.rsp_code);					/* ���ʹ���Ӧ�� */
	}
}


//���Ϊ0,ֱ��д0.00

int int_convert_ascii(int reg,uint8_t* buf,int point){
    //����ֵΪ65535 Ҳ��  100000
    
    uint8_t * p_buf= (uint8_t *)buf;
    int max_rate=10000;
    int buf_len=0;
    uint8_t is_start=0;
    //һ����Ҫ�������С����
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
    
    //Ȼ����Ҫ�ɼ��������Լ��ɼ���                    
     gt_modbus.tx_buf[gt_modbus.tx_count++] = 34 ; // "   
     gt_modbus.tx_buf[gt_modbus.tx_count++] = 125; // }
    
     gt_modbus.tx_buf[2] = 0x3; 
     
     modbusSendWithCRC(gt_modbus.tx_buf, gt_modbus.tx_count);	/* ������ȷӦ�� */

            //�����ʾ��Ҫ��ȡ�¶�,��ô��Ҫ�ɼ�һ��
      need_update_data|=UPDATE_TEMP_DATA;

            //�����ʾ��Ҫ��ȡ�¶�,��ô��Ҫ�ɼ�һ��
      need_update_data|=UPDATE_HUMIDITY_DATA;

      need_update_data|=UPDATE_GASSPRESSURE_DATA;

      need_update_data|=UPDATE_SUNLIGHT_DATA;

}



/*
*********************************************************************************************************
*	�� �� ��: MODS_SendAckOk
*	����˵��: ������ȷ��Ӧ��.
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: MODS_06H
*	����˵��: ��ȡ���ּĴ��� ��һ���������ּĴ�����ȡ�õ�ǰ�Ķ�����ֵ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void modbus_06_func(void)
{

/*
		д���ּĴ�����ע��06ָ��ֻ�ܲ����������ּĴ�����16ָ��������õ����������ּĴ���
		��������:
			11 �ӻ���ַ
			06 ������
			00 �Ĵ�����ַ���ֽ�
			01 �Ĵ�����ַ���ֽ�
			00 ����1���ֽ�
			01 ����1���ֽ�
			9A CRCУ����ֽ�
			9B CRCУ����ֽ�

		�ӻ���Ӧ:
			11 �ӻ���ַ
			06 ������
			00 �Ĵ�����ַ���ֽ�
			01 �Ĵ�����ַ���ֽ�
			00 ����1���ֽ�
			01 ����1���ֽ�
			1B CRCУ����ֽ�
			5A	CRCУ����ֽ�

		����:
			01 06 30 06 00 25  A710    ---- ������������Ϊ 2.5
			01 06 30 06 00 10  6707    ---- ������������Ϊ 1.0


			01 06 30 1B 00 00  F6CD    ---- SMA �˲�ϵ�� = 0 �ر��˲�
			01 06 30 1B 00 01  370D    ---- SMA �˲�ϵ�� = 1
			01 06 30 1B 00 02  770C    ---- SMA �˲�ϵ�� = 2
			01 06 30 1B 00 05  36CE    ---- SMA �˲�ϵ�� = 5

			01 06 30 07 00 01  F6CB    ---- ����ģʽ�޸�Ϊ T1
			01 06 30 07 00 02  B6CA    ---- ����ģʽ�޸�Ϊ T2

			01 06 31 00 00 00  8736    ---- ������ӿ��¼��
			01 06 31 01 00 00  D6F6    ---- �����澯��¼��

*/

	uint16_t reg;
	uint16_t value;

	gt_modbus.rsp_code = RSP_OK;

	if(gt_modbus.rx_count != 8)
	{
		gt_modbus.rsp_code = RSP_ERR_VALUE;		/* ����ֵ����� */
		goto err_ret;
	}

	reg = BEBufToUint16(&gt_modbus.rx_buf[2]); 	/* �Ĵ����� */
	value = BEBufToUint16(&gt_modbus.rx_buf[4]);	/* �Ĵ���ֵ */

	if (modbusWriteRegValue(reg, (int16_t)value) == 1)	/* �ú������д���ֵ����Ĵ��� */
	{
        //��Ҫ�ж�ֵ��û������ ��������Ҫ�޸���
        
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
        { //��Ҫ�޸�ͨѶЭ��ģʽ 
           // USART1_change(gt_modbus_reg.baudrate_reg,gt_modbus_reg.stopbit_reg,gt_modbus_reg.checkbit_reg); 
        }
        else if(reg*2 == (uint16_t)(&((T_MODBUS_REG *)0)->temperature_correct_reg)){
            //�������ݵ�У׼
            //����ʱ����
           
        }
        need_save_reg_data=1;
	}
	else
	{
		gt_modbus.rsp_code = RSP_ERR_REG_ADDR;		/* �Ĵ�����ַ���� */
	}

err_ret:
	if (gt_modbus.rsp_code == RSP_OK)				/* ��ȷӦ�� */
	{

		gt_modbus.rx_buf[0] = (gt_modbus_reg.device_addr_reg&0xff);
		modbusSendAckOk();
	}
	else
	{
		modbusSendAckErr(gt_modbus.rsp_code);		/* ��������������� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODS_AnalyzeApp
*	����˵��: ����Ӧ�ò�Э��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void modbusAnalyzeApp(void)
{
	switch (gt_modbus.rx_buf[1])				/* ��2���ֽ� ������ */
	{
		case 0x01:							/* ��ȡ��Ȧ״̬ */
			modbus_01_func();
			break;
		case 0x02:							/* ��ȡ����״̬ */
			//MODS_02H();
			break;		
		case 0x03:							/* ��ȡ���ּĴ��� ��*/
			modbus_03_func();
			break;		
		case 0x04:							/* ��ȡ����Ĵ��� */
			//MODS_04H();
			break;		
		case 0x05:							/* ǿ�Ƶ���Ȧ */
			//MODS_05H();
			break;		
		case 0x06:							/* д��������Ĵ��� */
			modbus_06_func();	
			break;			
		case 0x10:							/* д�������Ĵ��� */
			//MODS_10H();
			break;
		default:
			gt_modbus.rsp_code = RSP_ERR_CMD;
			modbusSendAckErr(gt_modbus.rsp_code);	/* ��������������� */
			break;
	}
}









void modbusInit(){
    start_recv_data = 1;  // ��ʾ���Խ�������
    recv_data_complete=0; //��ʾ�����������
    
    
    gt_modbus_reg.temperature_reg=0x0;
    //Ϊ�˳������ֲ����ö���װ
    USART1_Init(gt_modbus_reg.baudrate_reg,gt_modbus_reg.stopbit_reg,gt_modbus_reg.checkbit_reg);  //Ĭ��9600
   
    
    yd3082_init();    
    yd3082_setinout(1);
    //��Ҫע��ص�����
    registerRecvFunc(modbusReciveData);
    need_update_data=0;
    
  //  TIM14_Init(gt_modbus_reg.upload_time_reg);

}
//����modbus �ǿ���
uint8_t modbus_is_free(void){
    if(recv_data_complete != 0){
        return 0;
    }
    
     if(gt_modbus.rx_count != 0){
        return 0;
    }

    return 1;
    
}


//�����ϴ� ��
void modbusPoll()
{
	uint16_t addr;
	uint16_t crc1;
	/* ����3.5���ַ�ʱ���ִ��MODH_RxTimeOut()������ȫ�ֱ��� g_modbus_timeout = 1; ֪ͨ������ʼ���� */
	
   if (recv_data_complete == 0)	 //����û�н������  
	{
        if(gt_modbus.rx_count==0){
            if(g_is_active_upload){
               // active_upload();
                g_is_active_upload=0;
            }
        }
		return;								/* û�г�ʱ���������ա���Ҫ���� g_tModS.RxCount */
	}
	
	if (gt_modbus.rx_count < 4)				/* ���յ�������С��4���ֽھ���Ϊ���� */
	{
        
		goto err_ret;
	}

	/* ����CRCУ��� */
	crc1 = CRC16_Modbus(gt_modbus.rx_buf, gt_modbus.rx_count);
	if (crc1 != 0){
		goto err_ret;
	}

	/* վ��ַ (1�ֽڣ� */
	addr = gt_modbus.rx_buf[0];				/* ��1�ֽ� վ�� */
    
	if ((addr != (gt_modbus_reg.device_addr_reg&0xff))&&(addr != MODBUS_DEVICE_QUERY))		 			/* �ж��������͵������ַ�Ƿ���� */
	{

		goto err_ret;
	}

	/* ����Ӧ�ò�Э�� */
	modbusAnalyzeApp();						
	
err_ret:
    //�м�����и�ͣ��
    start_recv_data=1;
    recv_data_complete=0;
 	  gt_modbus.rx_count = 0;					/* ��������������������´�֡ͬ�� */
}


// ���� ʵ�ʵ��¶�ֵ
//void insert_temp_correct_value(uint16_t correct_value, uint16_t show_value)
//{
//    //��Ϊֵ�Ǵ�С��������� 
//    //��Ѱ����һ����Ч�ĵط�;
//    int seek_right_pos=-1;
//    uint8_t is_find=0; 
//    int i=0;
//    
//    while(i<MAX_CORRECT_SIZE){
//        if(gt_modbus_reg.temp_value[i].flag==TEMP_VAILD_FLAG){ //������û��һ����
//            if(gt_modbus_reg.temp_value[i].real_temp==correct_value){
//                gt_modbus_reg.temp_value[i].offset=correct_value-show_value;
//                return ;  // �Ѿ�Ū��
//            }
//            if(is_find==0){  //�Ƚ���   
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
//    if(is_find==1){  //˵���ҵ������λ��
//        //֮���Ҫ�����ƶ�  //��ô�Ѿ����˾Ͳ���Ҫ�ƶ���
//        if(i>=MAX_CORRECT_SIZE){
//            gt_modbus_reg.temp_value[seek_right_pos].real_temp=correct_value;
//            gt_modbus_reg.temp_value[seek_right_pos].offset=correct_value-show_value;
//        }
//        else{
//            //����Ҫ�����һ��
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
//    else {//���û���ҵ���
//        //
//        if(i>=MAX_CORRECT_SIZE){
//            gt_modbus_reg.temp_value[MAX_CORRECT_SIZE-1].real_temp=correct_value;
//            gt_modbus_reg.temp_value[MAX_CORRECT_SIZE-1].offset=correct_value-show_value;
//        }
//    }
//    
//    //�õ��Ǹ�λ���ǿյ�.

//}

//uint16_t get_temp_correct_value(uint16_t real_temp){
//    //���жϳ����Ǹ���ʾֵ
//   uint16_t show_temp=real_temp;
//   int i=0;
//   while(i<MAX_CORRECT_SIZE){
//       if(gt_modbus_reg.temp_value[i].real_temp>real_temp){
//            //��ʾ�Ѿ��ҵ�;
//          i--;
//          break;
//       }
//       i++;
//   }

//   if(i>=0){
//       //�����������;
//       int next_i=i+1;
//      double rate=(double)(gt_modbus_reg.temp_value[next_i].show_temp-gt_modbus_reg.temp_value[i].show_temp)      \
//                                /(double)(gt_modbus_reg.temp_value[next_i].real_temp-gt_modbus_reg.temp_value[i].real_temp);
//       //�õ���
//        real_temp+=(real_temp-gt_modbus_reg.temp_value[i].real_temp)*rate;
//        real_temp+=gt_modbus_reg.temp_value[i].offset;
//   }
//   else{
//       return show_temp;
//   }
//   
//   
//}














