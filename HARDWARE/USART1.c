#include "USART1.h"
#include "modbus_rtu.h"
extern  T_MODBUS_REG gt_modbus_reg;


uart_recv_data recvdata;


extern uint8_t start_recv_data ;  // ��ʾ���Խ�������
extern uint8_t recv_data_complete; //��ʾ�����������




//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{  
	while(!((USART1->ISR)&(1<<7))){}
  USART1->TDR=ch;
  return (ch);
}
#endif 






void TIM_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1; //��ʱ�������ȼ���Զ��Ըߵ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 

  TIM_TimeBaseStructure.TIM_Period = 100;  // �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ)   50
  TIM_TimeBaseStructure.TIM_Prescaler = (800 - 1);	//ʱ��Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

  TIM_ClearFlag(TIM14, TIM_FLAG_Update);			        // �������жϱ�־ 
  TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM14, DISABLE);
}



static uint8_t recv_data_time_out=0;

static uint8_t recv_data_busy=0;
void TIM14_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
  {
        TIM_ClearITPendingBit(TIM14,TIM_FLAG_Update);
        start_recv_data=1;
        recv_data_complete=1;
        recv_data_busy=1;//����æ
  }
}


/* USART��ʼ�� */
static  int16_t TIMER_LOAD_VALUE=0;
///
void USART1_Init(uint16_t baud,uint16_t stop_bit,uint16_t check_bit)
{
	
  TIM_Init();
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  //ʹ��GPIOA��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//ʹ��USART��ʱ��
	/* USART1�Ķ˿����� */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//����PA9�ɵڶ���������	TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);//����PA10�ɵڶ���������  RX	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    uint32_t temp_baud;

    switch(baud)
    {

        case 1:
            temp_baud=2400;
						TIMER_LOAD_VALUE=0;
            break;
        case 2:
            temp_baud=4800;
						TIMER_LOAD_VALUE=20;
            break;
        case 3:
            temp_baud=9600;
						TIMER_LOAD_VALUE=60;
            break;
        case 4:
            temp_baud=19200;
						TIMER_LOAD_VALUE=80;
            break;
        case 5:
            temp_baud=38400;
						TIMER_LOAD_VALUE=90;
            break;
        case 6:
            temp_baud=115200;
						TIMER_LOAD_VALUE=98;
            break;
        deafult:
            temp_baud=9600;
					  TIMER_LOAD_VALUE=0;
            break;
    }
    uint32_t temp_stopbit;
    switch(stop_bit)
    {
        case 1:
            temp_stopbit=USART_StopBits_1;
            break;
        case 2:
            temp_stopbit=USART_StopBits_1_5;
            break;
        case 3:
            temp_stopbit=USART_StopBits_2;
            break;
        default:
           temp_stopbit=USART_StopBits_1;
           break;        
    }
    uint32_t temp_checkbit;
    
    switch(check_bit)
    {
        case 1:
            temp_checkbit=USART_Parity_No;
            break;
        case 2:
            temp_checkbit=USART_Parity_Odd;
            break;
        case 3:
            temp_checkbit=USART_Parity_Even;
            break;
        default:
            temp_checkbit=USART_Parity_No;
            break;        
    }
    

	/* USART1�Ļ������� */
	USART_InitStructure.USART_BaudRate = temp_baud;              //������
    if(temp_checkbit==USART_Parity_No){
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }
    else{
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }
	
	USART_InitStructure.USART_StopBits = temp_stopbit;
	USART_InitStructure.USART_Parity = temp_checkbit;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);		
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);           //ʹ�ܽ����ж�
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //��������ж�
	USART_Cmd(USART1, ENABLE);                             //ʹ��USART1
	
	/* USART1��NVIC�ж����� */
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0x02;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);		
}

void USART1_change(uint16_t baud,uint16_t stop_bit,uint16_t check_bit)
{
    	/* USART1�Ļ������� */
    USART_InitTypeDef USART_InitStructure;
    
    uint32_t temp_baud;

    switch(baud)
    {

        case 1:
            temp_baud=2400;
						TIMER_LOAD_VALUE=0;
            break;
        case 2:
            temp_baud=4800;
						TIMER_LOAD_VALUE=20;
            break;
        case 3:
            temp_baud=9600;
						TIMER_LOAD_VALUE=60;
            break;
        case 4:
            temp_baud=19200;
						TIMER_LOAD_VALUE=80;
            break;
        case 5:
            temp_baud=38400;
						TIMER_LOAD_VALUE=90;
            break;
        case 6:
            temp_baud=115200;
						TIMER_LOAD_VALUE=98;
            break;
        deafult:
            temp_baud=9600;
					  TIMER_LOAD_VALUE=0;
            break;
    }
    uint32_t temp_stopbit;
    switch(stop_bit)
    {
        case 1:
            temp_stopbit=USART_StopBits_1;
            break;
        case 2:
            temp_stopbit=USART_StopBits_1_5;
            break;
        case 3:
            temp_stopbit=USART_StopBits_2;
            break;
        default:
           temp_stopbit=USART_StopBits_1;
            break;        
    }
    uint32_t temp_checkbit;
    
    switch(check_bit)
    {
        case 1:
            temp_checkbit=USART_Parity_No;
            break;
        case 2:
            temp_checkbit=USART_Parity_Even;
            break;
        case 3:
            temp_checkbit=USART_Parity_Odd;
            break;
        default:
           temp_checkbit=USART_Parity_No;
            break;        
    }
    

	/* USART1�Ļ������� */
	USART_InitStructure.USART_BaudRate = temp_baud;              //������
    if(temp_checkbit==USART_Parity_No){
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }
    else{
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }
	USART_InitStructure.USART_StopBits = temp_stopbit;
	USART_InitStructure.USART_Parity = temp_checkbit;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);		
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);           //ʹ�ܽ����ж�
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //��������ж�
	USART_Cmd(USART1, ENABLE);   

}

void registerRecvFunc(uart_recv_data func)
{
    recvdata=func;
}

//=============================================================================
//�ļ����ƣ�
//���ܸ�Ҫ��USART1�жϺ���
//����˵������
//�������أ���
//=============================================================================
void USART1_IRQHandler(void)
{
    //
    uint8_t recv_value;  
    if ( USART_GetITStatus( USART1, USART_IT_IDLE ) != RESET )
    {
        USART_ClearITPendingBit(USART1,USART_IT_IDLE);
       //�����ʾȡ��һ��һ֡����
        recv_value=USART_ReceiveData(USART1);	
		  	//recvdata(recv_value);
    }   
     
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
        
       USART_ClearITPendingBit(USART1,USART_IT_RXNE);
        
       TIM_Cmd(TIM14, DISABLE);  //�رն�ʱ��
	   TIM_SetCounter(TIM14,TIMER_LOAD_VALUE);//�����ʱ
        recv_value=USART_ReceiveData(USART1);	//��ȡ���յ�������
        TIM_Cmd(TIM14,ENABLE);  //�رն�ʱ��
        recvdata(recv_value);
        //��ʱ����ֲʹ�ܵ�
        
        
        
        //���յ�������Ҫ�ŵ���������
        
       // USART_SendData(USART1,USART_ReceiveData(USART1));
        // while (USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
    {
        USART_ClearITPendingBit(USART1,USART_IT_ORE);
        recv_value=USART_ReceiveData(USART1);//һ��Ҫ��
    }
			
}


void USART1_SENDDATA(uint8_t *buf, uint8_t len)
{
	uint8_t t;
    uint8_t temp_buf[128];
//    len=14;
//    temp_buf[0]=0x01;
//    temp_buf[1]=0x03;
//    temp_buf[2]=0x00;
//    temp_buf[3]=0x01;
//    temp_buf[4]=0x02;
//    temp_buf[5]=0x03;
//    temp_buf[6]=0x04;
//    temp_buf[7]=0x05;
//    temp_buf[8]=0x06;
//    temp_buf[9]=0x07;
//    temp_buf[10]=0x08;
//    temp_buf[11]=0x09;
//    temp_buf[12]=0x22;
//    temp_buf[13]=0x33;
    

	for(t=0;t<len;t++)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		USART_SendData(USART1,buf[t]);
	}
	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
}




void uart_exti_int(void)
{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    USART_InitTypeDef USART_InitStruct;
//    NVIC_InitTypeDef NVIC_InitStructure;
//    EXTI_InitTypeDef EXTI_InitStruct;
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//����ģʽ
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource10);
//    EXTI_InitStruct.EXTI_Line = EXTI_Line10;
//    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising; //�������ж�
//    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStruct);

}


//void EXTI4_15_IRQHandler(void)
//{
//    if(EXTI_GetITStatus(EXTI_Line10) != RESET)
//    {
//        EXTI_ClearITPendingBit(EXTI_Line7);
//        NVIC_SystemReset();//��λ����
//        
//    }
//}


