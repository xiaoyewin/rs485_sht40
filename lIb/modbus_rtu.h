#ifndef _MODBUS_RTU_H_
#define _MODBUS_RTU_H_
#include <stdint.h>

#include "USART1.h"  //这个底层串口的接口
#include "yd3082.h"  


#define MODBUS_DEVICE_ID 0x01 //设备的Id为0x01

//型号的ID
#define MODBUS_MODEL  1002 //设备的Id为固定 

#define MODBUS_DEVICE_QUERY 0xfa //设备的Id为0x01


#define S_RX_BUF_SIZE		    30
#define S_TX_BUF_SIZE		   128


/* RTU 应答代码 */
#define RSP_OK				       0		/* 成功 */
#define RSP_ERR_CMD			  0x01	/* 不支持的功能码 */
#define RSP_ERR_REG_ADDR	0x02	/* 寄存器地址错误 */
#define RSP_ERR_VALUE		  0x03	/* 数据值域错误 */
#define RSP_ERR_WRITE		  0x04	/* 写入失败 */


/* 01H 读强制单线圈(继电器) */
/* 05H 写强制单线圈(继电器) */
#define REG_D01		0x0101
#define REG_D02		0x0102
#define REG_D03		0x0103
#define REG_D04		0x0104
#define REG_DXX 	REG_D04

/* 02H 读取输入状态(控制按钮)  */
#define REG_T01		0x0201
#define REG_T02		0x0202
#define REG_TXX		REG_T02

/* 03H 读保持寄存器(内部寄存器) */
/* 06H 写保持寄存器(内部寄存器) */
/* 10H 写多个保存寄存器(内部寄存器) */
#define SLAVE_REG_P01		0x0301
#define SLAVE_REG_P02		0x0302

/* 04H 读取输入寄存器(模拟量信号) */
#define REG_A01		0x0401
#define REG_AXX		REG_A01


//#define TEMP_VAILD_FLAG		0x5151

//typedef struct
//{
//    uint16_t flag;//表示识别号，有效 
//    uint16_t show_temp;//需要显示的温度值 
//    uint16_t real_temp;//实际的温度值  
//    uint16_t offset;   //偏差的温度值
//}T_TEMP_CORRECT_VALUE;



//#pragma pack(1) // 这个不能加，一加就出错

#define UPDATE_TEMP_DATA              0x1
#define UPDATE_HUMIDITY_DATA          0x2
#define UPDATE_GASSPRESSURE_DATA      0x4
#define UPDATE_SUNLIGHT_DATA          0x8


typedef struct
{
	/* 03H 06H 读写保持寄存器 */
	int16_t temperature_reg; //0x0   //第一个值是识别好  为0x1234  //32768
    int16_t humidity_reg; //0x1   //   0-10000   
    int16_t gassPressure_reg; //0x2   10011 KP   32768 
    int16_t sunLight_reg; //0x3      0-65535  只有整数
    int16_t reserve_reg[96];  //0x2
    int16_t  model_number_reg;//0x64
    int16_t  testPoint_number_reg;//0x65 
    int16_t  device_addr_reg;//0x66
    int16_t  baudrate_reg;//0x67  //1:2400  2:4800 3:9600 4:19200  5:38400  6:115200
    int16_t  comm_mode_reg;//0x68    //0x1:RS485   OX2:主动上传 其他参数参数没有用
    int16_t  protocol_reg;//0x69
    int16_t  upload_time_reg;//0x6a   //1-3600  秒       、、
    int16_t  checkbit_reg;//0x6b
    int16_t  stopbit_reg;//0x6c
    int16_t  temperature_correct_reg;//0x6d
    int16_t  humidity_correct_reg;//0x6e
    int16_t  gassPressure_correct_reg;//0x6e
    int16_t  sunLight_correct_reg; //0x70
    //uint16_t  reserve_reg_1;  //reserve 6f 
    
//    uint16_t  temperature_correct__80_reg;
//    uint16_t  temperature_correct__40_reg;
//    uint16_t  temperature_correct_0_reg;
//    uint16_t  temperature_correct_40_reg;
//    uint16_t  temperature_correct_80_reg;
//    uint16_t  temperature_correct_120_reg;
//    uint16_t  temperature_correct_160_reg;
//    uint16_t  temperature_correct_200_reg;
//    uint16_t  temperature_correct_240_reg;
//    uint16_t  temperature_correct_280_reg;
//    uint16_t  temperature_correct_320_reg;
//    uint16_t  temperature_correct_360_reg;
//    uint16_t  temperature_correct_400_reg;
    //.... 设置最大显示的温度
    int16_t  temperature_min_reg;
    int16_t  temperature_max_reg;
    
    //T_TEMP_CORRECT_VALUE temp_value[MAX_CORRECT_SIZE];//一般最大需要20组数据，进行从0到大排序即可
   
}T_MODBUS_REG;

//#pragma pack()



void insert_temp_correct_value(uint16_t correct_value, uint16_t show_value);
uint16_t get_temp_correct_value(uint16_t real_temp);

//这个是从高字节向低字节发送
typedef struct
{

    uint16_t temperature_set_min_reg; //0xfffe
	uint16_t temperature_set_max_reg; //0xffff

   
}T_MODBUS_REG_HIGH;

typedef struct
{
	uint8_t rx_buf[S_RX_BUF_SIZE];
	uint8_t rx_count;
	uint8_t rx_status;
	uint8_t rx_new_flag;

	uint8_t rsp_code;

	uint8_t tx_buf[S_TX_BUF_SIZE];
	uint8_t tx_count;
}T_MODBUS;

void modbusInit();
void modbusPoll(); 

#endif
