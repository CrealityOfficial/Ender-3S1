#ifndef __I2C_EEPROM_H
#define __I2C_EEPROM_H

#include "../../inc/MarlinConfig.h"
// #include "../../Marlin.h"

#include <libmaple/gpio.h>


/******************** IIC ********************/
//IO方向设置
#define SDA_IN()  {PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH&=0XFFFF0FFF;PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH|=8<<12;}
#define SDA_OUT() {PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH&=0XFFFF0FFF;PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH|=3<<12;}

//IO操作函数	 
#define IIC_SCL_0()   digitalWrite(IIC_EEPROM_SCL,LOW)
#define IIC_SCL_1()   digitalWrite(IIC_EEPROM_SCL,HIGH)
#define IIC_SDA_0()   digitalWrite(IIC_EEPROM_SDA,LOW)
#define IIC_SDA_1()   digitalWrite(IIC_EEPROM_SDA,HIGH)
#define READ_SDA()    READ(IIC_EEPROM_SDA)  

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				        //发送IIC开始信号
void IIC_Stop(void);	  		      	//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);		//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);				        	//IIC发送ACK信号
void IIC_NAck(void);				        //IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  


/******************** EEPROM ********************/
#define BL24C01		127
#define BL24C02		255
#define BL24C04		511
#define BL24C08		1023
#define BL24C16		2047
#define BL24C32		4095
#define BL24C64	  8191
#define BL24C128	16383
#define BL24C256	32767  
#define EE_TYPE BL24C16
					  
uint8_t BL24CXX_ReadOneByte(uint16_t ReadAddr);							          //指定地址读取一个字节
void BL24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite);		//指定地址写入一个字节
void BL24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len);//指定地址开始写入指定长度的数据
uint32_t BL24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len);					//指定地址开始读取指定长度数据
void BL24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);	//从指定地址开始写入指定长度的数据
void BL24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);   	//从指定地址开始读出指定长度的数据

uint8_t BL24CXX_Check(void);  //检查器件
void BL24CXX_Init(void); //初始化IIC


#endif

