#include "i2c_eeprom.h"
#include <stdlib.h>
#include "../../pins/stm32f1/pins_CREALITY_S1.h"

// #include "../../inc/MarlinConfigPre.h"
// #include "../../Marlin.h"

// #include "LCD_RTS.h"


/******************** IIC ********************/

//初始化IIC
void IIC_Init(void)
{					     
  SET_OUTPUT(IIC_EEPROM_SDA);
  SET_OUTPUT(IIC_EEPROM_SCL);
 
	IIC_SCL_1();
	IIC_SDA_1();
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA_1();	  	  
	IIC_SCL_1();
	delay_us(4);
 	IIC_SDA_0();//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_0();//钳住I2C总线，准备发送或接收数据 
}	  

//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL_0();
	IIC_SDA_0();//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_1(); 
	IIC_SDA_1();//发送I2C总线结束信号
	delay_us(4);							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA_1();delay_us(1);	   
	IIC_SCL_1();delay_us(1);	 
	while(READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_0();//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL_0();
	SDA_OUT();
	IIC_SDA_0();
	delay_us(2);
	IIC_SCL_1();
	delay_us(2);
	IIC_SCL_0();
}

//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL_0();
	SDA_OUT();
	IIC_SDA_1();
	delay_us(2);
	IIC_SCL_1();
	delay_us(2);
	IIC_SCL_0();
}					 

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t txd)
{                        
	uint8_t t;   
	SDA_OUT(); 	    
	IIC_SCL_0();//拉低时钟开始数据传输
	for(t=0; t<8; t++)
	{              
		// IIC_SDA=(txd&0x80)>>7;
		if(txd & 0x80) {IIC_SDA_1();}
		else IIC_SDA_0();
		txd <<= 1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL_1();
		delay_us(2);
		IIC_SCL_0();	
		delay_us(2);
	}
} 	   

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_IN();//SDA设置为输入
	for(i=0; i<8; i++)
	{
		IIC_SCL_0(); 
		delay_us(2);
		IIC_SCL_1();
		receive <<= 1;
		if(READ_SDA()) receive++;   
		delay_us(1); 
	}					 
	if (!ack)
		IIC_NAck();//发送nACK
	else
		IIC_Ack(); //发送ACK   
	return receive;
}


/******************** EEPROM ********************/

//初始化IIC接口
void BL24CXX_Init(void)
{
	IIC_Init();
}

//在BL24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
uint8_t BL24CXX_ReadOneByte(uint16_t ReadAddr)
{				  
	uint8_t temp = 0;		  	    																 
  IIC_Start();  
	if(EE_TYPE>BL24C16)
	{
		IIC_Send_Byte(0XA0);	   //发送写命令
		IIC_Wait_Ack();
		IIC_Send_Byte(ReadAddr >> 8);//发送高地址
		IIC_Wait_Ack();		 
	}else IIC_Send_Byte(0XA0+((ReadAddr/256) << 1));   //发送器件地址0XA0,写数据 	 

	IIC_Wait_Ack(); 
  IIC_Send_Byte(ReadAddr%256);   //发送低地址
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(0XA1);           //进入接收模式			   
	IIC_Wait_Ack();	 
  temp = IIC_Read_Byte(0);		   
  IIC_Stop();//产生一个停止条件	    
	return temp;
}

//在BL24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void BL24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{				   	  	    																 
  IIC_Start();  
	if(EE_TYPE>BL24C16)
	{
		IIC_Send_Byte(0XA0);	    //发送写命令
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//发送高地址
 	}else
	{
		IIC_Send_Byte(0XA0+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据 
	}	 
	IIC_Wait_Ack();	   
  IIC_Send_Byte(WriteAddr%256);   //发送低地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //发送字节							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//产生一个停止条件 
	delay(2); 
}

//在BL24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void BL24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len)
{  	
	uint8_t t;
	for(t=0;t<Len;t++)
	{
		BL24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在BL24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
uint32_t BL24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len)
{  	
	uint8_t t;
	uint32_t temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=BL24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}

//检查BL24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
uint8_t BL24CXX_Check(void)
{
	uint8_t temp;
	temp=BL24CXX_ReadOneByte(255);//避免每次开机都写BL24CXX			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		BL24CXX_WriteOneByte(255,0X55);
		//关闭机箱风扇和LED灯
		BL24CXX_WriteOneByte(1, 0);
		BL24CXX_WriteOneByte(2, 0);
		//设置热床目标温度为0
		BL24CXX_WriteOneByte(3, 0);
		
    	temp=BL24CXX_ReadOneByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//在BL24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void BL24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=BL24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  

//在BL24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void BL24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
	while(NumToWrite--)
	{
		BL24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}

