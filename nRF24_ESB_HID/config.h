/****************************************Copyright (c)****************************************************
**                                        
**                                 
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			main.c
** Last modified  Date:          
** Last Version:	1.0
** Descriptions:		
**						
**--------------------------------------------------------------------------------------------------------
** Created by:			FiYu
** Created date:		2014-8-5
** Version:			    1.0
** Descriptions:		USB HID 无线收发数据实验程序：NRF24LU1P接收到USB数据后，将USB数据返回。
**	                通过USB HID配置nRF24LU1P的无线参数:无线信道和接收数据长度
**                  接收发射端的无线信息，通过USB HID上传
**                  通过USB HID将信息发送给nRF24LU1P，再由nRF24LU1P通过无线发送至设备
**                  
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:				
** Descriptions:		
**
** Rechecked by:			
**********************************************************************************************************/
#ifndef CONFIG_H__
#define CONFIG_H__

#include "usb_desc.h"


#define  RF_CHANNEL        4    // 信道
#define  RX_PAYLOAD_LEN    8    // 接收数据长度


/*-------------------管脚定义--------------------------------------------------*/
#define  LED    P04  // 开发板上的指示灯
#define  SW     P05  // 开发板上的按键


/* 本例程中nRF24LU1P管脚配置
P00: sck，编程接口，也可以配置为其他功能。
P01: mosi，编程接口，也可以配置为其他功能。
P02: miso，编程接口，也可以配置为其他功能。
P03: csn，编程接口，也可以配置为其他功能。

P04：输出，驱动LED	          
P05：输入，按键检测
*/



/*-------------------SUB命令--------------------------------------------------*/
#define  CMD_IDLE             0 
#define  CMD_CONFIG_RF        1    // 配置无线参数,信道和数据长度
#define  CMD_SENDTODEV        2    // 将接收到的数据发送给设备






#endif // CONFIG_H__
/********************************************END FILE*****************************************************/