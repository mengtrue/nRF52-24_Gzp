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
** Descriptions:		USB HID ÎÞÏßÊÕ·¢Êý¾ÝÊµÑé³ÌÐò£
**	                Í¨¹ýUSB HIDÅäÖÃnRF24LU1PµÄÎÞÏß²ÎÊý:ÎÞÏßÐÅµÀºÍ½ÓÊÕÊý¾Ý³¤¶È
**                  ½ÓÊÕ·¢Éä¶ËµÄÎÞÏßÐÅÏ¢£¬Í¨¹ýUSB HIDÉÏ´«
**                  Í¨¹ýUSB HID½«ÐÅÏ¢·¢ËÍ¸ønRF24LU1P
**                  
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:				
** Descriptions:		
**
** Rechecked by:			
**********************************************************************************************************/
#include "nrf24lu1p.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal_nrf.h"
#include "hal_usb.h"
#include "hal_usb_hid.h"
#include "usb_map.h"
#include "hal_flash.h"
#include "hal_delay.h"


/*-----------------------------------------------------------------------------
** ÄÚ²¿ºê¶¨Òå
-----------------------------------------------------------------------------*/
#define USB_IN_CMD usb_in_buf[0]
#define USB_OUT_CMD   usb_out_buf[0]
#define USB_CONFIGRF_CH usb_out_buf[1]
#define USB_CONFIGRF_LEN usb_out_buf[2]
#define ERROR_CODE usb_in_buf[1]

#define RF_OUT_CMD rf_out_buf[0]
#define RF_IN_CMD rf_in_buf[0]

#define SEND_USB() app_send_usb_in_data(0, 0)

//#define FORWARD_RF_TO_USB() app_send_usb_in_data(rf_in_buf, EP1_2_PACKET_SIZE)


//#define FORWARD_RF_TO_USB() app_send_usb_in_data(rf_in_buf, EP1_2_PACKET_SIZE)

#define RIGHT_HAND 0x1F

#define LEFT_HAND  0xaa 

#define Xor_data_num 3

#define Rx_data_num 64

#define EP1_2_PACKET_SIZE_change 0x40

#define Channel_piple_address 0x3c00
//#define FORWARD_RF_TO_USB() app_send_usb_in_data(rf_in_buf, 52)


/*-----------------------------------------------------------------------------
** USBÏà¹Ø±äÁ¿¶¨Òå
-----------------------------------------------------------------------------*/
static xdata uint8_t usb_in_buf[EP1_2_PACKET_SIZE_IN];

static xdata uint8_t usb_in_buf_1[EP1_2_PACKET_SIZE_IN];

static xdata uint8_t usb_out_buf[EP1_2_PACKET_SIZE_IN];

static bool xdata app_usb_out_data_ready = false;
extern code const usb_string_desc_templ_t g_usb_string_desc;
static bool xdata app_pending_usb_write = false;

static xdata uint8_t usb_out_test_0[5]={1,1,1,1,1};//{1,2,3,4,5,6,7,8,9,1,2,3,4,5};
static xdata uint8_t usb_out_test_1[5]={2,2,2,2,2};//{1,2,3,4,5,6,7,8,9,1,2,3,4,5};
//static xdata uint8_t usb_out_test_2[5]={3,3,3,3,3};//{1,2,3,4,5,6,7,8,9,1,2,3,4,5};
//static xdata uint8_t usb_out_test_3[5]={4,4,4,4,4};//{1,2,3,4,5,6,7,8,9,1,2,3,4,5};
//static xdata uint8_t usb_out_test_4[5]={5,5,5,5,5};//{1,2,3,4,5,6,7,8,9,1,2,3,4,5};
//static xdata uint8_t usb_out_test_5[5]={6,6,6,6,6};//{1,2,3,4,5,6,7,8,9,1,2,3,4,5};

static uint8_t usb_out_buff_size=0;

static xdata uint8_t channel_piple_write[4]={1,1,1,1};
static xdata uint8_t channel_piple_read[4]={2,2,2,2};
/*-----------------------------------------------------------------------------
** RFÏà¹Ø±äÁ¿¶¨Òå
-----------------------------------------------------------------------------*/
static uint8_t xdata rf_in_buf[32];
static uint8_t xdata rf_out_buf[32];

static uint8_t xdata rf_in_buf_1[32];

static uint8_t m_packt_num=0;

static uint8_t m_data_num=0;

static uint16_t m_packt_first_second;

static uint16_t m_count_number_sent=0;

//static uint8_t xdata rf_in_buf[EP1_2_PACKET_SIZE];
//static uint8_t xdata rf_out_buf[32];
static bool xdata packet_received = false;
static bool xdata radio_busy = false;
static bool xdata transmitted = false;

static bool xdata  rf_receive_data_second_pactket = false;

static uint8_t m_receive_data_left_or_right_hand=0;

uint8_t cmd = 0;

uint8_t send_data_count=0;



//-----------------------------------------------------------------------------
// Internal function prototypes
//-----------------------------------------------------------------------------
static void app_send_usb_in_data(uint8_t * buf, uint8_t size);
static void app_parse_usb_out_packet();
static void app_wait_while_usb_pending();
static void rf_config(void);

//-----------------------------------------------------------------------------
// Flash write and read function
//-----------------------------------------------------------------------------
static void channel_piple_flash_write(uint8_t * buf, uint8_t size);
static void channel_piple_flash_read(uint8_t * buf, uint8_t size);
/*-----------------------------------------------------------------------------
** USB»Øµ÷º¯ÊýÉùÃ÷
-----------------------------------------------------------------------------*/
hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req* req, uint8_t** data_ptr, uint8_t* size) large reentrant;
void suspend_cb(uint8_t allow_remote_wu) large reentrant;
void resume_cb() large reentrant;
void reset_cb() large reentrant;
uint8_t ep_1_in_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;
uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;

/*******************************************************************************************************
 * Ãè  Êö : MAINº¯Êý
 * Èë  ²Î : none
 * ·µ»ØÖµ : none
 *******************************************************************************************************/
void main()
{
  // USB HAL initialization
	//P0DIR = 0xEF;	
	
  hal_usb_init(true, device_req_cb, reset_cb, resume_cb, suspend_cb);   
	
 // hal_usb_endpoint_config(0x81, EP1_2_PACKET_SIZE, ep_1_in_cb);  // Configure 32 byte IN endpoint 1
	
	hal_usb_endpoint_config(0x81, EP1_2_PACKET_SIZE_IN, ep_1_in_cb);  // Configure 32 byte IN endpoint 1
	
  hal_usb_endpoint_config(0x02, EP1_2_PACKET_SIZE, ep_2_out_cb); // Configure 32 byte OUT endpoint 2
	
	

  
  rf_config();    // ÎÞÏßÅäÖÃ
  
  P0DIR = 0xEF;	  // ÅäÖÃP0:P04ÅäÖÃÎªÊä³ö
  LED = 1;        // Ï¨ÃðÖ¸Ê¾µÆ

  EA = 1;         // Ê¹ÄÜÈ«¾ÖÖÐ¶Ï

  //channel_piple_flash_read(channel_piple_read,4); //////read flash 
 
	//channel_piple_read[0]=hal_flash_byte_read(Channel_piple_address);
	
	//hal_flash_bytes_read(Channel_piple_address,hal_flash_byte_read,4);
	
  while(true)                                                                               
  {    
	
    if(hal_usb_get_state() == CONFIGURED)  // USBÒÑÅäÖÃ?
    { 
			//app_send_usb_in_data(usb_out_test_0, EP1_2_PACKET_SIZE_IN);
      if(app_usb_out_data_ready)// USB ½ÓÊÕµ½Ö÷»ú·¢ËÍµÄÊý¾Ý
      {
        // app_parse_usb_out_packet();
	       if(usb_out_buf[1]==0x02){
				  hal_nrf_write_ack_payload(0,usb_out_buf,6);
				 }
				 if(usb_out_buf[1]==0x03){
				  hal_nrf_write_ack_payload(1,usb_out_buf,6);
				 }
				 					
			//	hal_nrf_write_ack_payload(0,usb_out_buf,15);
			 
				
				// hal_nrf_write_ack_payload(0,usb_out_buf,usb_out_buff_size);
				// hal_nrf_write_ack_payload(1,usb_out_buf,6);
				// if(send_data_count==5)
				// {
					
				 //  rf_config();    // ÎÞÏßÅäÖÃ
					  // Clear and flush radio state
          // hal_nrf_get_clear_irq_flags();
          // hal_nrf_flush_rx();
         //  hal_nrf_flush_tx();
					// send_data_count=0;
				 //}
				  
				  CE_LOW(); // Enable receiver 	
				  RFCTL = 0x10;
          RFCKEN = 1;
	        RF = 1;
				  CE_HIGH(); // Enable receiver 				
					EA = 1;    // Ê¹ÄÜÈ«¾ÖÖÐ¶Ï
					
				  app_usb_out_data_ready = false;
					
				//  app_send_usb_in_data(usb_out_test_0, EP1_2_PACKET_SIZE_IN);
      }
      if(packet_received == true) // ½ÓÊÕµ½ÎÞÏßÊý¾Ý?
			{		
			  app_send_usb_in_data(usb_in_buf_1, EP1_2_PACKET_SIZE_IN);			
				packet_received = false;			
				LED = ~LED;			
			}
			
		}
	 if(hal_usb_get_state() == SUSPENDED)
		{
        hal_usb_wakeup();
		}
    /*
		if(cmd == CMD_CONFIG_RF)
		{
      CE_LOW(); // Enable receiver 		
		  hal_nrf_set_rf_channel(USB_CONFIGRF_CH);
		  hal_nrf_set_rx_payload_width((int)HAL_NRF_PIPE0, USB_CONFIGRF_LEN);// ½ÓÊÕÄ£Ê½ÏÂÐèÒªÉèÖÃÊý¾Ý³¤¶È
		  CE_HIGH(); // Enable receiver 
			cmd = CMD_IDLE;
			
		}
		*/
		  
  }
}

//-----------------------------------------------------------------------------
// Handle commands from Host application
//-----------------------------------------------------------------------------
static void app_parse_usb_out_packet()
{
	
	//hal_nrf_write_ack_payload(0,usb_out_buf,usb_out_buff_size);
	
	//hal_nrf_write_ack_payload(0,usb_out_buf,4);
	//hal_nrf_write_ack_payload(1,usb_out_buf,4);
	
  switch(USB_OUT_CMD)
  {
    case CMD_CONFIG_RF:
      
		  cmd = CMD_CONFIG_RF;
      break;

    case CMD_SENDTODEV:
      cmd = CMD_SENDTODEV;
		//  hal_nrf_write_ack_payload(0,usb_out_buf,4);
      break;
    

    default:
      break;
  }
}

//-----------------------------------------------------------------------------
// RF helper functions
//-----------------------------------------------------------------------------

// Initialize radio module
static void rf_config(void)
{
  // Enable radio SPI and clock
  RFCTL = 0x10;
  RFCKEN = 1;
	RF = 1;
	
	hal_nrf_set_rf_channel(RF_CHANNEL);
	hal_nrf_set_datarate(HAL_NRF_2MBPS );	  // RFËÙÂÊ£º250KBPS    HAL_NRF_250KBPS  HAL_NRF_2MBPS
	hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);  // CRC£º16bits	
	hal_nrf_open_pipe(HAL_NRF_PIPE0,true);	  // Open radio pipe(s) and enable/ disable auto acknowledge. 
	//hal_nrf_open_pipe(HAL_NRF_PIPE1,true);	  // Open radio pipe(s) and enable/ disable auto acknowledge. 
  hal_nrf_set_rx_payload_width((int)HAL_NRF_PIPE0, RX_PAYLOAD_LEN);// ½ÓÊÕÄ£Ê½ÏÂÐèÒªÉèÖÃÊý¾Ý³¤¶È
	hal_nrf_set_rx_payload_width((int)HAL_NRF_PIPE1, RX_PAYLOAD_LEN);// ½ÓÊÕÄ£Ê½ÏÂÐèÒªÉèÖÃÊý¾Ý³¤¶È
	//hal_nrf_set_output_power(HAL_NRF_0DBM);

  // Clear and flush radio state
  hal_nrf_get_clear_irq_flags();
  hal_nrf_flush_rx();
  hal_nrf_flush_tx();

  packet_received = false;
   	
	
	
	hal_nrf_enable_ack_payload(true);
  hal_nrf_enable_dynamic_payload(true);
  hal_nrf_setup_dynamic_payload(0xff);

	hal_nrf_set_operation_mode(HAL_NRF_PRX);
	hal_nrf_set_power_mode(HAL_NRF_PWR_UP);// Power up radio
	CE_HIGH(); // Enable receiver 
}

/*-----------------------------------------------------------------------------
** Êý¾ÝÕûºÏ¡¢Æ´½Ó
-----------------------------------------------------------------------------*/
static void rx_data_combine()
{
//	static uint8_t i=0;
	static uint8_t j=0;
	static uint8_t k=0;
	static uint8_t xdata swap_buf_1[32];
	//static uint8_t xdata swap_buf_2[32];
	
	memcpy(usb_in_buf_1,rf_in_buf_1+3,29);
	usb_in_buf_1[29]=rf_in_buf[3];
	usb_in_buf_1[30]=rf_in_buf[4];
	usb_in_buf_1[31]=rf_in_buf[5];
	/*
	for(i=0;i<32;i++)
	{
		usb_in_buf_1[i]=rf_in_buf_1[i];
	}
	*/
	memcpy(swap_buf_1,rf_in_buf+6,m_data_num-32);
	
	for(j=0;j<m_data_num-32;j++)
	{
		usb_in_buf_1[32+j]=swap_buf_1[j];
	}
	
	for(k=m_data_num;k<Rx_data_num;k++)
	{
		usb_in_buf_1[k]=0;
	}
	 //memcpy(usb_in_buf_1)
}

static bool rx_data_Xor()
{
	static uint8_t i=0;
	static uint8_t xdata Xor_data[1];
	
	Xor_data[0]=usb_in_buf_1[0];
	
	for(i=1;i<m_data_num-1;i++)
	{
		Xor_data[0]^=usb_in_buf_1[i];
	}
	
	if(Xor_data[0]==usb_in_buf_1[m_data_num])
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

// Interrupt handler for RF module
/*******************************************************************************************************
 * Ãè  Êö : RFÖÐ¶Ï·þÎñº¯Êý
 * Èë  ²Î : none
 * ·µ»ØÖµ : none
 *******************************************************************************************************/
NRF_ISR()
{
  uint8_t irq_flags;
//	static uint8_t i;
  // Read and clear IRQ flags from radio
  irq_flags = hal_nrf_get_clear_irq_flags();

  switch (irq_flags) 
  {
    // Transmission success.
    case (1 << (uint8_t)HAL_NRF_TX_DS):
      radio_busy = false;
      transmitted = true;
      break;

    // Transmission failed (maximum re-transmits)
    case (1 << (uint8_t)HAL_NRF_MAX_RT):
      hal_nrf_flush_tx();
      radio_busy = false;
      transmitted = false;
      break;

    // Data received 
    case (1 << (uint8_t)HAL_NRF_RX_DR):
      // Read payload
		  
             while (!hal_nrf_rx_fifo_empty()) 
			       { 
				
               hal_nrf_read_rx_payload(rf_in_buf);
              }
						  //ep_2_out_cb(rf_in_buf,32);
						 if(m_packt_num==0)
						 {
					      if(rf_in_buf[0]<=32)
		             {
		              memcpy(usb_in_buf_1,rf_in_buf,32);
			            packet_received = true;
		             } 
		            if(rf_in_buf[0]>32)
		             {
			            
									 m_data_num=rf_in_buf[0];
									 m_packt_first_second=rf_in_buf[1];
									 m_receive_data_left_or_right_hand=rf_in_buf[3];
									 if((m_packt_first_second==1))
										 {
											 m_packt_num++;
							         memcpy(rf_in_buf_1,rf_in_buf,32);
			                 packet_received = false;
										 }
									 /*
									 if((m_packt_first_second==1)&&(rf_in_buf[2]==31))//RIGHT_HAND))
										 {
											 m_packt_num++;
							         memcpy(rf_in_buf_1,rf_in_buf,32);
			                 packet_received = false;
										 }
										 
										else if((m_packt_first_second==1)&&(rf_in_buf[2]==LEFT_HAND))
										 {
											 m_packt_num++;
							         memcpy(rf_in_buf_1,rf_in_buf,32);
			                 packet_received = false;
										 }*/
							     break;
		             }
					  }
		        if((m_packt_num==1))
						 {			      
					       m_packt_num=0;
							   rf_receive_data_second_pactket=true;
							   if((rf_in_buf[1]==2)&&(m_receive_data_left_or_right_hand==rf_in_buf[2]))
							   {
					         rx_data_combine();
									 /*
									 if(rx_data_Xor())
									 {
					         packet_received=true;	
									 }
									 else
									 {
										packet_received=false;	
									 }
									 */
									  packet_received=true;	
							   }	
		        }
		    
	      // hal_nrf_write_ack_payload(0,usb_out_test,3);
      break;

  }
}

//-----------------------------------------------------------------------------
// USB Helper functions
//-----------------------------------------------------------------------------  

/*******************************************************************************************************
 * Ãè  Êö : USBÏòÖ÷»ú·¢ËÍÊý¾Ý
 * Èë  ²Î : buf£º·¢ËÍ»º´æÊ×µØÖ·
 *          size£º·¢ËÍÊý¾Ý³¤¶È
 * ·µ»ØÖµ : none
 *******************************************************************************************************/  
static void app_send_usb_in_data(uint8_t * buf, uint8_t size)
{
  app_wait_while_usb_pending();
  app_pending_usb_write = true;  
  memcpy(usb_in_buf, buf, size);  //size
 // hal_usb_send_data(1, usb_in_buf, EP1_2_PACKET_SIZE);
	hal_usb_send_data(1, usb_in_buf,size);// m_data_num-Xor_data_num);
	
	//memcpy(usb_in_buf_1, buf, size);  //size
 // hal_usb_send_data(1, usb_in_buf_1, EP1_2_PACKET_SIZE);
	
}


static void app_wait_while_usb_pending()
{    
  uint16_t timeout = 50000;   // Will equal ~ 50-100 ms timeout 
  while(timeout--)
  {
    if(!app_pending_usb_write)
    {
      break;
    }
  }    
}

//-----------------------------------------------------------------------------
// USB Callbacks
//-----------------------------------------------------------------------------  

hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req* req, uint8_t** data_ptr, uint8_t* size) large reentrant
{
  hal_usb_dev_req_resp_t retval;
	
  //app_send_usb_in_data(usb_out_test_3,EP1_2_PACKET_SIZE_IN);
	
  if( hal_usb_hid_device_req_proc(req, data_ptr, size, &retval) == true ) 
  {
    // The request was processed with the result stored in the retval variable
		// app_send_usb_in_data(usb_out_test_4,EP1_2_PACKET_SIZE_IN);
    return retval;
  }
  else
  {
    // The request was *not* processed by the HID subsystem
		// app_send_usb_in_data(usb_out_test_5,EP1_2_PACKET_SIZE_IN);
		//  hal_usb_reset();
	//	hal_usb_bus_disconnect();
	//	hal_usb_bus_connect();
    return STALL;   
  }
}

void suspend_cb(uint8_t allow_remote_wu) large reentrant
{
	//app_send_usb_in_data(usb_out_test_0,EP1_2_PACKET_SIZE_IN);
	
  USBSLP = 1; // Disable USB clock (auto clear)
  allow_remote_wu = 0;  
	
	//hal_usb_reset();
}

void resume_cb(void) large reentrant
{
	 //app_send_usb_in_data(usb_out_test_1,EP1_2_PACKET_SIZE_IN);
}

void reset_cb(void) large reentrant
{
	 //app_send_usb_in_data(usb_out_test_2,EP1_2_PACKET_SIZE_IN);
}



//-----------------------------------------------------------------------------
//Flash write for channel and piple
//-----------------------------------------------------------------------------
static void channel_piple_flash_write(uint8_t * buf, uint8_t n)
{
	
  hal_flash_bytes_write(Channel_piple_address,buf,n);
	

}

static void channel_piple_flash_read(uint8_t * buf, uint8_t n)
{
	
hal_flash_bytes_read(Channel_piple_address,buf,n);
	
	
}
//-----------------------------------------------------------------------------
// USB Endpoint Callbacks
//-----------------------------------------------------------------------------  
uint8_t ep_1_in_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant
{  
  app_pending_usb_write = false;
	//hal_nrf_write_tx_payload(usb_out_test,3);
	/*
	m_count_number_sent++;
	if(m_count_number_sent==801)
		{
			m_count_number_sent=0;
		}
	if(m_count_number_sent==800)
	{
	hal_nrf_write_ack_payload(0,usb_out_test_0,5);
	hal_nrf_write_ack_payload(1,usb_out_test_1,5);
	}
	*/
	//hal_nrf_write_ack_payload(0,usb_out_test_0,5);
	
	//hal_nrf_write_ack_payload(0,channel_piple_read,4);
	
//	hal_nrf_write_ack_payload(0,usb_out_buf,4);//usb_out_buff_size);
//	hal_nrf_write_ack_payload(0,usb_out_test_0,5);
//	hal_nrf_write_ack_payload(1,usb_out_test_1,5);
/*
 if(app_usb_out_data_ready)// USB ½ÓÊÕµ½Ö÷»ú·¢ËÍµÄÊý¾Ý
      {
        
			
				if(rf_receive_data_second_pactket==true){
					
				   hal_nrf_write_ack_payload(0,usb_out_buf,6);
				   app_usb_out_data_ready = false;
					 rf_receive_data_second_pactket=false;
				}
			}
			*/
  return 0x60; // NAK
  adr_ptr = adr_ptr;
  size = size;
	
	 
}

uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant
{
	
	static uint8_t i=0;
	
  memcpy(usb_out_buf, adr_ptr, *size);
	
	 //hal_nrf_write_ack_payload(0,usb_out_buf,4);
	 
   //hal_nrf_write_ack_payload(0,usb_out_test_0,5);
	
	usb_out_buff_size= *size;
	
	  
	
	//if(usb_out_buf[0]==0x55 && usb_out_buf[1]==0xaa)
	//{
	/*
		for ( i=0;i<4;i++)
		{
		   channel_piple_write[i]=usb_out_buf[i];
		}
		*/
	  //channel_piple_flash_write(channel_piple_write,4);
		
		//hal_flash_page_erase(18);
		
	 //hal_flash_bytes_write(Channel_piple_address,channel_piple_write,4);
	 //}
	
   //	hal_usb_reset();
//	hal_nrf_write_ack_payload(0,usb_out_test_0,5);

  app_usb_out_data_ready = true;
  //P0 = *size;
  return 0xff; // ACK
}

/********************************************END FILE*****************************************************/
