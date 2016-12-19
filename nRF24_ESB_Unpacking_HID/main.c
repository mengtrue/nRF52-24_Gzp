/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic
 * Semiconductor ASA.Terms and conditions of usage are described in detail
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 2211 $
 */

#include "nrf24lu1p.h"

#include <stdint.h>
#include <string.h>
#include "hal_nrf.h"
#include "hal_usb.h"
#include "hal_usb_hid.h"

// USB define
#define USB_WRITING_TIMEOUT                             50000           //USB write timeout
static uint8_t xdata usb_in_buf [USB_IN_PACKET_SIZE];                   //max data length into hid, according to sensor packet length
static uint8_t xdata usb_out_buf[USB_OUT_PACKET_SIZE];                  //data length to sensor
static bool    xdata app_usb_out_data_ready           = false;
static bool    xdata app_pending_usb_write            = false;
extern code const usb_string_desc_templ_t g_usb_string_desc;            //USB init desriptor report string
static void    usb_hid_init( void );

// Internal function prototypes
static void    app_send_data_to_usb(uint8_t * buf, uint8_t size);       //send data to usb
static void    app_wait_usb_write();                                    //delay function of inputing data to usb

// ESB define
#define ESB_PACKET_LENGTH                                32                 //nRF24 Dongle max ESB packet length
#define ESB_CHANNEL                                      4                  //nRF24 Dongle use channel( 1 ~ 100)
static uint8_t xdata esb_in_buf[ESB_PACKET_LENGTH];                         
static uint8_t xdata esb_out_buf[ESB_PACKET_LENGTH];
static bool    xdata esb_busy                          = false;
static bool    xdata esb_packet_received               = false;
static bool    xdata esb_packet_transmitted            = false;
static void    esb_init( void );

// USB callback
hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req * req, uint8_t ** data_ptr, uint8_t * size) large reentrant;
void suspend_cb(uint8_t allow_remote_wu)                      large reentrant;
void resume_cb()                                              large reentrant;
void reset_cb()                                               large reentrant;
static uint8_t ep_1_in_cb (uint8_t * adr_ptr, uint8_t * size) large reentrant;
static uint8_t ep_2_out_cb(uint8_t * adr_ptr, uint8_t * size) large reentrant;

// Glove
#define SENSOR_PACKET_LENGTH                             52                     //data length sent by sensor
#define ESB_PACKING_AMOUNT                               2                      //
#define UNPACKING_START                                   29
static bool    xdata unpacking_need                    = true;                  //need to unpacking
static uint8_t xdata hand[2]                           = {0xFF, 0xFF};          //left/right hand
static uint8_t xdata hand_unpack_number[2]             = {1, 1};                //left/right unpack number
static uint8_t xdata hand_data[2][SENSOR_PACKET_LENGTH];                        //left/right once package data
static uint8_t xdata hid_glove_data[USB_OUT_PACKET_SIZE];                       //data sent by usb
static void    unpacking_esb_data();                                            //esb received, unpacking

void main()
{    
    RFCTL = 0x10;                                      // Enable radio SPI  
    P0DIR = 0xEF;                                      // Set P0 as output
    RFCKEN = 1;                                        // Enable the radio clock

	  EA = 1;                                            // Enable global interrupt
	
	  usb_hid_init();                                    // init usb hid
	
    esb_init();                                        // init ESB

    while(true)
		{
			  if (hal_usb_get_state() == CONFIGURED)         //USB HID has been configured
				{
					  if (esb_packet_received == true)           //ESB received data
						{
							  if (unpacking_need == true)
								    unpacking_esb_data();
								else
								{
									  app_send_data_to_usb(esb_in_buf, 32);
							      esb_packet_received = false;
								}
						}
				}
		}
}

// Radio interrupt
NRF_ISR()
{
  uint8_t irq_flags;

  // Read and clear IRQ flags from radio
  irq_flags = hal_nrf_get_clear_irq_flags();
	
	switch (irq_flags)
	{
		  case (1 << (uint8_t) HAL_NRF_TX_DS):
				esb_busy = false;
			  esb_packet_transmitted = true;
			  break;
			
			case (1 << (uint8_t) HAL_NRF_MAX_RT):
				hal_nrf_flush_tx();
			  esb_busy = false;
			  esb_packet_transmitted = false;
			  break;
			
			case (1 << (uint8_t) HAL_NRF_RX_DR):
				while (!hal_nrf_rx_fifo_empty())
				{
					  hal_nrf_read_rx_payload(esb_in_buf);
				}
				esb_packet_received = true;
				break;		
	}
}

static void usb_hid_init( void )
{
	 	uint8_t j = 0;
	
	  hal_usb_init(true, device_req_cb, reset_cb, resume_cb, suspend_cb);
	  hal_usb_endpoint_config(0x81, USB_IN_PACKET_SIZE, ep_1_in_cb);
	  hal_usb_endpoint_config(0x02, USB_IN_PACKET_SIZE, ep_2_out_cb);
	
	  memset(hand_data[0], 0, SENSOR_PACKET_LENGTH);
	  memset(hand_data[1], 0, SENSOR_PACKET_LENGTH);
	

	  for ( j = 0; j < USB_OUT_PACKET_SIZE; j++)
	      hid_glove_data[j] = 1;
}

static void esb_init( void )
{
    RF = 1;                                                                    // Enable RF interrupt
	    
    hal_nrf_set_operation_mode(HAL_NRF_PRX);                                   // Configure radio as primary receiver (PRX)
	
	  //hal_nrf_close_pipe(HAL_NRF_ALL);
    //hal_nrf_open_pipe(HAL_NRF_PIPE0, true);
	
    //hal_nrf_set_rx_payload_width((int)HAL_NRF_PIPE0, ESB_PACKET_LENGTH);       // Set payload width to ESB_PACKET_LENGTH
	
	  hal_nrf_set_rf_channel(ESB_CHANNEL);
	  hal_nrf_set_datarate(HAL_NRF_2MBPS);
	  hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);

	  hal_nrf_enable_ack_payload(true);
    hal_nrf_enable_dynamic_payload(true);
    hal_nrf_setup_dynamic_payload(0xff);

    // Power up radio
    hal_nrf_set_power_mode(HAL_NRF_PWR_UP);

    // Enable receiver
    CE_HIGH();
}

hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req * req, uint8_t ** data_ptr, uint8_t * size) large reentrant
{
	  hal_usb_dev_req_resp_t retval;
	  if(hal_usb_hid_device_req_proc(req, data_ptr, size, &retval) == true)
		{
			  return retval;                                                     //request was processed with the result stored in retval variable
		}
		else
		{
			  return STALL;                                                      //request was *not* processed by HID subsystem
		}
	}

void suspend_cb(uint8_t allow_remote_wu) large reentrant
{
	  USBSLP = 1;
	  allow_remote_wu = 0;
}

void resume_cb( void ) large reentrant
{
}

void reset_cb(void) large reentrant
{
}

uint8_t ep_1_in_cb(uint8_t * adr_ptr, uint8_t * size) large reentrant
{
	  app_pending_usb_write = false;
	  return 0x60;                             //NAK
}

uint8_t ep_2_out_cb(uint8_t * adr_ptr, uint8_t * size) large reentrant
{
	  memcpy(usb_out_buf, adr_ptr, *size);
	  app_usb_out_data_ready = true;
	  return 0xff;                             //ACK
}

static void app_send_data_to_usb(uint8_t * buf, uint8_t size)
{
	  app_wait_usb_write();
	  app_pending_usb_write = true;
	  memcpy(usb_in_buf, buf, size);
	  hal_usb_send_data(1, usb_in_buf, USB_IN_PACKET_SIZE);
}

static void app_wait_usb_write()
{
	  uint16_t timeout = USB_WRITING_TIMEOUT;
	  while (timeout--)
		{
			  if (!app_pending_usb_write)
					  break;
		}
}

void copy_hand_unpack_data(bool full, uint8_t index)
{
	  uint8_t start = (hand_unpack_number[index] - 1) * UNPACKING_START;
	  uint8_t i = 0;
	  if (full == true)
		{			  
			  
    	  for (i = 0; i < ESB_PACKET_LENGTH - 2; i++)
	        hand_data[index][start + i] = esb_in_buf[i+3];
		}
		else if (full == false)
		{
			  for (i = 0; i < SENSOR_PACKET_LENGTH - start; i++)
            hand_data[index][start + i] = esb_in_buf[i+3];
		}			
}

void hand_package_data(uint8_t index)
{
	  if (esb_in_buf[1] != hand_unpack_number[index])                   //maybe miss packet
		{
			  memcpy(hand_data[index], 0, SENSOR_PACKET_LENGTH);        //drop current and pre packet
				hand_unpack_number[index] = 1;
				if (esb_in_buf[1] == 1)                                   //save current packet as first package
				{
					  copy_hand_unpack_data(true, index); 
            hand_unpack_number[index] += 1;
				}
		}
		else
		{							  
		    if (hand_unpack_number[index] != ESB_PACKING_AMOUNT)
				{
					  copy_hand_unpack_data(true, index);
					  hand_unpack_number[index] += 1;
				}								  
				else
				{
					  copy_hand_unpack_data(false, index);
					  app_send_data_to_usb(hand_data[index], USB_IN_PACKET_SIZE);
					  memset(hand_data[index], 0, SENSOR_PACKET_LENGTH);
					  hand_unpack_number[index] = 1;
				}
		}

		esb_packet_received = false;
}

static void unpacking_esb_data()
{
	  if (esb_in_buf[0] == USB_IN_PACKET_SIZE)
		{
			  if (hand[0] == 0xFF)
					  hand[0] = esb_in_buf[2];
				else if (hand[1] == 0xFF)
					  hand[1] = esb_in_buf[2];
				
				if ( esb_in_buf[2] == hand[0])
				{
					  hand_package_data(0);
				}
				else if (esb_in_buf[2] == hand[1])
				{
					  hand_package_data(1);
				}
				//hal_nrf_write_ack_payload(HAL_NRF_PIPE0, hid_glove_data, USB_OUT_PACKET_SIZE);
		}
					  
}
/** @} */
