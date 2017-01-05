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
#define ESB_CHANNEL                                      99                  //nRF24 Dongle use channel( 1 ~ 100)

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
#define LEFT_PIPE                                        2                           //define default left/right hand pipe
#define RIGHT_PIPE                                       3
static bool    xdata GLOVE_MULTI_PACKET                  = false;
static bool    xdata left_usb_in                         = false;                    //left pipe data in, need send to usb
static bool    xdata right_usb_in                        = false;                    //right pipe data in, need send to usb
static bool    xdata left_usb_out                        = false;                    //PC App send data out usb, for left pipe
static bool    xdata right_usb_out                       = false;                    //PC App send data out usb, for right pipe
static uint8_t xdata sensor_size                         = 0;                        //sensor_size <= 64, if more than 64, only change USB_IN_PACKET_SIZE (include crc bit if crc bit exist)
static uint8_t xdata left_packet_index                   = 0;                        //left hand glove packet index
static uint8_t xdata right_packet_index                  = 0;                        //right hand glove packet index
static uint8_t xdata packet_sum                          = 0;                        //sum value of total packet from hand glove
static uint8_t xdata esb_valid_length                    = 30;                       //every esb packet valid data length
static uint8_t xdata esb_packet_origin                   = 2;                        //every esb packet start point index
static uint8_t xdata packet_origin                       = 0;                        //unpacking every packet start point index
static uint8_t xdata packet_copy_length                  = 30;                       //packet length non-last packet
static uint8_t xdata packet_last_length                  = 0;                        //last packet length
static uint8_t xdata left_esb_data[ESB_PACKET_LENGTH];                               //data from left hand via ESB
static uint8_t xdata right_esb_data[ESB_PACKET_LENGTH];                              //data from right hand via ESB
static uint8_t xdata left_sensor_data[USB_IN_PACKET_SIZE];                           //left hand sensor data
static uint8_t xdata right_sensor_data[USB_IN_PACKET_SIZE];                          //right hand sensor data
static uint8_t xdata left_usb_data[ESB_PACKET_LENGTH];                               //left hand hid data (send to glove)
static uint8_t xdata right_usb_data[ESB_PACKET_LENGTH];                              //right hand hid data (send to glove)
static void          glove_hid_init(void);                                           //glove hid data init
static void          glove_multi_packet_init(uint8_t size);                          //init multi packet sum and length
static void          glove_hand_sensor_data(uint8_t hand, uint8_t size);             //deal with unpacking data
static void          single_packet_usb_in(uint8_t hand);                             //only one packet
static void          copy_esb_packet_valid_data(uint8_t * des, uint8_t * src, uint8_t start, uint8_t length);       //copy every esb packet
static bool          checkSensorCrc(uint8_t * sensor_data, uint8_t crc);             //check sensor crc bit value
static void          addZeroEnd(uint8_t * sensor_data);                              //after sensor_zie, sensor_data should be zero
static void          copy_usb_out_to_esb(uint8_t hand, uint8_t * usb_data);          //usb out, left hand copy to left_usb_data, right hand copy to right_usb_data
static void          mcu_wakeup_usb(void);                                           //wakeup usb from suspend (USBCON = 0x40)

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
                if (GLOVE_MULTI_PACKET == true)
                {
                    if (left_usb_in)
                    {
                        app_send_data_to_usb(left_sensor_data, USB_IN_PACKET_SIZE);
                        left_usb_in = false;
                        memset(left_sensor_data, 0, USB_IN_PACKET_SIZE);
                        esb_packet_received = false;
                        if (left_usb_out)
                        {
                            hal_nrf_write_ack_payload(LEFT_PIPE, left_usb_data, ESB_PACKET_LENGTH);
                            left_usb_out = false;
                        }
                    }
                    if (right_usb_in)
                    {
                        app_send_data_to_usb(right_sensor_data, USB_IN_PACKET_SIZE);
                        right_usb_in = false;
                        memset(right_sensor_data, 0, USB_IN_PACKET_SIZE);
                        esb_packet_received = false;
                        if (right_usb_out)
                        {
                            hal_nrf_write_ack_payload(RIGHT_PIPE, right_usb_data, ESB_PACKET_LENGTH);
                            right_usb_out = false;
                        }
                    }
                }
                else
                {
                    app_send_data_to_usb(esb_in_buf, USB_IN_PACKET_SIZE);
                    esb_packet_received = false;
                }
            }
        }
        else if (hal_usb_get_state() == SUSPENDED)
        {
            hal_usb_wakeup();
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
            if (hal_nrf_get_rx_data_source() == LEFT_PIPE)
            {
                hal_nrf_read_rx_payload(left_esb_data);
                if (left_esb_data[0] != LEFT_PIPE)
                {
                    GLOVE_MULTI_PACKET = true;
                    glove_hand_sensor_data(LEFT_PIPE, left_esb_data[0] - 1);
                }
                else
                {
                    single_packet_usb_in(LEFT_PIPE);
                }
                //left_usb_in = true;
            }
            else if (hal_nrf_get_rx_data_source() == RIGHT_PIPE)
            {
				        hal_nrf_read_rx_payload(right_esb_data);
                if (right_esb_data[0] != RIGHT_PIPE)
                {
                    GLOVE_MULTI_PACKET = true;
                    glove_hand_sensor_data(RIGHT_PIPE, right_esb_data[0] - 1);
                }
                else
                {
                    single_packet_usb_in(RIGHT_PIPE);
                }
                //right_usb_in = true;                
            }
            //hal_nrf_read_rx_payload(esb_in_buf);
				}
        /*if (GLOVE_MULTI_PACKET == false)
        {
            if (left_usb_in)
                single_packet_usb_in(LEFT_PIPE);
            if (right_usb_in)
                single_packet_usb_in(RIGHT_PIPE);
				}
        else
        {
            if (left_usb_in)
                glove_hand_sensor_data(LEFT_PIPE, left_esb_data[0] - 1);
            if (right_usb_in)
                glove_hand_sensor_data(RIGHT_PIPE, right_esb_data[0] - 1);
				}*/
        //esb_packet_received = true;
				break;		
	}

}

static void usb_hid_init( void )
{	
	  hal_usb_init(true, device_req_cb, reset_cb, resume_cb, suspend_cb);
	  hal_usb_endpoint_config(0x81, USB_IN_PACKET_SIZE, ep_1_in_cb);
	  hal_usb_endpoint_config(0x02, USB_OUT_PACKET_SIZE, ep_2_out_cb);
}

static void esb_init( void )
{
    RF = 1;                                                                    // Enable RF interrupt
	    
    hal_nrf_set_operation_mode(HAL_NRF_PRX);                                   // Configure radio as primary receiver (PRX)
	
	  hal_nrf_close_pipe(HAL_NRF_ALL);
    hal_nrf_open_pipe(LEFT_PIPE, true);	
    hal_nrf_set_rx_payload_width(LEFT_PIPE, ESB_PACKET_LENGTH);       // Set payload width to ESB_PACKET_LENGTH
    hal_nrf_open_pipe(RIGHT_PIPE, true);	
    hal_nrf_set_rx_payload_width(RIGHT_PIPE, ESB_PACKET_LENGTH);       // Set payload width to ESB_PACKET_LENGTH
	
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

static void glove_hid_init()
{
    uint8_t i = 0;
    for (i = 0; i < ESB_PACKET_LENGTH; i++)
    {
        left_usb_data[i] = 2;
        right_usb_data[i] = 3;
    }
    memset(left_sensor_data, 0, USB_IN_PACKET_SIZE);
    memset(right_sensor_data, 0, USB_IN_PACKET_SIZE);
}

static void glove_multi_packet_init(uint8_t size)
{
    sensor_size = size;
    packet_sum = sensor_size / esb_valid_length;
    if (sensor_size % esb_valid_length != 0)
        packet_sum += 1;
    packet_last_length = sensor_size - ( (packet_sum - 1) * packet_copy_length - 1) - 1;
}

static void glove_hand_sensor_data(uint8_t hand, uint8_t size)
{
    uint8_t esb_packet_index;
    if (sensor_size == 0 || sensor_size != size)
        glove_multi_packet_init(size);

    if (hand == LEFT_PIPE)
    {
        left_packet_index++;
        esb_packet_index = left_esb_data[1];
        if (esb_packet_index == left_packet_index)
        {
            packet_origin = (left_packet_index - 1) * packet_copy_length;
            if (left_packet_index != packet_sum)
                copy_esb_packet_valid_data(left_sensor_data, left_esb_data, packet_origin, packet_copy_length);
            else
            {
                copy_esb_packet_valid_data(left_sensor_data, left_esb_data, packet_origin, packet_last_length);
                if (checkSensorCrc(left_sensor_data, left_esb_data[packet_last_length + 2]) == true)
                {
                    addZeroEnd(left_sensor_data);
                    esb_packet_received = true;
                    left_usb_in = true;
                }
                else
                {
                    left_usb_in = false;
                    memset(left_sensor_data, 0, USB_IN_PACKET_SIZE);
                }
                left_packet_index = 0;
            }
		}
        else
        {
            left_usb_in = false;
            if (esb_packet_index == 1)
            {
                left_packet_index = 1;
                packet_origin = (left_packet_index - 1) * packet_copy_length;
                copy_esb_packet_valid_data(left_sensor_data, left_esb_data, packet_origin, packet_copy_length);
            }
            else
            {
                memset(left_sensor_data, 0, USB_IN_PACKET_SIZE);
                left_packet_index = 0;
            }
        }
    } else if (hand == RIGHT_PIPE)
    {
        right_packet_index++;
        esb_packet_index = right_esb_data[1];
        if (esb_packet_index == right_packet_index)
        {
            packet_origin = (right_packet_index - 1) * packet_copy_length;
            if (right_packet_index != packet_sum)
                copy_esb_packet_valid_data(right_sensor_data, right_esb_data, packet_origin, packet_copy_length);
            else
            {
                copy_esb_packet_valid_data(right_sensor_data, right_esb_data, packet_origin, packet_last_length);
                if (checkSensorCrc(right_sensor_data, right_esb_data[packet_last_length + 2]) == true)
                {
                    addZeroEnd(right_sensor_data);
                    esb_packet_received = true;
                    right_usb_in = true;
                }
                else
                {
                    right_usb_in = false;
                    memset(right_sensor_data, 0, USB_IN_PACKET_SIZE);
                }
                right_packet_index = 0;
            }
        }
        else
        {
            right_usb_in = false;
            if (esb_packet_index == 1)
            {
                right_packet_index = 1;
                packet_origin = (right_packet_index - 1) * packet_copy_length;
                copy_esb_packet_valid_data(right_sensor_data, right_esb_data, packet_origin, packet_copy_length);
            }
            else
            {
                memset(right_sensor_data, 0, USB_IN_PACKET_SIZE);
                right_packet_index = 0;
            }
        }
    }
}

static void single_packet_usb_in(uint8_t hand)
{
    uint8_t i = 0;
    memset(esb_in_buf, 0, USB_IN_PACKET_SIZE);
    if (hand == LEFT_PIPE)
    {
        for (i = 0; i < ESB_PACKET_LENGTH; i++)
            esb_in_buf[i] = left_esb_data[i];
    }
    else if (hand == RIGHT_PIPE)
    {
        for (i = 0; i < ESB_PACKET_LENGTH; i++)
            esb_in_buf[i] = right_esb_data[i];
    }
    esb_packet_received = true;
}

static void copy_esb_packet_valid_data(uint8_t * des, uint8_t * src, uint8_t start, uint8_t length)
{
    uint8_t i = 0;
    for (i = 0; i < length; i++)
        des[start + i] = src[esb_packet_origin + i];
}

static bool checkSensorCrc(uint8_t * sensor_data, uint8_t crc)
{
    uint8_t result = 0;
    uint8_t i = 0;
    for (i = 0; i < sensor_size; i++)
        result ^= sensor_data[i];
    if (result == crc && crc != 0)
        return true;
    else
        return false;
}

static void addZeroEnd(uint8_t * sensor_data)
{
    uint8_t i = 0;
    for (i = sensor_size; i < USB_IN_PACKET_SIZE; i++)
        sensor_data[i] = 0;
}

static void copy_usb_out_to_esb(uint8_t hand, uint8_t * usb_data)
{
    uint8_t i = 0;
    if (hand == LEFT_PIPE)
    {
        memset(left_usb_data, 0, ESB_PACKET_LENGTH);
        for (i = 0; i < ESB_PACKET_LENGTH; i++)
            left_usb_data[i] = usb_data[i];
        left_usb_out = true;
    }

    if (hand == RIGHT_PIPE)
    {
        memset(right_usb_data, 0, ESB_PACKET_LENGTH);
        for( i = 0; i < ESB_PACKET_LENGTH; i++)
            right_usb_data[i] = usb_data[i];
        right_usb_out = true;
    }
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
    adr_ptr = adr_ptr;
    size = size;
}

uint8_t ep_2_out_cb(uint8_t * adr_ptr, uint8_t * size) large reentrant
{
	  memcpy(usb_out_buf, adr_ptr, *size);
	  //app_usb_out_data_ready = true;
    copy_usb_out_to_esb(usb_out_buf[1], usb_out_buf);
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

/** @} */
