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
 * $LastChangedRevision: 133 $
 */

/** @file
 * @brief Gazell pairing library Host example 
 * @defgroup gzll_host_w_dyn_pair_example Gazell pairing library Host example 
 * @{
 * @ingroup nrf_examples
 *
 * @brief Gazell Link Layer Host using the Gazell Pairing Library for adding 
 * dynamic pairing functionality. 
 *
 * The example is monitoring for pairing requests as well as normal user data. 
 * Pairing will be granted to any device sending a pairing request. 
 * If user data is received the first payload byte (byte 0) is written to P0. 
 *
 * The project @ref gzll_device_w_dyn_pair_example can be used as a counterpart 
 * for transmitting data.
 *
 */

//lint -e534
//lint -e830
#include "nrf24lu1p.h"
#include <string.h>
#include "hal_usb.h"
#include "hal_usb_hid.h"
#include "usb_map.h"
#include "gzll_mcu.h"
#include "gzll.h"
#include "gzp.h"

#ifdef GZP_CRYPT_DISABLE
#error This example project uses gzp_crypt, please remove the definition "GZP_CRYPT_DISABLE".
#endif

//Gzp
#define LEFT_PIPE             2
#define RIGHT_PIPE            3

static bool    xdata gzp_rx_received            = false;
static bool    xdata left_usb_in               = false;
static bool    xdata right_usb_in              = false;
static uint8_t xdata gzp_tx_device_number       = 0;
static uint8_t xdata sensor_size                = 0;
static uint8_t xdata left_packet_index          = 0;
static uint8_t xdata right_packet_index         = 0;
static uint8_t xdata packet_sum                 = 0;
static uint8_t xdata gzp_valid_length           = 30;
static uint8_t xdata gzp_packet_origin          = 2;
static uint8_t xdata packet_origin              = 0;
static uint8_t xdata packet_copy_length         = 30;
static uint8_t xdata packet_last_length         = 0;
static uint8_t xdata left_gzp_data[GZLL_MAX_PAYLOAD_LENGTH];
static uint8_t xdata right_gzp_data[GZLL_MAX_PAYLOAD_LENGTH];
static uint8_t xdata left_sensor_data[USB_IN_PACKET_SIZE];
static uint8_t xdata right_sensor_data[USB_IN_PACKET_SIZE];
static uint8_t xdata left_usb_data[GZLL_MAX_PAYLOAD_LENGTH];
static uint8_t xdata right_usb_data[GZLL_MAX_PAYLOAD_LENGTH];

static void          nRF24_gzp_usb_init();
static void          nRF24_gzp_host_execute();
static void          nRF24_multi_packet_init(uint8_t size);
static void          nRF24_send_to_usb();
static void          nRF24_gzp_read_write();
static void          nRF24_gzp_copy_packet_valid_data(uint8_t * des, uint8_t * src, uint8_t start, uint8_t length);
static bool          nRF24_gzp_check_sensor_CRC(uint8_t * sensor_data, uint8_t crc);
static void          nRF24_gzp_add_zero(uint8_t * sensor_data);
static void          nRF24_multi_packet_gzp(uint8_t hand, uint8_t size);
static void          nRF24_single_packet_gzp(uint8_t hand);
static void          nRF24_usb_to_gzp(uint8_t * usb_data);

//USB
static uint8_t xdata usb_in_buf[USB_IN_PACKET_SIZE];
static uint8_t xdata usb_out_buf[USB_OUT_PACKET_SIZE];
static bool    xdata app_usb_out_data_ready = false;
static bool    xdata app_pending_usb_write = false;

extern code const usb_string_desc_templ_t g_usb_string_desc;

static void          app_send_data_to_usb(uint8_t * buf, uint8_t size);
static void          app_parse_usb_out_packet();
static void          app_wait_usb_write();

hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req* req, uint8_t** data_ptr, uint8_t* size) large reentrant;
void    suspend_cb(uint8_t allow_remote_wu) large reentrant;
void    resume_cb() large reentrant;
void    reset_cb() large reentrant;
uint8_t ep_1_in_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;
uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;

static void nRF24_gzp_host_execute()
{
    // If gzpair_host_execute() returns true, a pairing request has been received
    gzp_host_execute();

    // If Host ID request received
    if(gzp_id_req_received())
    {
        // Always grant request
        gzp_id_req_grant();
    }
}

static void nRF24_send_to_usb()
{
    if (hal_usb_get_state() == CONFIGURED && gzp_rx_received == true)
    {
        if (left_usb_in)
        {
            app_send_data_to_usb(left_sensor_data, USB_IN_PACKET_SIZE);
            left_usb_in = false;
            memset(left_sensor_data, 0, USB_IN_PACKET_SIZE);
            gzp_rx_received = false;
        }
        if (right_usb_in)
        {
            app_send_data_to_usb(right_sensor_data, USB_IN_PACKET_SIZE);
            right_usb_in = false;
            memset(right_sensor_data, 0, USB_IN_PACKET_SIZE);
            gzp_rx_received = false;
        }
    }
}

static void nRF24_single_packet_gzp(uint8_t hand)
{
    uint8_t i = 0;
    if (hand == LEFT_PIPE)
    {
        memset(left_sensor_data, 0, USB_IN_PACKET_SIZE);
        for (i = 0; i < GZLL_MAX_PAYLOAD_LENGTH; i++)
            left_sensor_data[i] = left_gzp_data[i];
        left_usb_in = true;
    }

    if (hand == RIGHT_PIPE)
    {
        memset(right_sensor_data, 0, USB_IN_PACKET_SIZE);
        for (i = 0; i < GZLL_MAX_PAYLOAD_LENGTH; i++)
            right_sensor_data[i] = right_gzp_data[i];
        right_usb_in = true;
    }
    gzp_rx_received = true;
}

static void nRF24_multi_packet_init(uint8_t size)
{
    sensor_size = size;
    packet_sum  = sensor_size / gzp_valid_length;
    if (sensor_size % gzp_valid_length != 0)
        packet_sum += 1;
    packet_last_length = sensor_size - ((packet_sum - 1) * packet_copy_length - 1) - 1;
}

static void nRF24_gzp_copy_packet_valid_data(uint8_t * des, uint8_t * src, uint8_t start, uint8_t length)
{
    uint8_t i = 0;
    for (i = 0; i < length; i++)
        des[start + i] = src[gzp_packet_origin + i];
}

static bool nRF24_gzp_check_sensor_CRC(uint8_t * sensor_data, uint8_t crc)
{
    uint8_t result = 0;
    uint8_t i      = 0;
    for (i = 0; i < sensor_size; i++)
        result ^= sensor_data[i];
    if (result == crc && crc != 0)
        return true;
    else
        return false;
}

static void nRF24_gzp_add_zero(uint8_t * sensor_data)
{
    uint8_t i = 0;
    for (i = sensor_size; i < USB_IN_PACKET_SIZE; i++)
        sensor_data[i] = 0;
}

static void nRF24_multi_packet_gzp(uint8_t hand, uint8_t size)
{
    uint8_t gzp_packet_index;
    if (sensor_size == 0 || sensor_size != size)
        nRF24_multi_packet_init(size);

    if (hand == LEFT_PIPE)
    {
        left_packet_index++;
        gzp_packet_index = left_gzp_data[1];
        if (gzp_packet_index == left_packet_index)
        {
            packet_origin = (left_packet_index - 1) * packet_copy_length;
            if (left_packet_index != packet_sum)
                nRF24_gzp_copy_packet_valid_data(left_sensor_data, left_gzp_data, packet_origin, packet_copy_length);
            else
            {
                nRF24_gzp_copy_packet_valid_data(left_sensor_data, left_gzp_data, packet_origin, packet_last_length);
                if (nRF24_gzp_check_sensor_CRC(left_sensor_data, left_gzp_data[packet_last_length + 2]) == true)
                {
                    nRF24_gzp_add_zero(left_sensor_data);
                    gzp_rx_received = true;
                    left_usb_in     = true;
                }
                else
                {
                    left_usb_in     = false;
                    memset(left_sensor_data, 0, USB_IN_PACKET_SIZE);
                }
                left_packet_index = 0;
            }
        }
        else
        {
            left_usb_in = false;
            if (gzp_packet_index == 1)
            {
                left_packet_index = 1;
                packet_origin     = (left_packet_index - 1) * packet_copy_length;
                nRF24_gzp_copy_packet_valid_data(left_sensor_data, left_gzp_data, packet_origin, packet_copy_length);
            }
            else
            {
                memset(left_sensor_data, 0, USB_IN_PACKET_SIZE);
                left_packet_index = 0;
            }
        }
    }

    if (hand == RIGHT_PIPE)
    {
        right_packet_index++;
        gzp_packet_index = right_gzp_data[1];
        if (gzp_packet_index == right_packet_index)
        {
            packet_origin = (right_packet_index - 1) * packet_copy_length;
            if (right_packet_index != packet_sum)
                nRF24_gzp_copy_packet_valid_data(right_sensor_data, right_gzp_data, packet_origin, packet_copy_length);
            else
            {
                nRF24_gzp_copy_packet_valid_data(right_sensor_data, right_gzp_data, packet_origin, packet_last_length);
                if (nRF24_gzp_check_sensor_CRC(right_sensor_data, right_gzp_data[packet_last_length + 2]) == true)
                {
                    nRF24_gzp_add_zero(right_sensor_data);
                    gzp_rx_received = true;
                    right_usb_in     = true;
                }
                else
                {
                    right_usb_in     = false;
                    memset(right_sensor_data, 0, USB_IN_PACKET_SIZE);
                }
                right_packet_index = 0;
            }
        }
        else
        {
            right_usb_in = false;
            if (gzp_packet_index == 1)
            {
                right_packet_index = 1;
                packet_origin     = (right_packet_index - 1) * packet_copy_length;
                nRF24_gzp_copy_packet_valid_data(right_sensor_data, right_gzp_data, packet_origin, packet_copy_length);
            }
            else
            {
                memset(left_sensor_data, 0, USB_IN_PACKET_SIZE);
                right_packet_index = 0;
            }
        }
    }
}

static void nRF24_gzp_read_write()
{
    uint8_t pipe;
    gzp_tx_device_number = 0;
    if (gzll_rx_data_ready(LEFT_PIPE))
    {
        gzp_tx_device_number++;
        pipe = LEFT_PIPE;
        if (gzll_rx_fifo_read(left_gzp_data, NULL, &pipe))
        {
            if (left_gzp_data[0] != LEFT_PIPE)
                nRF24_multi_packet_gzp(LEFT_PIPE, left_gzp_data[0] - 1);
            else
                nRF24_single_packet_gzp(LEFT_PIPE);
				}
		}
    if (gzll_rx_data_ready(RIGHT_PIPE))
    {
        gzp_tx_device_number++;
        pipe = RIGHT_PIPE;
        if (gzll_rx_fifo_read(right_gzp_data, NULL, &pipe))
        {
            if (right_gzp_data[0] != RIGHT_PIPE)
                nRF24_multi_packet_gzp(RIGHT_PIPE, right_gzp_data[0] - 1);
            else
                nRF24_single_packet_gzp(RIGHT_PIPE);
				}
    }
}

static void nRF24_usb_to_gzp(uint8_t * usb_data)
{
    uint8_t i;
    uint8_t hand = usb_data[1];

    if (hand == LEFT_PIPE)
    {
        for (i = 0; i < GZLL_MAX_PAYLOAD_LENGTH; i++)
            left_usb_data[i] = usb_data[i];
        gzll_tx_fifo_flush();
        gzll_ack_payload_write(left_usb_data, GZLL_MAX_PAYLOAD_LENGTH, LEFT_PIPE);
        memset(left_usb_data, 0, GZLL_MAX_PAYLOAD_LENGTH);
    }

    if (hand == RIGHT_PIPE)
    {
        for (i = 0; i < GZLL_MAX_PAYLOAD_LENGTH; i++)
            right_usb_data[i] = usb_data[i];
        gzll_tx_fifo_flush();
        gzll_ack_payload_write(right_usb_data, GZLL_MAX_PAYLOAD_LENGTH, RIGHT_PIPE);
        memset(right_usb_data, 0, GZLL_MAX_PAYLOAD_LENGTH);
    }
}

void main(void)
{
    nRF24_gzp_usb_init();                                          
  
    while (true)
    {
        if (app_usb_out_data_ready)
        {
            nRF24_usb_to_gzp(usb_out_buf);
					  memset(usb_out_buf, 0, USB_OUT_PACKET_SIZE);
            app_usb_out_data_ready = false;
        }

        if (gzp_tx_device_number < 2)
            nRF24_gzp_host_execute();

        nRF24_send_to_usb();

        if(gzll_get_rx_data_ready_pipe_number() >= 2)
        {
            nRF24_gzp_read_write();
		    }
    }          
}

static void nRF24_gzp_usb_init()
{
    memset(left_gzp_data, 0, GZLL_MAX_PAYLOAD_LENGTH);
    memset(right_gzp_data, 0, GZLL_MAX_PAYLOAD_LENGTH);
    memset(left_sensor_data, 0, USB_IN_PACKET_SIZE);
    memset(right_sensor_data, 0, USB_IN_PACKET_SIZE);
    memset(left_usb_data, 0, GZLL_MAX_PAYLOAD_LENGTH);
    memset(right_usb_data, 0, GZLL_MAX_PAYLOAD_LENGTH);

    // USB HAL initialization
    hal_usb_init(true, device_req_cb, reset_cb, resume_cb, suspend_cb);   
    hal_usb_endpoint_config(0x81, EP1_2_PACKET_SIZE, ep_1_in_cb);  // Configure 32 byte IN endpoint 1
    hal_usb_endpoint_config(0x02, EP1_2_PACKET_SIZE, ep_2_out_cb); // Configure 32 byte OUT endpoint 2

    mcu_init();
    gzll_init();
    gzp_init();
    gzp_pairing_enable(true);

    // Open pipe 2. (Pipe 0 and 1 are reserved by pairing library).
    gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) | (1 << 2));
    gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) | (1 << 3));
  
    // Set P0 as output
    P0DIR = 0;                                          

    EA = 1;

    // Enter host mode (start monitoring for data)
    gzll_rx_start();
}

static void app_send_data_to_usb(uint8_t * buf, uint8_t size)
{
    app_wait_usb_write();
    app_pending_usb_write = true;  
    memcpy(usb_in_buf, buf, size);
    hal_usb_send_data(1, usb_in_buf, EP1_2_PACKET_SIZE);
}

static void app_wait_usb_write()
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

hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req* req, uint8_t** data_ptr, uint8_t* size) large reentrant
{
    hal_usb_dev_req_resp_t retval;

    if( hal_usb_hid_device_req_proc(req, data_ptr, size, &retval) == true ) 
    {
        // The request was processed with the result stored in the retval variable
        return retval;
    }
    else
    {
        // The request was *not* processed by the HID subsystem
        return STALL;   
    }
}

void suspend_cb(uint8_t allow_remote_wu) large reentrant
{
    USBSLP = 1; // Disable USB clock (auto clear)
    allow_remote_wu = 0;  
}

void resume_cb(void) large reentrant
{
}

void reset_cb(void) large reentrant
{
}

//-----------------------------------------------------------------------------
// USB Endpoint Callbacks
//-----------------------------------------------------------------------------  
uint8_t ep_1_in_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant
{  
    app_pending_usb_write = false;
    return 0x60; // NAK
    adr_ptr = adr_ptr;
    size = size;
}

uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant
{
    memcpy(usb_out_buf, adr_ptr, *size);
    app_usb_out_data_ready = true;
    //P0 = *size;
    return 0xff; // ACK
}

/** @} */
