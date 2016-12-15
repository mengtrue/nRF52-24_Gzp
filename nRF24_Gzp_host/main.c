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

//USB
static xdata uint8_t usb_in_buf[EP1_2_PACKET_SIZE];
static xdata uint8_t usb_out_buf[EP1_2_PACKET_SIZE];
static bool xdata app_usb_out_data_ready = false;
extern code const usb_string_desc_templ_t g_usb_string_desc;
static bool xdata app_pending_usb_write = false;

static void app_send_usb_in_data(uint8_t * buf, uint8_t size);
static void app_parse_usb_out_packet();
static void app_wait_while_usb_pending();
void gzp_usb_init();
hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req* req, uint8_t** data_ptr, uint8_t* size) large reentrant;
void suspend_cb(uint8_t allow_remote_wu) large reentrant;
void resume_cb() large reentrant;
void reset_cb() large reentrant;
uint8_t ep_1_in_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;
uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;

void main(void)
{
  uint8_t payload[GZLL_MAX_PAYLOAD_LENGTH];

  gzp_usb_init();
  mcu_init();
  gzll_init();
  gzp_init();
  gzp_pairing_enable(true);

  // Open pipe 2. (Pipe 0 and 1 are reserved by pairing library).
  gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) | (1 << 2));
  
  // Set P0 as output
  P0DIR = 0;                                          

  EA = 1;

  // Enter host mode (start monitoring for data)
  gzll_rx_start();                                          
  
  for(;;)
  { 
    // If gzpair_host_execute() returns true, a pairing request has been received
    gzp_host_execute();

    // If Host ID request received
    if(gzp_id_req_received())
    {
      // Always grant request
      gzp_id_req_grant();
    }
   
    // If any data received (plaintext on pipe 2 or encrypted through Gazell pairing library)
    if((gzll_get_rx_data_ready_pipe_number() == 2) || (gzp_crypt_user_data_received()))
    {
      // Plaintext data received? 
      if(gzll_rx_fifo_read(payload, NULL, NULL))
      {
        // Write received payload[0] to port 0
        P0 = payload[0];
      }
      else
      {
        // Read data from Gazell pairing library
        gzp_crypt_user_data_read(payload, NULL);
        // Write received payload[0] to port 0
        P0 = payload[0];
      }
    }   
  }          
}

void gzp_usb_init()
{
  // USB HAL initialization
  hal_usb_init(true, device_req_cb, reset_cb, resume_cb, suspend_cb);   
  hal_usb_endpoint_config(0x81, EP1_2_PACKET_SIZE, ep_1_in_cb);  // Configure 32 byte IN endpoint 1
  hal_usb_endpoint_config(0x02, EP1_2_PACKET_SIZE, ep_2_out_cb); // Configure 32 byte OUT endpoint 2	
}

static void app_send_usb_in_data(uint8_t * buf, uint8_t size)
{
  app_wait_while_usb_pending();
  app_pending_usb_write = true;  
  memcpy(usb_in_buf, buf, size);
  hal_usb_send_data(1, usb_in_buf, EP1_2_PACKET_SIZE);
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
