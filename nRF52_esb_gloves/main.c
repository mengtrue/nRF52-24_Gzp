/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_drv_spis.h"

#define SPIS_INSTANCE 1 /**< SPIS instance index. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
static volatile bool spis_xfer_done;

//static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);
static nrf_esb_payload_t        tx_payload;
static nrf_esb_payload_t        rx_payload;

const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;

#define NORDIC_FIFO_LENGTH                             256            //Nordic nRF52832 spis fifo size
#define HID_TO_SENSOR_LENGTH                           14             //Package data length by HID sent to Glove
#define NORDIC_SPIS_RX_LENGTH                          100            //default spis rx data length
#define NORDIC_ESB_PACKET_LENGTH                       32             //ESB max packet length
#define NORDIC_ESB_PAYLOAD_LENGTH                      30             //ESB each packet payload length

static uint8_t SENSOR_PACKET_LENGTH;                                  //Package data length by Glove Sensor sent
static uint8_t ESB_PACKET_SUM_NUMBER;                                 //every sensor packet into esb packet numbers
static uint8_t ESB_PACKET_NUMBER;                                     //esb packet index

static uint8_t spis_rx_data[NORDIC_SPIS_RX_LENGTH];                  //spis receive data
static uint8_t spis_tx_data[HID_TO_SENSOR_LENGTH];                    //spis transfer data

static uint8_t spis_fifo[NORDIC_FIFO_LENGTH];                         //spis receive data fifo
static uint16_t spis_fifo_write_pos;                                  //spis receive data fifo write position

static volatile bool fifo_write;                                      //when writting to spis fifo, TRUE
static volatile bool fifo_read;                                       //when spis fifo reading, TRUE
static volatile bool esb_continue;                                    //esb is running

static uint8_t hand;

static uint8_t spis_buffer[NORDIC_ESB_PACKET_LENGTH];
static bool hid_esb_spis_need = false;
static uint8_t spis_index = 0;
static bool esbCRC(uint8_t * data, uint8_t size);
static void copyESBToSpis();

void init_spis_fifo()
{
	  memset(spis_fifo, 0, NORDIC_FIFO_LENGTH);
	  spis_fifo_write_pos = 0;
}	  

void spis_to_fifo()
{
	  if (spis_fifo_write_pos <= NORDIC_FIFO_LENGTH - SENSOR_PACKET_LENGTH)
		{
			  for (int i = 0; i < SENSOR_PACKET_LENGTH; i++)
			  {
					  spis_fifo[spis_fifo_write_pos+i] = spis_rx_data[i];
				}
				spis_fifo_write_pos += SENSOR_PACKET_LENGTH;
		}

}

void spis_put_esb_tx()
{
	  uint8_t data[NORDIC_ESB_PACKET_LENGTH] = {SENSOR_PACKET_LENGTH, ESB_PACKET_NUMBER, hand};
		uint8_t start = (ESB_PACKET_NUMBER == 1) ? 0 : (ESB_PACKET_NUMBER - 1) * NORDIC_ESB_PAYLOAD_LENGTH - 1;
		uint8_t i;
		
		for(i = 0; i < NORDIC_ESB_PAYLOAD_LENGTH - 1 && (i < SENSOR_PACKET_LENGTH - start); i++)
		{
			  data[i+3] = spis_fifo[start+i];
		}
		
		if (ESB_PACKET_NUMBER == ESB_PACKET_SUM_NUMBER)
		{
			  while(fifo_write == true)
				{
					  nrf_delay_us(100);
				}
				fifo_read = true;
				spis_fifo_write_pos -= SENSOR_PACKET_LENGTH;
				fifo_read = false;
				if (spis_fifo_write_pos == 0)
					  esb_continue = false;
		}
		memcpy(tx_payload.data, data, NORDIC_ESB_PACKET_LENGTH);
}

void nrf_esb_error_handler(uint32_t err_code, uint32_t line)
{
    NRF_LOG_ERROR("App failed at line %d with error code: 0x%08x\r\n",
                   line, err_code);
#if DEBUG //lint -e553
    while (true);
#else
    NVIC_SystemReset();
#endif

}

#define ESB_ERROR_CHECK(err_code) if (err_code) nrf_esb_error_handler(err_code, __LINE__);

/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */

void spis_fifo_send_esb()
{
	  if (ESB_PACKET_SUM_NUMBER == 1)
		{
			  memcpy(tx_payload.data, spis_rx_data, SENSOR_PACKET_LENGTH);
			  memset(spis_rx_data, 0, NORDIC_SPIS_RX_LENGTH);
		}
		else
		{
			  spis_put_esb_tx();
		}
		tx_payload.noack = false;
		nrf_esb_write_payload(&tx_payload);
		//if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
		//{
			  if (ESB_PACKET_NUMBER < ESB_PACKET_SUM_NUMBER)
				{
			      ESB_PACKET_NUMBER++;
				} else
				    ESB_PACKET_NUMBER = 1;
		//}
}

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("ESB TX SUCCESS EVENT\r\n");
				    if (esb_continue)
						{
							  spis_fifo_send_esb();
						}
            break;
        case NRF_ESB_EVENT_TX_FAILED:
					  NRF_LOG_DEBUG("ESB TX FAILED EVENT\r\n", esb_continue);				    
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
				    if (esb_continue)
						{
							  spis_fifo_send_esb();
						}
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            //NRF_LOG_DEBUG("RX RECEIVED EVENT\r\n");
            if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length > 0)
                {
									  NRF_LOG_DEBUG("RX RECEIVED PAYLOAD\r\n");
									  if (esbCRC(rx_payload.data, rx_payload.data[2]) == true)
									  //if (hid_esb_spis_need == false)
										{
											  //memcpy(spis_tx_data, rx_payload.data, NORDIC_ESB_PACKET_LENGTH);
									      //memcpy(spis_buffer, rx_payload.data, NORDIC_ESB_PACKET_LENGTH);
											  copyESBToSpis();
									      hid_esb_spis_need = true;
									      memset(rx_payload.data, 0, NORDIC_ESB_PACKET_LENGTH);
										}
									  
                }
            }
            break;
    }
    NRF_GPIO->OUTCLR = 0xFUL << 12;
    NRF_GPIO->OUTSET = (p_event->tx_attempts & 0x0F) << 12;
}

void spis_event_handler(nrf_drv_spis_event_t event)
{
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {        
			  SENSOR_PACKET_LENGTH = event.rx_amount;
			  NRF_LOG_INFO("spis rx length: %d\r\n", SENSOR_PACKET_LENGTH);
				if (SENSOR_PACKET_LENGTH > NRF_ESB_MAX_PAYLOAD_LENGTH)
				{
					  while (spis_fifo_write_pos > NORDIC_FIFO_LENGTH - SENSOR_PACKET_LENGTH || fifo_read == true)
				    {
					      nrf_delay_us(100);
				    }
						hand = spis_rx_data[0];
				    fifo_write     = true;
				    spis_to_fifo();
				    fifo_write     = false;
						
						if (esb_continue == false)
				    {
					      esb_continue = true;
							
					      if (SENSOR_PACKET_LENGTH % (NRF_ESB_MAX_PAYLOAD_LENGTH - 2) != 0)
									ESB_PACKET_SUM_NUMBER = SENSOR_PACKET_LENGTH / (NRF_ESB_MAX_PAYLOAD_LENGTH - 2) + 1;
								else
									ESB_PACKET_SUM_NUMBER = SENSOR_PACKET_LENGTH / (NRF_ESB_MAX_PAYLOAD_LENGTH - 2);
								ESB_PACKET_NUMBER = 1;
						}
				}
				else
				{
				    ESB_PACKET_SUM_NUMBER = 1;
            ESB_PACKET_NUMBER = 1;					
				}						
				//ESB_PACKET_NUMBER = 1;
				spis_fifo_send_esb();
				
				spis_xfer_done = true;
			  memset(spis_rx_data, 0, NORDIC_SPIS_RX_LENGTH);
				
				if (hid_esb_spis_need && spis_index == 0)
				{
					  spis_index = 1;
					  memcpy(spis_tx_data, spis_buffer, NORDIC_ESB_PACKET_LENGTH);
					  NRF_LOG_INFO("spis tx data is %d %d %d %d\r\n", spis_tx_data[0], spis_tx_data[1], spis_tx_data[2], spis_tx_data[3]);
				}
				else if (spis_index == 2)
				{		
					  spis_index = 0;
					  hid_esb_spis_need = false;
				    memset(spis_buffer, 0, NORDIC_ESB_PACKET_LENGTH);
					  memset(spis_tx_data, 0, NORDIC_ESB_PACKET_LENGTH);
				}
				
				if (spis_index == 1)
				    spis_index++;
    }
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
    nrf_gpio_range_cfg_output(8, 15);
    LEDS_CONFIGURE(LEDS_MASK);
}

void spis_init( void )
{
	  init_spis_fifo();
	
	  nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spis_config.csn_pin               = APP_SPIS_CS_PIN;
    spis_config.miso_pin              = APP_SPIS_MISO_PIN;
    spis_config.mosi_pin              = APP_SPIS_MOSI_PIN;
    spis_config.sck_pin               = APP_SPIS_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spis_init(&spis, &spis_config, spis_event_handler));
	
	  memset(spis_rx_data, 0, NORDIC_SPIS_RX_LENGTH);
	  memset(spis_tx_data, 0, NORDIC_ESB_PACKET_LENGTH);
	  memset(spis_buffer, 0, NORDIC_ESB_PACKET_LENGTH);
}

uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };
		
		tx_payload.pipe    = 0;
    tx_payload.length  = NRF_ESB_MAX_PAYLOAD_LENGTH;

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.retransmit_count         = 1;
    nrf_esb_config.bitrate                  = RADIO_MODE_MODE_Nrf_2Mbit;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;
	  nrf_esb_config.payload_length           = NRF_ESB_MAX_PAYLOAD_LENGTH;
	  nrf_esb_config.crc                      = RADIO_CRCCNF_LEN_Two;
		
    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);
		
		err_code = nrf_esb_set_rf_channel(2);
		VERIFY_SUCCESS(err_code);
		
		err_code = nrf_esb_set_tx_power(NRF_ESB_TX_POWER_4DBM);
		VERIFY_SUCCESS(err_code);

    return err_code;
}

static bool esbCRC(uint8_t * data, uint8_t size)
{
	  uint8_t value = data[size+1];
	  uint8_t result = 0;
	  uint8_t i;
	
	  if (value == 0)
		    return false;
		
	  for(i = 0; i <= size; i++)
	  {
			  result = result ^ data[i];
		}
		
		NRF_LOG_INFO("CRC value is %d, now value is %d\r\n", value, result);
		
		if (result == value)
			  return true;
		else
			  return false;
}

static void copyESBToSpis()
{
	  uint8_t len = rx_payload.data[2];
	  uint8_t i = 0;
	  for (i = 0; i < 2; i++)
	      spis_buffer[i] = rx_payload.data[i];
	  for (i = 2; i < len; i++)
	      spis_buffer[i] = rx_payload.data[i+1];
}

int main(void)
{
    ret_code_t err_code;

    gpio_init();

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    clocks_start();

    err_code = esb_init();
    ESB_ERROR_CHECK(err_code);

    spis_init();

    LEDS_CONFIGURE(LEDS_MASK);

    NRF_LOG_DEBUG("Enhanced ShockBurst Transmitter Example running.\r\n");

    while (true)
    {
        spis_xfer_done = false;
			  APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, spis_tx_data, NORDIC_ESB_PACKET_LENGTH, spis_rx_data, NORDIC_SPIS_RX_LENGTH));
        while (!spis_xfer_done)
        {
            __WFE();
        }
        /*NRF_LOG_DEBUG("Transmitting packet %02x\r\n", tx_payload.data[1]);

        tx_payload.noack = false;
        if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
        {
            // Toggle one of the LEDs.
            nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
            nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
            nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
            nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));
            tx_payload.data[1]++;
        }
        else
        {
            NRF_LOG_WARNING("Sending packet failed\r\n");
        }*/
				//nrf_delay_ms(15);
        NRF_LOG_FLUSH();
    }
}
/*lint -restore */
