/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 25977 $
 */


/**
 * This project requires a running counterpart project, which is either of the following:
 *
 * 1) nRF24Lxx host running the gzll_host_w_dynamic_pairing example from the
 * compatible version of the nRFgo SDK
 *
 * 2) nRF5x host running the gzp_host_dynamic_pairing_example example.
 *
 *
 * The application sends packets continuously. If a packet transmission
 * fails (either times out or encryption fails), the device will attempt pairing
 * to a host by sending a pairing request, consisting of an "address request" and a
 * "Host ID" request.
 *
 * If the device is paired to a host, pairing data will be stored in non
 * volatile memory.
 *
 * Before adding a packet to the TX queue, the contents of
 * the GPIO Port BUTTONS are copied to the first payload byte (byte 0).
 * When an ACK is received, the contents of the first payload byte of
 * the ACK are output on GPIO Port LEDS.
 *
 * The application alternates between sending the packets encrypted
 * through the pairing library or directly as plain text using pipe
 * UNENCRYPTED_gzp_data_pipe.
 *
 */
#include "nrf_gzll.h"
#include "nrf_gzp.h"
#include "app_error.h"
#include "nrf_gzll_error.h"
#include "nrf_delay.h"

#include "nrf_drv_spis.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/
//< Pipes 0 and 1 are reserved for GZP pairing and data. See nrf_gzp.h.
#define NRF_GZLLDE_RXPERIOD_DIV_2 600//504 ///< RXPERIOD/2 on LU1 = timeslot period on nRF5x.

// Ensure that we try all channels before giving up
#define MAX_TX_ATTEMPTS (NRF_GZLL_DEFAULT_TIMESLOTS_PER_CHANNEL_WHEN_DEVICE_OUT_OF_SYNC * \
                         NRF_GZLL_DEFAULT_CHANNEL_TABLE_SIZE)

//GZP
static gzp_id_req_res_t id_req_status              = GZP_ID_RESP_NO_REQUEST;
static uint8_t          gzp_data_pipe              = 0;
static volatile bool    gzp_tx_success             = false;    
static uint8_t          gzp_tx_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; ///< Payload to send to Host. Data and acknowledgement payloads
static uint8_t          gzp_rx_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];
static uint32_t         gzp_rx_payload_length      = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

static int8_t           gzp_paired                 = -2;
static volatile bool    gzp_to_send                = false;
static uint8_t          gzp_packet_sum_number      = 1;
static uint8_t          gzp_packet_index           = 1;

static void nRF52_device_gzp_init(void);
static void nRF52_device_wait_host(void);
static void nRF52_device_send_pair_request(void);
static bool nRF52_device_send_data(void);
static void nRF52_device_set_payload(void);
static void nRF52_device_set_pipe(uint8_t pipe);
static void nRF52_device_ack_receive(void);
static void nRF52_device_gzp_process(void);

//SPI
#define SPIS_INSTANCE           1
#define SPIS_RX_LENGTH          100
#define NORDIC_FIFO_LENGTH      256
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);

static volatile bool    spis_xfer_done             = false;
static volatile bool    fifo_read                  = false;
static volatile bool    fifo_write                 = false;
static volatile bool    gzp_spis_need              = false;

static uint8_t          gzp_spis_index             = 0;
static uint8_t          spis_rx_sensor_length      = 0;
static uint16_t         spis_fifo_write_pos        = 0;
static uint8_t          m_spis_buffer[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];
static uint8_t          m_spis_tx_buf[SPIS_RX_LENGTH];
static uint8_t          m_spis_rx_buf[SPIS_RX_LENGTH];
static uint8_t          spis_fifo[NORDIC_FIFO_LENGTH];

static void spis_event_handle(nrf_drv_spis_event_t event);
static void spis_init(void);
static void spis_to_fifo(void);
static void spis_fifo_to_gzp_tx(void);
static void spis_fifo_send_gzp(void);
static void spis_process(void);

static bool checkGzpCRC(uint8_t * data, uint8_t size);
static void copyGZPTospis(void);

static bool checkGzpCRC(uint8_t * data, uint8_t size)
{
    uint8_t value = data[size + 1];
    uint8_t result = 0;
    uint8_t i;

    if (value == 0)
        return false;

    for (i = 0; i <= size; i++)
        result = result ^ data[i];
    NRF_LOG_INFO("CRC value is %d, now check value is %d\r\n", value, result);

    if (result == value)
        return true;
    else
        return false;
}

static void copyGZPTospis()
{
    uint8_t len = gzp_rx_payload[2];
    uint8_t i = 0;
    for (i = 0; i < 2; i++)
        m_spis_buffer[i] = gzp_rx_payload[i];
    for (i = 2; i < len; i++)
        m_spis_buffer[i] = gzp_rx_payload[i+1];
}

static void nRF52_device_gzp_init()
{
    // Initialize the Gazell Link Layer
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Attempt sending every packet up to MAX_TX_ATTEMPTS times
    result_value = nrf_gzll_set_max_tx_attempts(MAX_TX_ATTEMPTS);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_timeslot_period(NRF_GZLLDE_RXPERIOD_DIV_2); // Half RX period on nRF24Lxx device
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Erase pairing data. This example is intended to demonstrate pairing after every reset.
    // See the gzp_desktop_emulator example for a demonstration on how to maintain pairing data between resets.
    gzp_erase_pairing_data();

    // Initialize the Gazell Pairing Library
    gzp_init();

    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);

    memset(gzp_tx_payload, 0, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH);
    memset(gzp_rx_payload, 0, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH);
}

static void nRF52_device_wait_host()
{
    // If waiting for the host to grant or reject an ID request.
    if (id_req_status == GZP_ID_RESP_PENDING)
    {
        // Send a new ID request for fetching response.
        id_req_status = gzp_id_req_send();
    }
}

static void nRF52_device_send_pair_request()
{
    // Send "system address request". Needed for sending any user data to the host.
    if (gzp_address_req_send())
    {
        // Send "Host ID request". Needed for sending encrypted user data to the host.
        id_req_status = gzp_id_req_send();
    }
    else
    {
        // System address request failed.
        NRF_LOG_ERROR("address req send fail\r\n");
        NRF_LOG_FLUSH();
    }
}

static bool nRF52_device_send_data()
{
    bool result = true;
    
    nrf_gzp_reset_tx_complete();
    nrf_gzp_reset_tx_success();

    spis_fifo_send_gzp();

    // Send packet as plain text.
    if (nrf_gzll_add_packet_to_tx_fifo(gzp_data_pipe, gzp_tx_payload, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH))
    {
        while (!nrf_gzp_tx_complete())
        {
            __WFI();
        }
        result = nrf_gzp_tx_success();
        NRF_LOG_INFO("gzp tx success : %d\r\n", gzp_tx_success);
        NRF_LOG_FLUSH();
    }
    else
    {
        NRF_LOG_ERROR("TX fifo error \r\n");
        NRF_LOG_FLUSH();
    }
    return result;
}

static void nRF52_device_set_payload()
{
    uint8_t i = 0;
    for (i = 0; i < NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH; i++)
        gzp_tx_payload[i] = i;
    gzp_tx_payload[0] = gzp_data_pipe;
}

static void nRF52_device_set_pipe(uint8_t pipe)
{
    gzp_data_pipe = pipe;	
}

static void nRF52_device_ack_receive()
{
    uint8_t error;
    if (nrf_gzll_fetch_packet_from_rx_fifo(gzp_data_pipe, gzp_rx_payload, &gzp_rx_payload_length))
    {
			  NRF_LOG_INFO("GZP ACK received : %d\r\n", gzp_rx_payload[gzp_rx_payload_length - 1]);
        NRF_LOG_FLUSH();

        if (checkGzpCRC(gzp_rx_payload, gzp_rx_payload[2]) == true)
        {
            copyGZPTospis();
            gzp_spis_need = true;
            memset(gzp_rx_payload, 0, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH);
        }
    }
    else
    {
        error = nrf_gzll_get_error_code();
        NRF_LOG_INFO("GZP ACK received error is %d\r\n", error);
        NRF_LOG_FLUSH();
    }
}

static void spis_to_fifo()
{
    for (uint8_t i = 0; i < spis_rx_sensor_length; i++)
    {
        spis_fifo[spis_fifo_write_pos + i] = m_spis_rx_buf[i];
    }
    spis_fifo_write_pos += spis_rx_sensor_length;
}

static void spis_fifo_to_gzp_tx()
{
    uint8_t i;
    uint8_t data[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH] = {spis_rx_sensor_length, gzp_packet_index};
    uint8_t start = (gzp_packet_index - 1) * NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    for (i = 0; i < NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH && (i < spis_rx_sensor_length - start); i++)
        data[i + 2] = spis_fifo[start + 1];

    if (gzp_packet_index == gzp_packet_sum_number)
    {
        while (fifo_write == true)
            nrf_delay_us(100);
        fifo_read = true;
        spis_fifo_write_pos -= spis_rx_sensor_length;
        fifo_read = false;
		}

    memcpy(gzp_tx_payload, data, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH);
}

static void spis_fifo_send_gzp()
{
    if (gzp_packet_sum_number == 1)
    {
        memcpy(gzp_tx_payload, m_spis_rx_buf, spis_rx_sensor_length);
        memset(m_spis_rx_buf, 0, NORDIC_FIFO_LENGTH);
    }
    else
    {
        spis_fifo_to_gzp_tx();
    }

    if (gzp_packet_index < gzp_packet_sum_number)
        gzp_packet_index++;
    else
        gzp_packet_index = 1;
}

static void spis_event_handle(nrf_drv_spis_event_t event)
{
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {        
        spis_rx_sensor_length = event.rx_amount;
			  NRF_LOG_INFO("SPIS Transfer completed. Received length : %d\r\n", spis_rx_sensor_length);
        if (spis_rx_sensor_length > NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH)
        {
            while (spis_fifo_write_pos > NORDIC_FIFO_LENGTH - spis_rx_sensor_length || fifo_read == true)
                nrf_delay_us(100);
            if (gzp_data_pipe == 0)
                gzp_data_pipe = m_spis_rx_buf[0];
            fifo_write = true;
            spis_to_fifo();
            fifo_write = false;

            if (gzp_to_send == false)
            {
                gzp_to_send = true;
                if (spis_rx_sensor_length % (NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH - 2) != 0)
                    gzp_packet_sum_number = spis_rx_sensor_length / (NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH - 2) + 1;
                else
                    gzp_packet_sum_number = spis_rx_sensor_length / (NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH - 2);
                gzp_packet_index = 1;
						}
				}
        else
        {
            gzp_packet_sum_number = 1;
            gzp_packet_index      = 1;
            gzp_to_send           = true;
        }

        spis_xfer_done = true;
        memset(m_spis_rx_buf, 0, SPIS_RX_LENGTH);

        if (gzp_spis_need && gzp_spis_index == 0)
        {
            gzp_spis_index = 1;
            memcpy(m_spis_tx_buf, m_spis_buffer, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH);
        }
        else if (gzp_spis_index == 2)
        {
            gzp_spis_index = 0;
            gzp_spis_need = false;
            memset(m_spis_buffer, 0, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH);
            memset(m_spis_tx_buf, 0, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH);
				}

        if (gzp_spis_index == 1)
            gzp_spis_index++;
        
	  }
}

static void spis_init()
{
    memset(m_spis_rx_buf, 0, SPIS_RX_LENGTH);
    memset(m_spis_tx_buf, 0, SPIS_RX_LENGTH);
    memset(spis_fifo, 0, NORDIC_FIFO_LENGTH);

    nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spis_config.csn_pin               = APP_SPIS_CS_PIN;
    spis_config.miso_pin              = APP_SPIS_MISO_PIN;
    spis_config.mosi_pin              = APP_SPIS_MOSI_PIN;
    spis_config.sck_pin               = APP_SPIS_SCK_PIN;

    APP_ERROR_CHECK(nrf_drv_spis_init(&spis, &spis_config, spis_event_handle));
}

static void nRF52_device_gzp_process()
{
    if (gzp_to_send == true)
    {
        NRF_LOG_INFO("gzp paired is %d\r\n", gzp_paired);
        gzp_tx_success = false;

        if (gzp_paired < 0)
            gzp_tx_success = false;
        else if (nrf_gzll_get_tx_fifo_packet_count(gzp_data_pipe) < 3)
            gzp_tx_success = nRF52_device_send_data();

        if (spis_fifo_write_pos == 0)
            gzp_to_send = false;

        // Check if data transfer failed.
        if (!gzp_tx_success)
        {
            NRF_LOG_ERROR("Gazelle: transmission failed\r\n");
            NRF_LOG_FLUSH();

            gzp_paired = gzp_get_pairing_status();
            if (gzp_paired < 0)
                nRF52_device_send_pair_request();
        }
				else
        {
            if (nrf_gzll_get_rx_fifo_packet_count(gzp_data_pipe) > 0)
            {
                nRF52_device_ack_receive();
            }
				}

        nRF52_device_wait_host();
    }
}

static void spis_process()
{
    spis_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_spis_tx_buf, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH, m_spis_rx_buf, SPIS_RX_LENGTH));
}

/*****************************************************************************/
/**
 * @brief Main function.
 *
 * @return ANSI required int return type.
 */
/*****************************************************************************/
int main(void)
{
    spis_init();

    nRF52_device_gzp_init();

		while (true)
    {
			  NRF_LOG_INFO("while 1 spis_xfer_done : %d, gzp_to_send : %d\r\n", spis_xfer_done, gzp_to_send);

        nRF52_device_gzp_process();

        spis_process();

        NRF_LOG_INFO("while 2 spis_xfer_done : %d, gzp_to_send : %d\r\n", spis_xfer_done, gzp_to_send);
        while (!spis_xfer_done && !gzp_to_send)
        {
            __WFE();
        }
    }
}
