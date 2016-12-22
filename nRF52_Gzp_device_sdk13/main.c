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
 * UNENCRYPTED_DATA_PIPE.
 *
 */
#include "nrf_gzll.h"
#include "nrf_gzp.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_gzll_error.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/
#define UNENCRYPTED_DATA_PIPE     2   ///< Pipes 0 and 1 are reserved for GZP pairing and data. See nrf_gzp.h.
#define NRF_GZLLDE_RXPERIOD_DIV_2 600//504 ///< RXPERIOD/2 on LU1 = timeslot period on nRF5x.

// Ensure that we try all channels before giving up
#define MAX_TX_ATTEMPTS (NRF_GZLL_DEFAULT_TIMESLOTS_PER_CHANNEL_WHEN_DEVICE_OUT_OF_SYNC * \
                         NRF_GZLL_DEFAULT_CHANNEL_TABLE_SIZE)

//GZP
static gzp_id_req_res_t id_req_status   = GZP_ID_RESP_NO_REQUEST;
static uint8_t          data_pipe;
static bool             tx_success      = false;
static bool             send_crypt_data = false;     
static uint8_t          payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; ///< Payload to send to Host. Data and acknowledgement payloads
static uint8_t          ack[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];
static uint32_t         ack_length      = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

//test
static int8_t paired = -2;
static void nRF52_device_wait_host(void);
static void nRF52_device_send_pair_request(void);
static bool nRF52_device_send_crypt_data(void);
static bool nRF52_device_send_data(void);
static void nRF52_device_set_payload(void);
static void nRF52_device_set_pipe(uint8_t pipe);
static void nRF52_device_ack_receive();

/**
 * @brief Function to read the button states.
 *
 * @return Returns the states of buttons.
 */
static uint8_t input_get(void)
{
    uint8_t result = 0;
    for (uint32_t i = 0; i < BUTTONS_NUMBER; i++)
    {
        if (bsp_button_is_pressed(i))
        {
            result |= (1 << i);
        }
    }

    return ~(result);
}


/**
 * @brief Initialize the BSP modules.
 */
static void ui_init(void)
{
    // BSP initialization.
    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS, NULL, NULL);

    APP_ERROR_CHECK(err_code);

    // Set up logger
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Gazell dynamic pairing example. Device mode.\r\n");
    NRF_LOG_FLUSH();
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

static bool nRF52_device_send_crypt_data()
{
	  return gzp_crypt_data_send(payload, GZP_ENCRYPTED_USER_DATA_MAX_LENGTH);
}

static bool nRF52_device_send_data()
{
    bool result = true;
    
    nrf_gzp_reset_tx_complete();
    nrf_gzp_reset_tx_success();

    // Send packet as plain text.
    //if (nrf_gzll_add_packet_to_tx_fifo(UNENCRYPTED_DATA_PIPE, payload, GZP_MAX_FW_PAYLOAD_LENGTH))
    if (nrf_gzll_add_packet_to_tx_fifo(data_pipe, payload, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH))
    {
        while (!nrf_gzp_tx_complete())
        {
            __WFI();
        }
        result = nrf_gzp_tx_success();
        NRF_LOG_INFO("tx success : %d\r\n", tx_success);
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
        payload[i] = i;
    payload[0] = data_pipe;
}

static void nRF52_device_set_pipe(uint8_t pipe)
{
    data_pipe = pipe;	
}

static void nRF52_device_ack_receive()
{
    uint8_t error;
    if (nrf_gzll_fetch_packet_from_rx_fifo(data_pipe, ack, &ack_length))
    {
			  NRF_LOG_INFO("ACK received : %d\r\n", ack[ack_length - 1]);
        NRF_LOG_FLUSH();
    }
    else
    {
        error = nrf_gzll_get_error_code();
        NRF_LOG_INFO("ACK received error is %d\r\n", error);
        NRF_LOG_FLUSH();
    }
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
    // Set up the user interface (buttons and LEDs)
    ui_init();

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

    nRF52_device_set_pipe(2);

    nRF52_device_set_payload();

		for (;;)
    {
        //payload[0] = input_get();
        NRF_LOG_INFO("paired is %d\r\n", paired);
        tx_success = false;

        // Send every other packet as encrypted data.
        if (send_crypt_data)
        {
            if (paired < 0)
                tx_success = false;
						else if (nrf_gzll_get_tx_fifo_packet_count(data_pipe) < 3)
                // Send encrypted packet using the Gazell pairing library.
                tx_success = nRF52_device_send_crypt_data();
        }
        else
        {
            if (paired < 0)
                tx_success = false;
            else if (nrf_gzll_get_tx_fifo_packet_count(data_pipe) < 3)
                tx_success = nRF52_device_send_data();
        }

        // Check if data transfer failed.
        if (!tx_success)
        {
            NRF_LOG_ERROR("Gazelle: transmission failed\r\n");
            NRF_LOG_FLUSH();

            paired = gzp_get_pairing_status();
            if (paired < 0)
                nRF52_device_send_pair_request();
        }
				else if (nrf_gzll_get_rx_fifo_packet_count(data_pipe) > 0)
        {
            nRF52_device_ack_receive();
        }

        nRF52_device_wait_host();

        nrf_delay_ms(1);
    }
}
