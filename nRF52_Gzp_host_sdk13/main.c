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
 * $LastChangedRevision: 17930 $
 */


/**
 * This project requires a running counterpart project, which is either of the following:
 *
 * 1) nRF24Lxx device running the gzll_device_w_dynamic_pairing example from the
 * compatible version of the nRFgo SDK
 *
 * 2) nRF5x device running the gzp_device_dynamic_pairing_example example.
 *
 * The application listens for packets continuously, monitoring for pairing
 * requests, as well as normal user data.
 *
 * The Gazell pairing library uses pipe 0 and pipe 1 for encrypted communication.
 * The application will grant any request for a host ID, thus granting pairing.
 * Unencrypted packets can be received on pipe 2.
 *
 * When DATA is received, the contents of the first payload byte
 * are output on GPIO Port LEDS.
 *
 */
#include "nrf_gzll.h"
#include "nrf_gzp.h"
#include "nrf_ecb.h"
#include "bsp.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_gzll_error.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/
#define UNENCRYPTED_DATA_PIPE     2   ///< Pipes 0 and 1 are reserved for GZP pairing and data. See nrf_gzp.h.
#define NRF_GZLLDE_RXPERIOD_DIV_2 600//504 ///< RXPERIOD/2 on LU1 = timeslot period on nRF5x.

#define APP_TIMER_PRESCALER       0   ///< Value of the RTC PRESCALER register.
#define APP_TIMER_OP_QUEUE_SIZE   8u  ///< Size of timer operation queues.

#define LEFT_HAND_PIPE            2
#define RIGHT_HAND_PIPE           3

static uint8_t left_ack[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];
static uint8_t right_ack[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];

static uint8_t hands_number     = 0;
static bool    left_ack_enable  = false;
static bool    right_ack_enable = false;
static uint8_t left_ack_length;
static uint8_t right_ack_length;

// Debug helper variables
static uint32_t length;
// Data and acknowledgement payloads
static uint8_t payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];

static void nRF52_host_exec_receive(void);
static void nRF52_host_set_length(void);
static void nRF52_host_receive(void);
static void nRF52_host_show_payload(void);
static void nRF52_host_set_left_ack(void);
static void nRF52_host_set_right_ack(void);
static void nRF52_host_process_left_ack(void);
static void nRF52_host_process_right_ack(void);

static void nRF52_host_gzll_goto_idle(void);
static void nRF52_host_gzll_tx_fifo_flush(void);
static void nRF52_host_gzll_rx_fifo_flush(void);

/**
 * @brief Initialize the BSP modules.
 */
static void ui_init(void)
{
    // Initialize application timer.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    uint32_t err_code = bsp_init(BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 NULL);
    APP_ERROR_CHECK(err_code);

    // Set up logger
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Gazell dynamic pairing example. Host mode.\r\n");
    NRF_LOG_FLUSH();

    bsp_board_leds_init();
}


/**
 * @brief Function to control LED outputs.
 *
 * @param[in] val Desirable state of the LEDs.
 */
static void output_present(uint8_t val)
{
    uint32_t i;

    for (i = 0; i < LEDS_NUMBER; i++)
    {
        if (val & (1 << i))
        {
            bsp_board_led_on(i);
        }
        else
        {
            bsp_board_led_off(i);
        }
    }
}

static void nRF52_host_gzll_goto_idle()
{
    nrf_gzll_disable();
    while (nrf_gzll_is_enabled())
    {}
}

static void nRF52_host_gzll_tx_fifo_flush()
{
	  uint8_t i;
    for (i = 0; i < NRF_GZLL_CONST_PIPE_COUNT; i++)
        nrf_gzll_flush_tx_fifo(i);
}

static void nRF52_host_gzll_rx_fifo_flush()
{
    uint8_t i;
    for (i = 0; i < NRF_GZLL_CONST_PIPE_COUNT; i++)
        nrf_gzll_flush_tx_fifo(i);
}

static void nRF52_host_exec_receive()
{
    gzp_host_execute();

    // If a Host ID request received
    if (gzp_id_req_received())
    {
        // Always grant a request
        gzp_id_req_grant();
    }
}

static void nRF52_host_set_length()
{
    length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;
    left_ack_length = 15;
    right_ack_length = 15;
}

static void nRF52_host_receive()
{
    hands_number = 0;
    if (nrf_gzll_get_rx_fifo_packet_count(LEFT_HAND_PIPE) || nrf_gzll_get_rx_fifo_packet_count(RIGHT_HAND_PIPE))
    {
        NRF_LOG_INFO("rx fifo received\r\n");
        NRF_LOG_FLUSH();
        if (nrf_gzll_fetch_packet_from_rx_fifo(LEFT_HAND_PIPE, payload, &length))
        {
					  hands_number++;
            output_present(payload[0]);
            NRF_LOG_INFO("receive data : %d\r\n", payload[0]);
            //nRF52_host_show_payload();
            NRF_LOG_FLUSH();

            nRF52_host_process_left_ack();
        }

        if (nrf_gzll_fetch_packet_from_rx_fifo(RIGHT_HAND_PIPE, payload, &length))
        {
					  hands_number++;
            output_present(payload[0]);
            NRF_LOG_INFO("receive data : %d\r\n", payload[0]);
            //nRF52_host_show_payload();
            NRF_LOG_FLUSH();

            nRF52_host_process_right_ack();
        }
    }
    else if (gzp_crypt_user_data_received())
    {
        NRF_LOG_INFO("crypt rx fifo received\r\n");
        NRF_LOG_FLUSH();
        if (gzp_crypt_user_data_read(payload, (uint8_t *)&length))
        {
            hands_number++;
            output_present(payload[0]);
            NRF_LOG_INFO("crypt receive data : %d\r\n", payload[0]);
            NRF_LOG_FLUSH();
        }
    }
}

static void nRF52_host_show_payload()
{
    uint8_t i = 0;
    for (i = 0; i < NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH; i++)
    {
        NRF_LOG_INFO("%d ", payload[i]);
		}
    NRF_LOG_INFO("\r\n");
}

static void nRF52_host_set_left_ack()
{
    uint8_t i = 0;
    for (i = 0; i < left_ack_length; i++)
        left_ack[i] = 2;
    left_ack_enable = true;
}

static void nRF52_host_set_right_ack()
{
    uint8_t i = 0;
    for (i = 0; i < right_ack_length; i++)
        right_ack[i] = 3;
    right_ack_enable = true;
}

static void nRF52_host_process_left_ack()
{
    bool ack_result;
    uint8_t error;
    uint8_t number = nrf_gzll_get_tx_fifo_packet_count(LEFT_HAND_PIPE);          //tx fifo max number is 3
    NRF_LOG_INFO("LEFT PIPE tx fifo packet number is %d\r\n", number);
    NRF_LOG_FLUSH();
    if (left_ack_enable && (number < 3))
    {
        nRF52_host_gzll_goto_idle();
        nRF52_host_gzll_tx_fifo_flush();
        ack_result = nrf_gzll_add_packet_to_tx_fifo(LEFT_HAND_PIPE, left_ack, left_ack_length);
        if (!ack_result)
        {
            error = nrf_gzll_get_error_code();
            nrf_gzll_flush_tx_fifo(LEFT_HAND_PIPE);
            nrf_gzll_flush_rx_fifo(LEFT_HAND_PIPE);
					  NRF_LOG_ERROR("LEFT ACK TX fifo error : %d\r\n", error);
            NRF_LOG_FLUSH();
        }
        else
        {
            left_ack_enable = false;
            memset(left_ack, 0, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH);
        }
        gzll_rx_start();
		}
}

static void nRF52_host_process_right_ack()
{
    bool ack_result;
    uint8_t error;
    uint8_t number = nrf_gzll_get_tx_fifo_packet_count(RIGHT_HAND_PIPE);          //tx fifo max number is 3
    NRF_LOG_INFO("RIGHT PIPE tx fifo packet number is %d\r\n", number);
    NRF_LOG_FLUSH();
    if (right_ack_enable && (number < 3))
    {
        nRF52_host_gzll_goto_idle();
        nRF52_host_gzll_tx_fifo_flush();
        ack_result = nrf_gzll_add_packet_to_tx_fifo(RIGHT_HAND_PIPE, right_ack, right_ack_length);
        if (!ack_result)
        {
            error = nrf_gzll_get_error_code();
            nrf_gzll_flush_tx_fifo(RIGHT_HAND_PIPE);
            nrf_gzll_flush_rx_fifo(RIGHT_HAND_PIPE);
					  NRF_LOG_ERROR("RIGHT ACK TX fifo error : %d\r\n", error);
            NRF_LOG_FLUSH();
        }
        else
        {
            right_ack_enable = false;
            memset(right_ack, 0, NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH);
        }
        gzll_rx_start();
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
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_HOST);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_timeslot_period(NRF_GZLLDE_RXPERIOD_DIV_2); // Half RX period on an nRF24Lxx device
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Initialize the Gazell Pairing Library
    gzp_init();
    result_value = nrf_gzll_set_rx_pipes_enabled(nrf_gzll_get_rx_pipes_enabled() |
                                                 (1 << UNENCRYPTED_DATA_PIPE));
    GAZELLE_ERROR_CODE_CHECK(result_value);

    gzp_pairing_enable(true);

    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);

    nRF52_host_set_left_ack();
    nRF52_host_set_right_ack();

    for (;;)
    {
        if (hands_number != 2)
        {
            nRF52_host_exec_receive();
            nRF52_host_set_length();
        }

        nRF52_host_set_left_ack();
        nRF52_host_set_right_ack();

        nRF52_host_receive();
    }
}


