#include "spi_haptic.h"
#include <string.h>
#include "APP_ERROR.h"

#define SPI_INSTANCE               0
#define SPI_DATA_LENGTH            20

spi_haptic_t * p_spi_haptic;

static const nrf_drv_spi_t    spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
static       uint8_t          m_tx_buf[SPI_DATA_LENGTH];
static       uint8_t          m_rx_buf[SPI_DATA_LENGTH];
static       uint8_t          tx_length;
static       uint8_t          rx_length;

void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    memset(m_tx_buf, 0, SPI_DATA_LENGTH);
    tx_length = 0;

    rx_length = p_event->data.done.rx_length;
    memcpy(m_rx_buf, p_event->data.done.p_rx_buffer, rx_length);
    p_spi_haptic->evt_handler(m_rx_buf, rx_length);

    memset(m_rx_buf, 0, SPI_DATA_LENGTH);
    rx_length = 0;
}

uint32_t spi_haptic_init(spi_haptic_t * p_haptic, const spi_haptic_init_t * p_haptic_init)
{
    memset(m_tx_buf, 0, SPI_DATA_LENGTH);
    memset(m_rx_buf, 0, SPI_DATA_LENGTH);
    tx_length = 0;
    rx_length = 0;

    p_haptic->evt_handler    = p_haptic_init->evt_handler;
    p_spi_haptic = p_haptic;

    nrf_drv_spi_config_t    spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin      = SPI_SS_PIN;
    spi_config.miso_pin    = SPI_MISO_PIN;
    spi_config.mosi_pin    = SPI_MOSI_PIN;
    spi_config.sck_pin     = SPI_SCK_PIN;

    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));

    return NRF_SUCCESS;
}

void spi_haptic_send(uint8_t * tx_buf, uint8_t length)
{
    tx_length = length;
    memcpy(m_tx_buf, tx_buf, tx_length);
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, tx_length, m_rx_buf, rx_length));
}
