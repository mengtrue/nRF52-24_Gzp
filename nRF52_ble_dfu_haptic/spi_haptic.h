#include "nrf_drv_spi.h"

typedef struct spi_haptic_s spi_haptic_t;

typedef void (*spi_haptic_event_handler_t)(uint8_t * rx_buf, uint8_t rx_length);

struct spi_haptic_s {
    spi_haptic_event_handler_t    evt_handler;
};

typedef struct {
    spi_haptic_event_handler_t    evt_handler;
}spi_haptic_init_t;

uint32_t spi_haptic_init(spi_haptic_t * p_haptic, const spi_haptic_init_t * p_haptic_init);
void spi_haptic_send(uint8_t * tx_buf, uint8_t length);
