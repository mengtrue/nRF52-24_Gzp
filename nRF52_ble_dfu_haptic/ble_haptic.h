#include <stdint.h>
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_UUID_HAPTIC_SERVICE                 0x0001
#define BLE_UUID_HAPTIC_REPORT_CHARACTERISTIC   0x0002
#define BLE_UUID_HAPTIC_FLAG_CHARACTERISTIC     0x0004
#define BLE_UUID_HAPTIC_RUMBLE_CHARACTERISTIC   0x0006
#define BLE_HAPTIC_BASE_UUID               {{0x8F, 0xFA, 0x4A, 0xC0, 0xFA, 0x20, 0x11, 0xE4, 0xA1, 0xEC, 0x00, 0x02, 0x00, 0x00, 0xC5, 0x1B}};

typedef enum {
    BLE_HAPTIC_EVT_RUMBLE_SEND,
    BLE_HAPTIC_EVT_NOTIFICATION_ENABLED,
    BLE_HAPTIC_EVT_NOTIFICATION_DISABLED
}ble_haptic_evt_type_t;

typedef struct {
    ble_haptic_evt_type_t type;
} ble_haptic_evt_t;

typedef struct ble_haptic_s ble_haptic_t;

typedef void (*ble_haptic_evt_handler_t)(ble_haptic_t * p_haptic, ble_haptic_evt_t * p_evt);
typedef void (*ble_haptic_rumble_data_handler_t)(ble_haptic_t * p_haptic, uint8_t * p_data, uint16_t length);

struct ble_haptic_s {
    uint8_t                             uuid_type;                     // UUID type for haptic UUID
    uint16_t                            service_handler;               // SoftDevice provide to handle of haptic
    uint16_t                            conn_handler;
    bool                                is_notification_enabled;
    ble_gatts_char_handles_t            rumble_char;
    ble_haptic_evt_handler_t            evt_handler;
    ble_haptic_rumble_data_handler_t    rumble_data_handler;
};

typedef struct {
    ble_haptic_evt_handler_t            evt_handler;
    ble_haptic_rumble_data_handler_t    rumble_data_handler;
}ble_haptic_init_t;

uint32_t ble_haptic_init(ble_haptic_t * p_haptic, const ble_haptic_init_t * p_haptic_init);

void ble_haptic_on_ble_evt(ble_haptic_t * p_haptic, ble_evt_t * p_ble_evt);


#ifdef __cplusplus
}
#endif
