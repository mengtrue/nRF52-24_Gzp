#include "ble_haptic.h"
#include "nrf_log.h"
#include <string.h>
#include "sdk_macros.h"

ble_haptic_t * p_m_haptic;

static uint32_t rumble_char_add(ble_haptic_t * p_haptic, const ble_haptic_init_t * p_haptic_init)
{
    ble_gatts_char_md_t    char_md;
    ble_gatts_attr_md_t    cccd_md;
    ble_gatts_attr_t       attr_char_value;
    ble_uuid_t             ble_uuid;
    ble_gatts_attr_md_t    attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    /* Require security to limit risk of DoS attacks?*/
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_haptic->uuid_type;
    ble_uuid.uuid = BLE_UUID_HAPTIC_RUMBLE_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 1;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_L2CAP_MTU_DEF;

    return sd_ble_gatts_characteristic_add(p_haptic->service_handler,
                                           &char_md,
                                           &attr_char_value,
                                           &p_haptic->rumble_char);
}

uint32_t ble_haptic_init(ble_haptic_t * p_haptic, const ble_haptic_init_t * p_haptic_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t nus_base_uuid = BLE_HAPTIC_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_haptic);
    VERIFY_PARAM_NOT_NULL(p_haptic_init);
    p_m_haptic = p_haptic;
    p_haptic->conn_handler               = BLE_CONN_HANDLE_INVALID;
    p_haptic->evt_handler                = p_haptic_init->evt_handler;
    p_haptic->is_notification_enabled    = false;
    err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_haptic->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_haptic->uuid_type;
    ble_uuid.uuid = BLE_UUID_HAPTIC_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_haptic->service_handler);
    VERIFY_SUCCESS(err_code);

    err_code = rumble_char_add(p_haptic, p_haptic_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

static void on_haptic_cccd_write(ble_haptic_t * p_haptic, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == BLE_CCCD_VALUE_LEN)
    {
        p_haptic->is_notification_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
        
        if (p_haptic->evt_handler != NULL)
        {
            ble_haptic_evt_t evt;
            if (p_haptic->is_notification_enabled)
                evt.type = BLE_HAPTIC_EVT_NOTIFICATION_ENABLED;
            else
                evt.type = BLE_HAPTIC_EVT_NOTIFICATION_DISABLED;
            p_haptic->evt_handler(p_haptic, &evt);
        }
    }
}

static void on_haptic_value_write(ble_haptic_t * p_haptic, ble_gatts_evt_write_t const * p_evt_write)
{

    if (p_haptic->evt_handler != NULL)
    {
        ble_haptic_evt_t evt;
        evt.type = BLE_HAPTIC_EVT_RUMBLE_SEND;
        p_haptic->evt_handler(p_haptic, &evt);
    }
}

static void on_connect(ble_haptic_t * p_haptic, ble_evt_t const * p_ble_evt)
{
    p_haptic->conn_handler = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_haptic_t * p_haptic, ble_evt_t const * p_ble_evt)
{
    if (p_haptic->conn_handler != p_ble_evt->evt.gatts_evt.conn_handle)
        return;
    p_haptic->conn_handler = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_haptic_t * p_haptic, ble_evt_t const * p_ble_evt)
{
    const ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_haptic->rumble_char.cccd_handle)
    {     
        on_haptic_cccd_write(p_haptic, p_evt_write);
    }

    if (p_evt_write->handle == p_haptic->rumble_char.value_handle)
    {
        on_haptic_value_write(p_haptic, p_evt_write);
    }
}

void ble_haptic_on_ble_evt(ble_haptic_t * p_haptic, ble_evt_t * p_ble_evt)
{
    VERIFY_PARAM_NOT_NULL_VOID(p_haptic);
    VERIFY_PARAM_NOT_NULL_VOID(p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
			  case BLE_GAP_EVT_CONNECTED:
            on_connect(p_haptic, p_ble_evt);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_haptic, p_ble_evt);
            break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_haptic, p_ble_evt);
        default:
            break;
    }
}
