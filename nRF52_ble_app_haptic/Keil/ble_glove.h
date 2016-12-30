/** @file
 *
 * @defgroup ble_glo Glove Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Glove Service module.
 *
 * @details This module implements the Glove Service, FLAG/REPORT/RUMBLE Characteristics
 *
 * @note The application must propagate BLE stack events to the Glove Service module by calling
 *       ble_bas_on_ble_evt() from the @ref softdevice_handler callback.
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */
#ifndef BLE_GLO_H__
#define BLE_GLO_H__
 
#include <stdint.h>
#include <stdbool.h>
 
#include "ble.h"
 
#ifdef __cplusplus
extern "C" {
#endif

/* use Manus VR demo, make UUID */
#define BLE_UUID_GLOVE_BASE_UUID               {{0x8F, 0xFA, 0x4A, 0xC0, 0xFA, 0x20, 0x11, 0xE4, 0xA1, 0xEC, 0x00, 0x02, 0x00, 0x00, 0xC5, 0x1B}};
#define BLE_UUID_GLOVE_SERVICE                 0x0001
#define BLE_UUID_GLOVE_REPORT_CHARACTERISTIC   0x0002
#define BLE_UUID_GLOVE_FLAG_CHARACTERISTIC     0x0004
#define BLE_UUID_GLOVE_RUMBLE_CHARACTERISTIC   0x0006
 
/* Glove Flag value */
#define GLOVE_FLAGS_HANDEDNESS                 0x1                //0 means left hand, 1 means right hand
#define GLOVE_FLAGS_CAL_GYRO                   0x2                //gyroscope
#define GLOVE_FLAGS_CAL_ACCEL                  0x4                //accelerator
#define GLOVE_FLAGS_CAL_FINGERS                0x8                //fingers

#define GLOVE_SERVICE_MAX_DATA_LENGTH          20                 //Glove Service data max length sent once
#define GLOVE_FLAG_DATA_LENGTH                 1                  //Glove Flag value length, 1 byte
#define GLOVE_REPROT_DATA_LENGTH               20                 //Glove Report data value length, 24 bytes
#define GLOVE_RUMBLE_DATA_LENGTH               3                  //Glove Rumble value length, 2 bytes
 
typedef struct ble_glo_s ble_glo_t;
 
/* Glove Service data handler type */
typedef void (*ble_glo_data_handler_t) (ble_glo_t * p_glo, uint8_t * p_data, uint16_t length);
 
/* Glove Service structure, status information for this service */
struct ble_glo_s
{
 uint8_t                       uuid_type;                                 //16 bit UUID or octers 12-13 of 128 bit UUID
 uint16_t                      service_handle;                            //handle of Glove Service
 uint16_t                      conn_handle;                               //handle of current connection, if not in a connection, BLE_CONN_HANDLE_INVALID
 bool                          is_notification_supported;                 //TRUE if notification of Glove is supported
 ble_gatts_char_handles_t      glo_flag_handles;                          //handles realted to Glove flag characteristic
 ble_gatts_char_handles_t      glo_report_handles;                        //handles realted to Glove report characteristic
 ble_gatts_char_handles_t      glo_rumble_handles;                        //handles realted to Glove rumble characteristic
 ble_glo_data_handler_t        glo_flag_handler;                          //hanlder of glove flag receive by client
 ble_glo_data_handler_t        glo_rumble_handler;                        //handler of glove rumble receive by client
};
 
/* Glove Service init structure, all data needed for initialization of the service */
typedef struct
{
 bool                          support_notification;
 ble_glo_data_handler_t        glo_flag_handler;
 ble_glo_data_handler_t        glo_rumble_handler;
} ble_glo_init_t;
 
/** initializing Glove Service
* [out] p_glo        supplied by app, identify this particular service instance
* [in]  p_glo_init   infomation needed to initialize service
*  return            NRF_SUCCESS  on successful initialization of service, otherwise an error code
*/
uint32_t ble_glo_init(ble_glo_t * p_glo, const ble_glo_init_t * p_glo_init);

/** handling Glove Service's BLE events
* [in] p_glo         Glove Service structure
* [in] p_ble_evt     event received from SoftDevice
*/
void ble_glo_on_ble_evt(ble_glo_t * p_glo, ble_evt_t * p_ble_evt);

/** send glove report data to peer device
* [in]  p_glo        Glove Service
* [in]  p_string     pointer to data needed to send
* [in]  length       length of data needed to send
* return             NRF_SUCCESS  on successful initialization of service, otherwise an error code
*/
uint32_t ble_glo_report_send(ble_glo_t * p_glo, uint8_t * p_string, uint16_t length);
 
 
#ifdef __cplusplus
}
#endif
 
#endif
