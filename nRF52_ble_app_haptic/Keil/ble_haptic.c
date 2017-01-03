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
 */
#include "sdk_config.h"

#include "sdk_common.h"
#include "ble_haptic.h"
#include "ble_hci.h"
#include "ble_srv_common.h"

static uint8_t  haptic_flag_value = 0;                             //default left hand

/* add Glove flag characteristic */
static uint32_t haptic_flag_char_add(ble_haptic_t * p_haptic, const ble_haptic_init_t * p_haptic_init)
{
	ble_uuid_t                ble_uuid;
	ble_gatts_attr_t          attr_char_value;
	ble_gatts_attr_md_t       attr_md;
	ble_gatts_char_md_t       char_md;
	
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read            = 1;                //Characteristic Properties
	char_md.char_props.write           = 1;
	char_md.char_props.write_wo_resp   = 1;
	char_md.p_char_user_desc           = NULL;             //Characteristic descriptor, if not required, NULL
	char_md.p_char_pf                  = NULL;             //presentation format structure, NULL if CPF descriptor is not required
	char_md.p_cccd_md                  = NULL;             //Attribute metadata for Client Characteristic Configuration Descriptor
	char_md.p_sccd_md                  = NULL;             //Attribute metadata for Server Characteristic Configuration Description
	char_md.p_user_desc_md             = NULL;             //Attribute metadata for user description descriptor
	
	ble_uuid.type                      = p_haptic->uuid_type;
	ble_uuid.uuid                      = BLE_UUID_HAPTIC_FLAG_CHARACTERISTIC;
	
	memset(&attr_md, 0, sizeof(attr_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);            //Attribute metadata read permission
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);           //Attribute metadata write permission
	attr_md.vloc                       = BLE_GATTS_VLOC_STACK;     //value location, located in stack memory
	attr_md.vlen                       = 0;                        //variable length attribute
	attr_md.rd_auth                    = 0;                        //read authorization and value will be requested from app on every read operation
	attr_md.wr_auth                    = 0;                        //write authorization will be requested from app on every write operation(not write command)
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.init_len           = sizeof(uint8_t);          //attribute value length in bytes
	attr_char_value.init_offs          = 0;                        //attribute value offset in bytes
	attr_char_value.max_len            = HAPTIC_FLAG_DATA_LENGTH;   //maximum attribute value length
	attr_char_value.p_attr_md          = &attr_md;                 //pointer to the attribute metadata structure
	attr_char_value.p_uuid             = &ble_uuid;                //pointer to the attribute UUID
	attr_char_value.p_value            = &haptic_flag_value;         //pointer to attribute data
	
	return sd_ble_gatts_characteristic_add(p_haptic->service_handle, &char_md, &attr_char_value, &p_haptic->haptic_flag_handles);
}

/* adding the Glove report data characteristic */
static uint32_t haptic_report_char_add(ble_haptic_t * p_haptic, const ble_haptic_init_t * p_haptic_init)
{
	ble_uuid_t                ble_uuid;
	ble_gatts_attr_t          attr_char_value;
	ble_gatts_attr_md_t       attr_md;
	ble_gatts_attr_md_t       cccd_md;
	ble_gatts_char_md_t       char_md;
	
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc                       = BLE_GATTS_VLOC_STACK;
	
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.notify          = 1;
	char_md.char_props.read            = 1;
	char_md.p_char_user_desc           = NULL;
	char_md.p_char_pf                  = NULL;
	char_md.p_user_desc_md             = NULL;
	char_md.p_cccd_md                  = &cccd_md;
	char_md.p_sccd_md                  = NULL;
	
	ble_uuid.type                      = p_haptic->uuid_type;
	ble_uuid.uuid                      = BLE_UUID_HAPTIC_REPORT_CHARACTERISTIC;
	
	memset(&attr_md, 0, sizeof(attr_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc                       = BLE_GATTS_VLOC_STACK;
	attr_md.vlen                       = 0;
	attr_md.rd_auth                    = 0;
	attr_md.wr_auth                    = 0;

  memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.init_len           = sizeof(uint8_t);
	attr_char_value.init_offs          = 0;
	attr_char_value.max_len            = HAPTIC_REPROT_DATA_LENGTH;
	attr_char_value.p_attr_md          = &attr_md;
  attr_char_value.p_uuid             = &ble_uuid;

  return sd_ble_gatts_characteristic_add(p_haptic->service_handle, &char_md, &attr_char_value, &p_haptic->haptic_report_handles);
}

/* adding the Glove rumble data characteristic */
static uint32_t haptic_rumble_char_add(ble_haptic_t * p_haptic, const ble_haptic_init_t * p_haptic_init)
{
	ble_uuid_t                ble_uuid;
	ble_gatts_attr_t          attr_char_value;
	ble_gatts_attr_md_t       attr_md;
	ble_gatts_char_md_t       char_md;
	
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read             = 1;
	char_md.char_props.write            = 1;
	char_md.char_props.write_wo_resp    = 1;
	char_md.p_char_user_desc            = NULL;
	char_md.p_char_pf                   = NULL;
	char_md.p_sccd_md                   = NULL;
	char_md.p_user_desc_md              = NULL;
	
	ble_uuid.type                       = p_haptic->uuid_type;
	ble_uuid.uuid                       = BLE_UUID_HAPTIC_RUMBLE_CHARACTERISTIC;
	
	memset(&attr_md, 0, sizeof(attr_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vlen                        = 0;
	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth                     = 0;
	attr_md.wr_auth                     = 0;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.init_len            = sizeof(uint8_t);
	attr_char_value.init_offs           = 0;
	attr_char_value.max_len             = HAPTIC_RUMBLE_DATA_LENGTH;
	attr_char_value.p_attr_md           = &attr_md;
	attr_char_value.p_uuid              = &ble_uuid;
	
	return sd_ble_gatts_characteristic_add(p_haptic->service_handle, &char_md, &attr_char_value, &p_haptic->haptic_rumble_handles);
}

/* Glove Service initialization function */
uint32_t ble_haptic_init(ble_haptic_t * p_haptic, const ble_haptic_init_t * p_haptic_init)
{
	uint32_t        err_code;
	ble_uuid_t      ble_uuid;
	ble_uuid128_t   haptic_base_uuid = BLE_UUID_HAPTIC_BASE_UUID;
	
	VERIFY_PARAM_NOT_NULL(p_haptic);
	VERIFY_PARAM_NOT_NULL(p_haptic_init);
	
	//init glove service structure
	p_haptic->conn_handle                 = BLE_CONN_HANDLE_INVALID;
	p_haptic->is_notification_supported   = p_haptic_init->support_notification;
	p_haptic->haptic_flag_handler            = p_haptic_init->haptic_flag_handler;
	p_haptic->haptic_rumble_handler          = p_haptic_init->haptic_rumble_handler;
	
	//add custom base UUID
	err_code = sd_ble_uuid_vs_add(&haptic_base_uuid, &p_haptic->uuid_type);
	VERIFY_SUCCESS(err_code);
	
	ble_uuid.type = p_haptic->uuid_type;
	ble_uuid.uuid = BLE_UUID_HAPTIC_SERVICE;
	
	//add Glove Service
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_haptic->service_handle);
	VERIFY_SUCCESS(err_code);
	
	//add Glove Flag characteristic in Glove Service
	err_code = haptic_flag_char_add(p_haptic, p_haptic_init);
	VERIFY_SUCCESS(err_code);
	
	//add Glove Report characteristic in Glove Service
	err_code = haptic_report_char_add(p_haptic, p_haptic_init);
	VERIFY_SUCCESS(err_code);
	//add Glove Rumble characteristic in Glove Service
	err_code = haptic_rumble_char_add(p_haptic, p_haptic_init);
	VERIFY_SUCCESS(err_code);
	
	return NRF_SUCCESS;
}

/* handling BLE_GAP_EVT_CONNECTED event from SoftDevice */
static void on_connect(ble_haptic_t * p_haptic, ble_evt_t * p_ble_evt)
{
	p_haptic->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/* handling BLE_GAP_EVT_DISCONNECTED event from SoftDevice */
static void on_disconnect(ble_haptic_t * p_haptic, ble_evt_t * p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
	p_haptic->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/* handling BLE_GAP_EVT_WRITE event from SoftDevice */
static void on_write(ble_haptic_t * p_haptic, ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	if((p_evt_write->handle == p_haptic->haptic_report_handles.cccd_handle) && (p_evt_write->len == 2))
	{
		if(ble_srv_is_notification_enabled(p_evt_write->data))
		{
			p_haptic->is_notification_supported = true;
		}
		else
		{
			p_haptic->is_notification_supported = false;
		}
	}
  else if ((p_evt_write->handle ==  p_haptic->haptic_flag_handles.value_handle) && (p_haptic->haptic_flag_handler != NULL))
	{
			p_haptic->haptic_flag_handler(p_haptic, p_evt_write->data, p_evt_write->len);
			sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	}
	else if ((p_evt_write->handle ==  p_haptic->haptic_rumble_handles.value_handle) && (p_haptic->haptic_rumble_handler != NULL))
	{
			p_haptic->haptic_rumble_handler(p_haptic, p_evt_write->data, p_evt_write->len);
	}
}
/* handle BLE events */
void ble_haptic_on_ble_evt(ble_haptic_t * p_haptic, ble_evt_t * p_ble_evt)
{
	if((p_haptic == NULL) || (p_ble_evt == NULL))
	{
		return;
	}
	
	switch(p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			on_connect(p_haptic, p_ble_evt);
		  break;
		case BLE_GAP_EVT_DISCONNECTED:
			on_disconnect(p_haptic, p_ble_evt);
		  break;
		case BLE_GATTS_EVT_WRITE:
			on_write(p_haptic, p_ble_evt);
		  break;
		default:
			break;
	}
}

/* send Glove report data to BLE client device */
uint32_t ble_haptic_report_send(ble_haptic_t * p_haptic, uint8_t * p_string, uint16_t length)
{
	ble_gatts_hvx_params_t hvx_params;
	VERIFY_PARAM_NOT_NULL(p_haptic);
	
	if((p_haptic->conn_handle ==  BLE_CONN_HANDLE_INVALID) || (!p_haptic->is_notification_supported))
	{
		return NRF_ERROR_INVALID_STATE;
	}
	if(length > HAPTIC_SERVICE_MAX_DATA_LENGTH)
	{
		return NRF_ERROR_INVALID_PARAM;
	}
	
	memset(&hvx_params, 0, sizeof(hvx_params));
	hvx_params.handle                  = p_haptic->haptic_report_handles.value_handle;
	hvx_params.p_data                  = p_string;
	hvx_params.p_len                   = &length;
	hvx_params.type                    = BLE_GATT_HVX_NOTIFICATION;
	
	return sd_ble_gatts_hvx(p_haptic->conn_handle, &hvx_params);
}
