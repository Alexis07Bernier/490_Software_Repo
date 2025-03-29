#include "ble_cus.h"

/**@brief Function for initializing the Custom ble service.
 *
 * @param[in]   p_cus       Custom service structure.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_cus_init(ble_cus_t *p_cus, const ble_cus_init_t *p_cus_init) {
  
  uint32_t err_code;
  ble_uuid_t ble_uuid;
  ble_add_char_params_t add_char_params;

  /* Adding the service */

  // Initialize service structure.
  p_cus->evt_handler = p_cus_init->evt_handler;
  p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;

  // Add the Custom ble Service UUID
  ble_uuid128_t base_uuid = CUS_SERVICE_UUID_BASE;
  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = CUS_SERVICE_UUID;

  // Add the service to the database
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  /* Adding the service characteristics */

  // Add the characteristic.
  uint8_t char_init_value[NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3] = {0};

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = VALUE_CHAR_UUID;
  add_char_params.uuid_type = p_cus->uuid_type;

  add_char_params.init_len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3;
  add_char_params.max_len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3;
  add_char_params.p_init_value = char_init_value;

  add_char_params.char_props.read = 1;
  add_char_params.char_props.notify = 1;

  add_char_params.read_access = SEC_OPEN;
  add_char_params.cccd_write_access = SEC_OPEN;

  err_code = characteristic_add(p_cus->service_handle,
      &add_char_params,
      &p_cus->value_char_handles);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  return NRF_SUCCESS;
}

/**@brief Function for updating the value on the ble characteristic.
 *
 * @param[in]   p_cus             Custom service structure.
 * @param[in]   p_value           value.
 * @param[in]   conn_handle       Connection handle.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_cus_value_update(ble_cus_t * p_cus, uint8_t * p_value, uint16_t conn_handle) {
  ble_gatts_hvx_params_t params;
//  uint16_t len = sizeof(*p_value);
  uint16_t len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3;

  memset(&params, 0, sizeof(params));
  params.type = BLE_GATT_HVX_NOTIFICATION;
  params.handle = p_cus->value_char_handles.value_handle;
  params.p_data = p_value;
  params.p_len = &len;

  return sd_ble_gatts_hvx(conn_handle, &params);
}
