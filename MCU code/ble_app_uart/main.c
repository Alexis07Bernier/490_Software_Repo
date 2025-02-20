#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_delay.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"

#include "ble_bas.h"
#include "ble_cus.h"

#include "sensors.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define DEVICE_NAME "Puck Sensor" /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL 1600      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

//#define APP_ADV_DURATION 18000  /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG 1  /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Minimum acceptable connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS)  /**< Maximum acceptable connection interval (7.5 ms). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 0                               /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define DATA_TIMER_INTERVAL APP_TIMER_TICKS_US(2500)  /**< Sensors timer interval, in ms. */
#define BLE_TIMER_INTERVAL  APP_TIMER_TICKS(20) /**< BLE timer interval, in ms. */

NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
BLE_CUS_DEF(m_cus);

APP_TIMER_DEF(m_data_timer_id); /**< Sensors timer. */
APP_TIMER_DEF(m_ble_timer_id); /**< BLE timer. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static ble_uuid_t m_adv_uuids[] =                        /**< Universally unique service identifiers. */
    {
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};

uint8_t m_data[8*21+2];

static volatile bool m_data_flag = false;
static volatile bool m_ble_flag = false;

static void advertising_start(bool erase_bonds);
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name);
static void pm_evt_handler(pm_evt_t const *p_evt);
static void gap_params_init(void);
static void gatt_init(void);
static void nrf_qwr_error_handler(uint32_t nrf_error);
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void conn_params_init(void);
static void sleep_mode_enter(void);
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context);
static void ble_stack_init(void);
static void peer_manager_init(void);
static void delete_bonds(void);
static void advertising_init(void);
static void buttons_leds_init(bool *p_erase_bonds);
static void log_init(void);
static void power_management_init(void);
static void idle_state_handle(void);

static void leds_states_char_commands_handler(uint8_t const *commands, uint16_t length);
static void cus_evt_handler(ble_cus_t *p_cus, ble_cus_evt_t *p_evt);
static void cus_init(void);
static void qwr_init(void);
static void services_init(void);

static void value_update(void);
static void data_timer_timeout_handler(void *p_context);
static void application_timers_start(void);
static void timers_init(void);

/**@brief Application main function.
 */
int main(void) {
  bool erase_bonds;
  uint8_t index = 0;
  uint8_t m_gyro_value[6], m_mag_value[7];
  uint16_t m_acc_value[3], m_high_gyro_value[1];
  uint16_t m_counter = 0;

  // Initialize.
  nrf_delay_ms(100);
  log_init();
  timers_init();
  buttons_leds_init(&erase_bonds);
  power_management_init();
  ble_stack_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();
  peer_manager_init();
    
  twi_master_init();
  spi_init();

  while (itg3701_init() == false) {
    NRF_LOG_INFO("ITG3701 initialization failed");
  }
  NRF_LOG_INFO("ITG3701 initialization is successful");
  
  while (mmc5983ma_init() == false) {
    NRF_LOG_INFO("MMC5983MA initialization failed");
  }
  NRF_LOG_INFO("MMC5983MA initialization is successful");

  nrf_gpio_cfg_sense_input(28, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
//  nrf_gpio_cfg_output(30);

// Start execution.
  application_timers_start();
  advertising_start(erase_bonds);

// Enter main loop.
  for (;;) {

    if (m_data_flag == true && m_conn_handle != BLE_CONN_HANDLE_INVALID) {
     
     (void) ReadAcc(m_acc_value);
     (void) ReadHighGyro(m_high_gyro_value);
//      NRF_LOG_INFO("ax:%d ay:%d az:%d", m_acc_value[0], m_acc_value[1], m_acc_value[2]);
      
      if (ITG3701_ReadGyro(m_gyro_value) == true)
      {
//        NRF_LOG_INFO("gx:%d gy:%d gz:%d hg:%d", m_gyro_value[0], m_gyro_value[1], m_gyro_value[2], m_high_gyro_value);
      }
      if (MMC5983MA_ReadMag(m_mag_value) == true)
      {
//        NRF_LOG_INFO("mx:%d my:%d mz:%d", m_mag_value[0], m_mag_value[1], m_mag_value[2]);
      }

        for (uint8_t i = 0; i < (sizeof(m_acc_value) / 2); i++)
        {
          m_data[index++] = m_acc_value[i] >> 8;
          m_data[index++] = m_acc_value[i] & 0xFF;
        }
        for (uint8_t i = 0; i < sizeof(m_gyro_value); i++)
        {
          m_data[index++] = m_gyro_value[i];
        }
        for (uint8_t i = 0; i < (sizeof(m_high_gyro_value) / 2); i++)
        {
          m_data[index++] = m_high_gyro_value[i] >> 8;
          m_data[index++] = m_high_gyro_value[i] & 0xFF;
        }
        for (uint8_t i = 0; i < sizeof(m_mag_value); i++)
        {
          m_data[index++] = m_mag_value[i];
        }

//    NRF_LOG_INFO("%d %d %d %d", sizeof(m_acc_value), sizeof(m_gyro_value), sizeof(m_high_gyro_value), sizeof(m_mag_value));
      m_data_flag = false;
//      nrf_gpio_pin_toggle(30);
    }
    
    if (m_ble_flag == true && m_conn_handle != BLE_CONN_HANDLE_INVALID) {
    m_data[index++] = m_counter >> 8;
    m_data[index] = m_counter & 0xFF;
    m_counter++;
    index = 0;
    value_update();
    m_ble_flag = false;

    }
    
    idle_state_handle();
  }
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt) {
  pm_handler_on_pm_evt(p_evt);
//    pm_handler_disconnect_on_sec_failure(p_evt);
  pm_handler_flash_clean(p_evt);

  switch (p_evt->evt_id) {
  case PM_EVT_PEERS_DELETE_SUCCEEDED:
    advertising_start(false);
    break;

  default:
    break;
  }
}

/**@brief Function for updating the value characteristic in Service.
 */
static void value_update(void) {
  ret_code_t err_code = NRF_SUCCESS;

  err_code = ble_cus_value_update(&m_cus, m_data, m_conn_handle);

  if ((err_code != NRF_SUCCESS) &&
      (err_code != NRF_ERROR_INVALID_STATE) &&
      (err_code != NRF_ERROR_RESOURCES) &&
      (err_code != NRF_ERROR_BUSY) &&
      (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
    APP_ERROR_HANDLER(err_code);
  }
}

/**@brief Function for handling the sensor measurement timer timeout.
 *
 * @details This function will be called each time the timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void data_timer_timeout_handler(void *p_context) {
  UNUSED_PARAMETER(p_context);
  m_data_flag = true;
}

static void ble_timer_timeout_handler(void *p_context) {
  UNUSED_PARAMETER(p_context);
  m_ble_flag = true;
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {
  // Initialize timer module.
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  // Create the data timer
  err_code = app_timer_create(&m_data_timer_id,
      APP_TIMER_MODE_REPEATED,
      data_timer_timeout_handler);
  APP_ERROR_CHECK(err_code);

  // Create the BLE timer
  err_code = app_timer_create(&m_ble_timer_id,
      APP_TIMER_MODE_REPEATED,
      ble_timer_timeout_handler);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
  ret_code_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)DEVICE_NAME,
      strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the leds characteristic received data.
 *
 * @param[in]   commands   commands the app sent.
 * @param[in]   length     The length of the commands in bytes.
 */
static void leds_states_char_commands_handler(uint8_t const *commands, uint16_t length) {
  ret_code_t err_code;

  for (uint8_t i = 0; i < length + 1; i++) {
    commands[i] == 0 ? bsp_board_led_off(i) : bsp_board_led_on(i);
  }
}

/**@brief Function for handling the custom Service events.
 *
 * @details This function will be called for all Custom Service ble events which are passed to
 *          the application.
 *
 * @param[in]   ble_cus_t   Custom Service structure.
 * @param[in]   p_evt       Event received from the Custom Service.
 */
static void cus_evt_handler(ble_cus_t *p_cus, ble_cus_evt_t *p_evt) {
  ret_code_t err_code;

  switch (p_evt->evt_type) {
  case BLE_VALUE_CHAR_NOTIFICATIONS_ENABLED:
    NRF_LOG_INFO("acc value char notifications are enabled.");
    break;

  case BLE_VALUE_CHAR_NOTIFICATIONS_DISABLED:
    NRF_LOG_INFO("acc value char notifications are disabled.");
    break;

  default:
    // Do nothing.
    break;
  }
}

/**@brief Function for initializing the Custom ble service.
 */
static void cus_init(void) {
  ret_code_t err_code;
  ble_cus_init_t cus_init = {0};

  cus_init.evt_handler = cus_evt_handler;

  err_code = ble_cus_init(&m_cus, &cus_init);
  APP_ERROR_CHECK(err_code);
}

static void qwr_init(void) {
  ret_code_t err_code;
  nrf_ble_qwr_init_t qwr_init = {0};

  // Initialize Queued Write Module.
  qwr_init.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void) {
  qwr_init();
  cus_init();
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
  ret_code_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
  ret_code_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
 */
static void application_timers_start(void) {
  ret_code_t err_code;

  err_code = app_timer_start(m_data_timer_id, DATA_TIMER_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_start(m_ble_timer_id, BLE_TIMER_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void) {
  ret_code_t err_code;

  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

  // Prepare wakeup buttons.
  err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  ret_code_t err_code;

  switch (ble_adv_evt) {
  case BLE_ADV_EVT_FAST:
    NRF_LOG_INFO("Fast advertising.");
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_ADV_EVT_IDLE:
    sleep_mode_enter();
    break;

  default:
    break;
  }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  ret_code_t err_code = NRF_SUCCESS;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_DISCONNECTED:
    NRF_LOG_INFO("Disconnected.");
    // LED indication will be changed when advertising starts.
    break;

  case BLE_GAP_EVT_CONNECTED:
    NRF_LOG_INFO("Connected.");
    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  } break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    NRF_LOG_DEBUG("GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  default:
    // No implementation needed.
    break;
  }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void) {
  ble_gap_sec_params_t sec_param;
  ret_code_t err_code;

  err_code = pm_init();
  APP_ERROR_CHECK(err_code);

  memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

  // Security parameters to be used for all security procedures.
  sec_param.bond = SEC_PARAM_BOND;
  sec_param.mitm = SEC_PARAM_MITM;
  sec_param.lesc = SEC_PARAM_LESC;
  sec_param.keypress = SEC_PARAM_KEYPRESS;
  sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
  sec_param.oob = SEC_PARAM_OOB;
  sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
  sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
  sec_param.kdist_own.enc = 1;
  sec_param.kdist_own.id = 1;
  sec_param.kdist_peer.enc = 1;
  sec_param.kdist_peer.id = 1;

  err_code = pm_sec_params_set(&sec_param);
  APP_ERROR_CHECK(err_code);

  err_code = pm_register(pm_evt_handler);
  APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void) {
  ret_code_t err_code;

  NRF_LOG_INFO("Erase bonds!");

  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
  ret_code_t err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = true;
  init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.advdata.uuids_complete.p_uuids = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
#ifdef APP_ADV_DURATION
  init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
#endif
  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool *p_erase_bonds) {
  ret_code_t err_code;
  bsp_event_t startup_event;

  err_code = bsp_init(BSP_INIT_LEDS, NULL);
  APP_ERROR_CHECK(err_code);

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds) {
  if (erase_bonds == true) {
    delete_bonds();
    // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
  } else {
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
  }
}