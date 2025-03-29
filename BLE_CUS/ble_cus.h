#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "sdk_common.h"
#include "app_error.h"

#define BLE_CUS_BLE_OBSERVER_PRIO 2

/**@brief   Macro for defining a ble_cus instance.
 *
 * @param   _name   Name of the instance.
 */
#define BLE_CUS_DEF(_name)  static ble_cus_t  _name;

#define CUS_SERVICE_UUID_BASE     {0x0e, 0x73, 0xf2, 0xf5, 0xd7, 0x05, 0xe6, 0x80,\
                                   0x3f, 0x4e, 0x5b, 0x11, 0xd0, 0x03, 0x36, 0x0a}

#define CUS_SERVICE_UUID          0x1000

#define VALUE_CHAR_UUID       0x1001


/**@brief Custom service event types.
 *
 */
typedef enum
{
  BLE_VALUE_CHAR_NOTIFICATIONS_ENABLED,
  BLE_VALUE_CHAR_NOTIFICATIONS_DISABLED,
} ble_cus_evt_type_t;


typedef struct
{
   ble_cus_evt_type_t evt_type;                     
} ble_cus_evt_t;

/**@brief Forward declaration of the ble_cus_t type. */
typedef struct ble_cus_s ble_cus_t;

/**@brief Custom service event handler type. */
typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_cus, ble_cus_evt_t * p_evt);

/**@brief Custom Service init structure. 
 * This contains all options needed for the initialization of the service.
 *
 */
typedef struct
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */    
    ble_srv_cccd_security_mode_t  value_char_attr_md;             /**< Used to set the security mode of the cccd for the char. */
} ble_cus_init_t;

/**@brief Custom Service structure.
 *        This contains various status information for the service.
 */
struct ble_cus_s
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
   
    ble_gatts_char_handles_t      value_char_handles;             /**< Handles related to the value characteristic. */
          
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type;                      /**< Holds the service uuid type. */
};

/**@brief Function for initializing the Custom ble service.
 *
 * @param[in]   p_cus       Custom service structure.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init);

/**@brief Function for updating the values on the ble characteristic.
 *
 * @param[in]   p_cus             Custom service structure.
 * @param[in]   p_value           values.
 * @param[in]   conn_handle       Connection handle.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_cus_value_update(ble_cus_t * p_cus, uint8_t  * p_value, uint16_t conn_handle);
