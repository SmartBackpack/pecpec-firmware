/**
 *  2017-06-12 © Justas Riabovas
 *  Bluetooth related part
 */


#ifndef	APP_BLE_H
#define APP_BLE_H

#include "app_error.h"

 /**< Exported Functions */
void ble_init(void);
void advertising_start(bool erase_bonds);
//void bsp_event_handler(bsp_event_t event);

ret_code_t ble_start();
ret_code_t ble_send(uint8_t * p_string, uint16_t length);

#endif //APP_BLE_H
