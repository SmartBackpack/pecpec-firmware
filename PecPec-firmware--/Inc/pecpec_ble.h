/**
 *  2017-06-12 © Justas Riabovas
 *  Bluetooth related part
 */


#ifndef	APP_BLE_H
#define APP_BLE_H

#include "app_error.h"

 /**< Exported Functions */
void ble_init();
ret_code_t ble_start();
ret_code_t ble_send(uint8_t * p_string, uint16_t length);
void temperature_measurement_send(void);

#endif //APP_BLE_H
