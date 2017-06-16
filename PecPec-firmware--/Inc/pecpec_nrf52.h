/**
 *  2017-06-12 © Justas Riabovas
 *  Various functions for NRF52 chip control
 */

#ifndef	NRF52_PORT_H
#define NRF52_PORT_H

#include "MMA8453.h"
#include "nrf_gpio.h"

#define LED1_Off()  nrf_gpio_pin_clear(LED1_CUSTOM)
#define LED1_On()  nrf_gpio_pin_set(LED1_CUSTOM)

#define I2C_PullUp_On()  nrf_gpio_pin_set(I2C_PULLUP)
#define I2C_PullUp_Off()  nrf_gpio_pin_clear(I2C_PULLUP)

extern MMA8453_hndl  Acc1, Acc2;

/**< Exported functions */
void MMA8453_Init_and_Test(MMA8453_hndl * hndl);
void Acc_Get_and_Display();
void power_manage(void);
void sleep_mode_enter(void);
void twi_init (void);
void timer_twi_start();
void timer_twi_stop();
void buttons_leds_ports_init();
void buttons_leds_ports_start();
void LED1_blink(uint32_t time);

#endif //NRF52_PORT_H
