/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_delay.h"

#include "nrf_drv_systick.h"
#include "app_error.h"
#include "nrf.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "ble_nus.h"
#include "printf.h" //Kustaa Nyholm / SpareTimeLabs printf implementation
#include "SEGGER_RTT.h" //enable RTT
#include "MMA8453.h" //enable RTT

#include "app_scheduler.h"
#include "ser_hal_transport.h"
#include "ser_conn_handlers.h"

#define NRF_LOG_MODULE_NAME "PEC_MAIN"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "pecpec_nrf52.h"
#include "pecpec_ble.h"
#include <string.h>

//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

/** Global variables */
//Timers
//APP_TIMER_DEF(timer_100ms_id);
APP_TIMER_DEF(timer_1s_id);
static volatile uint8_t temp_request = false;

char RTT_Debug_Tx[SEGGER_RTT_CONFIG_BUFFER_SIZE_UP];

//static void timer_100ms_handler(void * p_context)
//{
//}

static void timer_1s_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    LED1_blink(50);
    //Acc_Get_and_Display();
    temp_request++; //request data from temp sensor


//    uint32_t Int1, Int2;
//    Int1 = nrf_gpio_pin_read(ACC_INT1_PIN);
//    Int2 = nrf_gpio_pin_read(ACC_INT2_PIN);
//    NRF_LOG_INFO("%d %d\n\r", Int1 ,Int2);
//    NRF_LOG_FLUSH();
}


/** \brief Connect with Kustaa Nyholm / SpareTimeLabs printf implementation
 * \param p void*
 * \param ch char
 * \return void
 */
void pchar_ToSpareTimeLabs(void* p, char ch)
{
    SEGGER_RTT_Write(0, (void *)&ch, 1);
}
/**
 * @brief Function for application main entry.
 */
int main(void)
{
    bool     erase_bonds;
    /*Connect with Kustaa Nyholm / SpareTimeLabs printf implementation*/
    init_printf(NULL, pchar_ToSpareTimeLabs);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    ///Init and create timers
    // Initialize app_timer
    APP_ERROR_CHECK(app_timer_init());
    //create timers
    //APP_ERROR_CHECK(app_timer_create(&timer_100ms_id, APP_TIMER_MODE_REPEATED, timer_100ms_handler));
    APP_ERROR_CHECK(app_timer_create(&timer_1s_id, APP_TIMER_MODE_REPEATED, timer_1s_handler));

    ///TWI - I2C init
    twi_init();
    ///temp sensor init
    nrf_delay_ms(15);  //temp sensor startup time
    ///Init IO
    buttons_leds_ports_init(&erase_bonds);
    ///INIT BLE
    ble_init();

#if (UART_ENABLED == 1)
    uart_init();
    printf("\r\nUART: Start\r\n");
#endif


    ///init both accelerometers
    MMA8453_Init_and_Test(&Acc1);
    MMA8453_Init_and_Test(&Acc2);


    ///DUMP ACC registers
//    uint16_t i = 1;
//    uint8_t b;
//    for (i = 0; i < 50; i++){
//        err_code = MMA8453_readByte_BLOCK(MMA8453_ADDRESS_ALT_LOW, i, &b, 0);
//
//        NRF_LOG_INFO("ACC: %i %x:%x\r\n", err_code, i, b);
//        NRF_LOG_FLUSH();
//    }
    ///Start BLE
    ble_start();
    NRF_LOG_INFO("BLE: connectivity started\r\n");
    NRF_LOG_FLUSH();


    ///Start timers
    //APP_ERROR_CHECK(app_timer_start(timer_100ms_id, APP_TIMER_TICKS(100), NULL));
    APP_ERROR_CHECK(app_timer_start(timer_1s_id, APP_TIMER_TICKS(1000), NULL));

    ///Enable IO (interrupts)
    buttons_leds_ports_start();

    // Enter main loop.
    for (;;)
    {
        ///temp sensor test
        if(temp_request == 5) { //read sensor data every 5 seconds
            temp_request = 0;
            I2C_PullUp_On();
            uint8_t buffer[9];
            uint8_t c;
            for (c = 0; c < sizeof(buffer); c++) buffer[c] = 0;
            MMA_res res;
            UNUSED_PARAMETER(res);
            res = MMA8453_readBytes_BLOCK (0x40, 0xE3, 0x03, buffer, 0);
            float t;
            uint8_t decit;
            t = (float)((buffer[0] << 8) | (buffer[1] & 0xFC));
            t *= 175.72;
            t /= 65536;
            t -= 46.85;
            decit = ((uint8_t)(t * 10) % 10);
            //NRF_LOG_INFO("TEMP: %x %x %x, %i\r\n", buffer[0], buffer[1], buffer[2], res);


            res = MMA8453_readBytes_BLOCK (0x40, 0xE5, 0x03, buffer, 0);
            I2C_PullUp_Off();
            float h;
            h = (float)((buffer[0] << 8) | (buffer[1] & 0xFC));

            h *= 125;
            h /= 65536;
            h -= 6;
            // NRF_LOG_INFO("HUMIDITY: %x %x %x, %i\r\n", buffer[0], buffer[1], buffer[2], res);
            NRF_LOG_INFO("TEMP, HUM: %i.%u; %i %%RH\r\n", t, decit, h);


            static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
            sprintf((char *) data_array, "2: TEMP, HUM: %d.%u; %u %%RH\r", (int8_t)t, decit, (uint8_t)h);
            ble_send(data_array, strlen((char *) data_array));

            temperature_measurement_send();

            NRF_LOG_FLUSH();
        }
        power_manage();
    }
}

/**
 *@}
 **/
