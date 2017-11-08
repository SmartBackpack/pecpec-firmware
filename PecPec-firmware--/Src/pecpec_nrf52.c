/**
 *  2017-06-12 © Justas Riabovas
 *  Various functions for NRF52 chip control
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pecpec_nrf52.h"

#include "nrf_soc.h"
#include "ble_nus.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_gpiote.h"

#include "nrf_drv_twi.h"
#include "MMA8453.h"

#include "ser_phy_debug_comm.h"
#define NRF_LOG_MODULE_NAME "PEC_NRF52"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "pecpec_ble.h"

//GPIOTE
// GPIOTE user identifier.
//static app_gpiote_user_id_t   INT1_user_id;
///Local variables
APP_TIMER_DEF(LED1_timer_id);
APP_TIMER_DEF(timer_twi_id);

//Processing data
#define TILTOK  1
#define TILTFAIL 0
#define TILT_ALARM_CNT  5 //Counter of invalid values to start alarm
#define TILTOK_TH   30  //threshold values for tilt detection
#define TILTFAIL_TH 40  //threshold values for tilt detection
static volatile bool Tilt1Stat = false, Tilt2Stat = false, TiltStat = false;
//static volatile long Tilt1Cnt = 0, Tilt2Cnt = 0,
//static volatile uint8_t TiltOkCnt = 0;
static volatile uint8_t TiltFailedCnt = 0;

/**@brief Function for placing the application in low power state while waiting for events.
 */
void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter(void)
{
    uint32_t err_code;
    //Turn off various stuff
    LED1_Off();

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}
/* data buffers */
uint8_t twi_rx_buff[TWI_RX_BUFF_SIZE], twi_tx_buff[TWI_TX_BUFF_SIZE];

/* TWI instance. */
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Accelerometer stuff */
MMA8453_hndl  Acc1, Acc2;

/** \brief Read acceleration info from sensors and put it to BLE_UART and RTT
 */
void Acc_Get_and_Display (void)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    int16_t x1 = 0, y1 = 0, z1 = 0;
    int16_t x2 = 0, y2 = 0, z2 = 0;

    ///Read Accelerometers
    //Enable I2C lines
    I2C_PullUp_On();
    //Get data
    MMA8453_getAcceleration(&Acc1, &x1, &y1, &z1);
    MMA8453_getAcceleration(&Acc2, &x2, &y2, &z2);
    //Disable I2C lines
    I2C_PullUp_Off();

    ///Push RAW data to RTTand BLE
    //Push data to RTT
    NRF_LOG_INFO("%d %d %d; %d %d %d\n\r", x1 ,y1 ,z1, x2 ,y2 ,z2);
    //Push data to BLE UART
    sprintf((char *) data_array, "Acc: %d %d %d; %d %d %d\r", x1 ,y1 ,z1, x2 ,y2 ,z2);
    ble_send(data_array, strlen((char *) data_array));
    NRF_LOG_FLUSH();

    ///Detect tilt
    //y axis is used to detect tilt
    //0 .. -64 - tilted position
    //0 - horizontal position
    //0 .. 65 - not tilted position
    if (y1 > TILTFAIL_TH) Tilt1Stat = false;
    else if (y1 < TILTOK_TH) Tilt1Stat = true;
    if (y2 > TILTFAIL_TH) Tilt2Stat = false;
    else if (y2 < TILTOK_TH) Tilt2Stat = true;

    ///Process data
    TiltStat = TILTOK;
    if (Tilt1Stat ^ Tilt2Stat)   //sensors status not equal
    {
        //inceremnt failed counters
        //if (Tilt1Stat == TILTOK) Tilt2Cnt++;
        //else Tilt1Cnt++;

        //Wait for n failed times
        if (TiltFailedCnt >= TILT_ALARM_CNT) TiltStat = TILTFAIL;
        else TiltFailedCnt++;
    }
    else     //Sensor status equal
    {
        TiltFailedCnt = 0; //reset failed couter
        //TiltOkCnt ++; //increment OK counter
        if (Tilt1Stat == TILTFAIL)   //Both sensors in not tilted, standby
        {
            //LEDSOff();
        }
        else     //Both sensors tilted, operating mode
        {
        }
    }

    ///Print status to RTT and BLE
    //Push data to RTT
    NRF_LOG_INFO("STAT:%u;%u;%u;0;0;0;%u\n\r", Tilt1Stat, Tilt2Stat, TiltStat, TiltFailedCnt);
    //Push data to BLE UART
    sprintf((char *) data_array, "STAT:%u;%u;%u;0;0;0;%u\r", Tilt1Stat, Tilt2Stat, TiltStat, TiltFailedCnt);
    ble_send(data_array, strlen((char *) data_array));
    NRF_LOG_FLUSH();
}

/** \brief Initialize and test accelerometers
 *
 * \param hndl MMA8453_hndl*
 * \return void
 *
 */
void MMA8453_Init_and_Test(MMA8453_hndl * hndl)
{
    uint32_t err_code = NRF_SUCCESS;
    char ACC1str[] = "ACC1: ";
    char ACC2str[] = "ACC2: "; //lenght must be equal to ACC1
    char ACC[strlen(ACC1str)];

    //Enable I2C lines
    I2C_PullUp_On();

    //Detect which ACC chip handle was given to this function and take specific tasks
    if (hndl == &Acc1)
    {
        memcpy(ACC, ACC1str, strlen(ACC1str));
        MMA8453_setAddress(hndl, MMA8453_ADDRESS_ALT_LOW);
    }
    else if (hndl == &Acc2)
    {
        memcpy(ACC, ACC2str, strlen(ACC2str));
        MMA8453_setAddress(hndl, MMA8453_ADDRESS_ALT_HIGH);
    }
    else
    {
        NRF_LOG_INFO("Unknown Accelerometer handle")
        NRF_LOG_FLUSH();
        //Disable I2C lines
        I2C_PullUp_Off();
        return;
    }

    if (MMA8453_testConnection(hndl))
    {
        NRF_LOG_INFO ("%sconnection successful\r\n", (uint32_t)ACC);
        NRF_LOG_INFO ("%sinit start\r\n", (uint32_t)ACC);
        err_code = MMA8453_initialize(hndl, 0); //timeout is not implemented in NRF52 port
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_INFO ("%sinit err %i\r\n", (uint32_t)ACC, err_code);
        }
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO ("%sinit end\r\n", (uint32_t)ACC);
    }
    else
    {
        NRF_LOG_INFO ("%sconnection failed\r\n", (uint32_t)ACC);
    }
    NRF_LOG_FLUSH();
    //Disable I2C lines
    I2C_PullUp_Off();
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    MMA8453_twi_handler(p_event);

//    switch (p_event->type)
//    {
//    case NRF_DRV_TWI_EVT_DONE:
//        if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
//        {
//
//        }
//
//        if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX)
//        {
//
//        }
//        break;
//    default:
//        break;
//    }
}

static void timer_twi_handler(void * p_context)
{
    NRF_LOG_INFO("I2C Timeout");
    UNUSED_VARIABLE(p_context);
    MMA8453_twi_stop();
}

void timer_twi_start()
{
    APP_ERROR_CHECK(app_timer_start(timer_twi_id, APP_TIMER_TICKS(50), NULL));
}

void timer_twi_stop()
{
    APP_ERROR_CHECK(app_timer_stop(timer_twi_id));
}
/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    //Create timer
    APP_ERROR_CHECK(app_timer_create(&timer_twi_id, APP_TIMER_MODE_SINGLE_SHOT, timer_twi_handler));

    const nrf_drv_twi_config_t config =
    {
        .scl                = ACC_SCL_PIN,
        .sda                = ACC_SDA_PIN,
        .frequency          = NRF_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}



#if (UART_ENABLED == 1)
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
    case APP_UART_DATA_READY:
        UNUSED_VARIABLE(app_uart_get(&data_array[index]));
        index++;

        if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
        {
            NRF_LOG_DEBUG("Ready to send data over BLE NUS\r\n");
            NRF_LOG_HEXDUMP_DEBUG(data_array, index);

            do
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            while (err_code == NRF_ERROR_BUSY);

            index = 0;
        }
        break;

    case APP_UART_COMMUNICATION_ERROR:
        APP_ERROR_HANDLER(p_event->data.error_communication);
        break;

    case APP_UART_FIFO_ERROR:
        APP_ERROR_HANDLER(p_event->data.error_code);
        break;

    default:
        break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_event_handle,
                        APP_IRQ_PRIORITY_LOWEST,
                        err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */
#endif //UART_ENABLED

void LED1_blink(uint32_t time)
{
    LED1_On();
    APP_ERROR_CHECK(app_timer_start(LED1_timer_id, APP_TIMER_TICKS(time), NULL));
}

static void LED1_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    LED1_Off();
}

/** \brief GPIOTE event handler
 * \return void
 *
 */
void INT_gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    char text[40]; ///!!!!! Check that strings will fit into data_array
    uint8_t buff = 0;
    UNUSED_PARAMETER(pin);
    UNUSED_PARAMETER(action);
    //LED1_blink(100);

    I2C_PullUp_On(); //Enable communication
    //Get interrupt source and take corresponding actions
    APP_ERROR_CHECK(MMA8453_ReadRegister(&Acc1, MMA8453_INT_SOURCE, &buff));
    NRF_LOG_DEBUG("INT: 0x%X\r\n", buff);

    //Entry points MMA8453_SRC_DRDY and MMA8453_SRC_ASLP skipped
    text[0] = 0; //clear string
    if (buff & (1 << MMA8453_SRC_TRANS))
    {
        APP_ERROR_CHECK(MMA8453_ReadRegister(&Acc1, MMA8453_TRANSIENT_SRC, &buff));
        sprintf((char *) text, "INT: Transient 0x%X\r", buff);
    }
    else if (buff & (1 << MMA8453_SRC_LNDPRT))
    {
        APP_ERROR_CHECK(MMA8453_ReadRegister(&Acc1, MMA8453_PL_STATUS, &buff));
        sprintf((char *) text, "INT: Land/Portrait 0x%X\r", buff);
    }
    else if (buff & (1 << MMA8453_SRC_PULSE))
    {
        APP_ERROR_CHECK(MMA8453_ReadRegister(&Acc1, MMA8453_PULSE_SRC, &buff));
        sprintf((char *) text, "INT: Pulse 0x%X\r", buff);
    }
    else if (buff & (1 << MMA8453_SRC_FF_MT))
    {
        APP_ERROR_CHECK(MMA8453_ReadRegister(&Acc1, MMA8453_FF_MT_SRC, &buff));
        sprintf((char *) text, "INT: Freefall/Motion 0x%X\r", buff);
    }
    I2C_PullUp_Off(); //Disable communication
    if (text[0])   //If string is not empty LOG and transmit it
    {
        NRF_LOG_INFO("%s\n", (uint32_t)text);
        ble_send((uint8_t*) text, strlen (text));
    }

    NRF_LOG_FLUSH();
}

/**@brief Function for initializing buttons and leds.
 */
void buttons_leds_ports_init()
{
    ///GPIOTE driver
    APP_ERROR_CHECK(nrf_drv_gpiote_init());

    ///External sensors power (not Accelerometer)
    nrf_gpio_cfg_output(EXT_POW);
    nrf_gpio_pin_set(EXT_POW);


    ///I2c pullup
    nrf_gpio_cfg_output(I2C_PULLUP);
    nrf_gpio_pin_clear(I2C_PULLUP);

    ///LED
    nrf_gpio_cfg_output(LED1_CUSTOM);
    nrf_gpio_cfg_output(LED2_CUSTOM);
    nrf_gpio_pin_clear(LED1_CUSTOM);
    nrf_gpio_pin_clear(LED2_CUSTOM);
    //LED timer
    APP_ERROR_CHECK(app_timer_create(&LED1_timer_id, APP_TIMER_MODE_SINGLE_SHOT, LED1_timer_handler));

    ///Initialize INT1 and INT2 interrupts
    nrf_gpio_cfg_input(ACC_INT1_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(ACC_INT2_PIN, NRF_GPIO_PIN_NOPULL);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(ACC_INT1_PIN, &in_config, INT_gpiote_event_handler));
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(ACC_INT2_PIN, &in_config, INT_gpiote_event_handler));
}


/**@brief Enable ports (interrupts, etc...)
 */
void buttons_leds_ports_start()
{
    nrf_drv_gpiote_in_event_enable(ACC_INT1_PIN, true);
    nrf_drv_gpiote_in_event_enable(ACC_INT2_PIN, true);
}
