

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/**< ports */
#define I2C_PULLUP   8
#define ACC_INT1_PIN    5
#define ACC_INT2_PIN    6
#define ACC_SCL_PIN             27    // SCL signal pin
#define ACC_SDA_PIN             26    // SDA signal pin

#define MAX_GPIOTE_USERS    2

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0
#define TWI_RX_BUFF_SIZE 10 //must be higher than than 2
#define TWI_TX_BUFF_SIZE 10 //must be higher than than 2
#if ((TWI_RX_BUFF_SIZE < 2) || (TWI_TX_BUFF_SIZE < 2))
#error Invalid TWI buffer size specified
#endif
extern uint8_t twi_rx_buff[TWI_RX_BUFF_SIZE], twi_tx_buff[TWI_TX_BUFF_SIZE];

#define LED1_CUSTOM   11
#define LED2_CUSTOM   12

#define NRF_CLOCK_LFCLKSRC_BOARD      {.source        = NRF_CLOCK_LF_SRC_RC,            \
                                 .rc_ctiv       = 16,                                \
                                 .rc_temp_ctiv  = 2,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#endif //APP_CONFIG_H
