#ifndef INC_FIRMWARE_H
#define INC_FIRMWARE_H

#include "common_defines.h"

#define BAUDRATE                    (115200)

#define MAIN_BQ_I2C_ADDR
#define MAIN_DISPLAY_I2C_ADDR

#define MAIN_UART1_PORT             (GPIOA)
#define MAIN_UART1_TX_PIN           (GPIO9)
#define MAIN_UART1_RX_PIN           (GPIO10)

#define MAIN_BQ_ALERT1_PORT         (GPIOB)
#define MAIN_BQ_ALERT1_PIN          (GPIO13)

#define MAIN_BQ_ALERT2_PORT         (GPIOB)
#define MAIN_BQ_ALERT2_PIN          (GPIO12)

#define MAIN_NERROR_PORT            (GPIOB)
#define MAIN_NERROR_PIN             (GPIO14)

#define MAIN_RELAY_ON_PORT          (GPIOC)
#define MAIN_RELAY_ON_PIN           (GPIO1)

#define MAIN_PMOS_ON_GPIO_PORT      (GPIOC)
#define MAIN_PMOS_ON_GPIO_PIN       (GPIO3)

#define MAIN_DISPLAY_RESET_PORT     (GPIOC)
#define MAIN_DISPLAY_RESET_PIN      (GPIO4)

#define MAIN_REED_OFF_PORT          (GPIOC)
#define MAIN_REED_OFF_PIN           (GPIO7)


#endif // INC_FIRMWARE_H