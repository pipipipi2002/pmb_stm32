#ifndef INC_PMB_FIRMWARE_H
#define INC_PMB_FIRMWARE_H

#include "common_defines.h"


#define PMB_BAUDRATE               (115200)

#define PMB_BQ_I2C_ADDR
#define PMB_DISPLAY_I2C_ADDR

#define PMB_UART1_PORT             (GPIOA)
#define PMB_UART1_TX_PIN           (GPIO9)
#define PMB_UART1_RX_PIN           (GPIO10)

#define PMB_I2C1_PORT              (GPIOB)
#define PMB_I2C1_SCL_PIN           (GPIO6)
#define PMB_I2C1_SDA_PIN           (GPIO7)

#define PMB_CAN_PORT               (GPIOA)
#define PMB_CAN_RX_PIN             (GPIO11)
#define PMB_CAN_TX_PIN             (GPIO12)

#define PMB_BQ_ALERT1_PORT         (GPIOB)
#define PMB_BQ_ALERT1_PIN          (GPIO13)

#define PMB_BQ_ALERT2_PORT         (GPIOB)
#define PMB_BQ_ALERT2_PIN          (GPIO12)

#define PMB_NERROR_PORT            (GPIOB)
#define PMB_NERROR_PIN             (GPIO14)

#define PMB_RELAY_ON_PORT          (GPIOC)
#define PMB_RELAY_ON_PIN           (GPIO1)

#define PMB_PMOS_ON_GPIO_PORT      (GPIOC)
#define PMB_PMOS_ON_GPIO_PIN       (GPIO3)

#define PMB_DISPLAY_RESET_PORT     (GPIOC)
#define PMB_DISPLAY_RESET_PIN      (GPIO4)

#define PMB_REED_OFF_PORT          (GPIOC)
#define PMB_REED_OFF_PIN           (GPIO7)


#endif // INC_PMB_FIRMWARE_H