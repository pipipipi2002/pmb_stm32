#ifndef INC_PMB_FIRMWARE_H
#define INC_PMB_FIRMWARE_H

#define PMB_CODE
#include "common_defines.h"
#include "can.h"


//
// Software Configurations
//
#define PMB_ID                          (5)
#define PMB_CURRENT_SCALE               (2)
#define PMB_MAX_CURR                    (80)
#define PMB_SHUNT_RES                   (1)         //mOhm
#define PMB_ARR_SIZE                    (10)

#define PMB_CAN_HB_MSG_INTVL       (500)
#define PMB_CAN_BATT_MSG_INTVL     (200)
#define PMB_CAN_BOARD_MSG_INTVL    (1000)
#define PMB_OLED_REFRESH_INTVL     (500)
#define PMB_STATUS_UPDATE_INTVL             (50)        // Internal Data Update

#if (PMB_ID % 2 == 1) 
    #define BB_CAN_ID_BATT_STAT     (BB_CAN_ID_BATT_1_STAT)
    #define BB_CAN_ID_PMB_STAT      (BB_CAN_ID_PMB_1_STAT)
    #define BB_HEARTBEAT_ID_PMB     (BB_HEARTBEAT_ID_PMB_1)
#else 
    #define BB_CAN_ID_BATT_STAT     (BB_CAN_ID_BATT_2_STAT)
    #define BB_CAN_ID_PMB_STAT      (BB_CAN_ID_PMB_2_STAT)
    #define BB_HEARTBEAT_ID_PMB     (BB_HEARTBEAT_ID_PMB_2)
#endif // PMB_ID

// 
// BQ calibration
// 
#define PMB_PERFORM_CALIBRATION     (0)
#define PMB_CALIBRATION_VOLTAGE         (16000)
#define PMB_CALIBRATION_CURRENT         (3000)

// 
// Hardware Configurations
//
#define PMB_UART_BAUDRATE          (115200)

#define PMB_BQ_I2C_PORT            (I2C1)
#define PMB_BQ_I2C_ADDR

#define PMB_DISPLAY_I2C_PORT       (I2C1)
#define PMB_DISPLAY_I2C_ADDR       (0x78)

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

#define PMB_RELAY_OFF_PORT         (GPIOC)
#define PMB_RELAY_OFF_PIN          (GPIO1)

#define PMB_PMOS_ON_GPIO_PORT      (GPIOC)
#define PMB_PMOS_ON_GPIO_PIN       (GPIO3)

#define PMB_DISPLAY_RESET_PORT     (GPIOC)
#define PMB_DISPLAY_RESET_PIN      (GPIO4)

#define PMB_REED_OFF_PORT          (GPIOC)
#define PMB_REED_OFF_PIN           (GPIO7)

//
// SSD1306 Configuration
//
#define SSD1306_USE_I2C
#define SSD1306_I2C_PORT            (PMB_DISPLAY_I2C_PORT)  
#define SSD1306_I2C_ADDR            (PMB_DISPLAY_I2C_ADDR >> 1)
#define SSD1306_Reset_Port          (PMB_DISPLAY_RESET_PORT)
#define SSD1306_Reset_Pin           (PMB_DISPLAY_RESET_PIN)
#define SSD1306_INCLUDE_FONT_6x8    (1)

#endif // INC_PMB_FIRMWARE_H