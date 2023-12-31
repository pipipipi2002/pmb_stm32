// Libopencm3 Header Files
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/syscfg.h>
#include <printf.h>

// Shared Header Files
#include "common_defines.h"
#include "can_defines.h"
#include "board_defines.h"
#include "mainapp_defines.h"
#include "system.h"
#include "uart_if.h"
#include "log.h"
#include "gpio_if.h"
#include "can_if.h"

// Own Header Files
#include "firmware.h"
#include "i2c_if.h"
#include "ADS1115.h"
#include "BQ34110.h"
#include "SSD1306/ssd1306.h"
#include "SSD1306/ssd1306_fonts.h"

#ifndef MAINAPP
    #error "MAINAPP OPTION NOT SELECTED"
#endif

typedef struct {
    uint32_t device_id;         
    uint32_t commit_id;         // Inserted by updator firmware
} fw_info_ts;

/*
 * Global Variables 
 */
// Interrupt Vector Table in SRAM 
volatile uint32_t vectorTable[48] __attribute__((section(".ramvectortable")));
// Dev ID to be stored in flash
__attribute__((section(".fw_info"))) volatile fw_info_ts fw_info = {
    .device_id = DEVICE_ID,
    .commit_id = 0xFFFFFFFF
};

// Telemetry Data
static uint16_t batt_voltage = 0;
static int32_t batt_current = 0;
static uint16_t batt_status = 0;
static char batt_state[15] = "";
static float board_temperature = 0;
static float board_pressure = 0;
static uint8_t incomingCanData[8] = {0};

// Timer for "cyclic functions"
static timeout_ts updateTimer;
static timeout_ts CAN_BattTimer;
static timeout_ts CAN_BoardTimer;
static timeout_ts CAN_HbTimer;
static timeout_ts displayTimer;

// CAN Messages Placeholder
static canMsg_tu canBattMsg, canBoardMsg;
static canMsg_tu canHbMsg = {.heartbeatId = BB_HEARTBEAT_ID_PMB};

/*
 * Internal Function Declarations
 */
int main(void);
static void vector_setup(void);
static void setup(void);
static float getPressure(void);
static void poweroff(void);
static void sendFwInfo(void);
static void encodeCanMsgBattStat(void);
static void encodeCanMsgBoardStat(void);
static void displayOnMessage(void);
static void displayOffMessage(void);
static void displayDataMessage(void);
static void setState(uint16_t status);

/*
 * Function Defintions
 */

/**
 * @brief Relocate vector table to SRAM location. 
 *        Due to Cortex M0 restriction on editing the VTOR.
 *        Refer to AN4065.
 */
static void vector_setup(void) {
    /* As Cortex M0 doesnt allow VTOR to be editted, we copy the vector */
    for (uint32_t i = 0; i < 48; i++) {
        vectorTable[i] =  *(volatile uint32_t*)(MAIN_APP_START_ADDR + (i << 2));
    }
    /* Enable clock to SYSCFG APB2 Peripheral */
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    /* Link 0x0000_0000 to SRAM at 0x2000_0000 */
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_MEM_MODE_SRAM;
}

int main(void) {    
    system_rccInit();       // Setup RCC 
    vector_setup();         // Setup IVT relocation
    system_systickInit();   // Setup Systick

    setup();

    $SUCCESS("Entered Application");
    $SUCCESS("Power Monitoring Board AUV4");
    $SUCCESS("PMB Number: %d", PMB_ID);
    sendFwInfo();

    displayOnMessage();

    /* Supply power */
    gpio_clear(PMB_RELAY_OFF_PORT, PMB_RELAY_OFF_PIN);      // Latch pwoer to PMB 
    // TODO Do some verification before supplying power to vehicle
    gpio_set(PMB_PMOS_ON_GPIO_PORT, PMB_PMOS_ON_GPIO_PIN);  // Power to Vehicle
    $SUCCESS("PMOS ON, Battery connected to Vehicle");

#if (PMB_PERFORM_CALIBRATION == 1)
    $SUCCESS("Starting PMB BQ Calibration");
    BQ_Init();
    BQ_CalibrateVoltage(PMB_CALIBRATION_VOLTAGE);
    BQ_CalibrateCurrent(PMB_CALIBRATION_CURRENT / PMB_CURRENT_SCALE);
    $SUCCESS("Completed PMB BQ Calibration");
#endif // PMB_PERFORM_CALIBRATION

    while (true) {
        /* Check for off reed switch activation */
        if (gpio_get(PMB_REED_OFF_PORT, PMB_REED_OFF_PIN)) {
            $SUCCESS("Shutdown Sequence Activated");
            poweroff();
            while (1); // Will not reach here
        }

        /* Update PMB and Battery Stats */
        if (timeout_hasElapsed(&updateTimer)) {
            $INFO("Updating internal data");
            batt_status = BQ_GetBattStatus();
            batt_current = BQ_GetCurrent() * PMB_CURRENT_SCALE;
            batt_current = (batt_current < 0) ? -1 * batt_current : batt_current;
            batt_voltage = BQ_GetVoltage();
            board_temperature = BQ_GetTemp();
            board_pressure = getPressure();
        }

        /* Send Battery Stats via CAN */
        if (timeout_hasElapsed(&CAN_BattTimer)) {
            $INFO("Sending Battery Statistics via CAN");

            encodeCanMsgBattStat();
            canif_sendVehMsg(&canBattMsg, BB_CAN_STD_MSG_SIZE, BB_CAN_ID_BATT_STAT);
        }

        /* Send Board Stats via CAN */
        if (timeout_hasElapsed(&CAN_BoardTimer)) {
            $INFO("Sending Board Statistics via CAN");

            encodeCanMsgBoardStat();
            canif_sendVehMsg(&canBoardMsg, BB_CAN_STD_MSG_SIZE, BB_CAN_ID_PMB_STAT);
        }
        
        /* Send Heartbeat via CAN */
        if (timeout_hasElapsed(&CAN_HbTimer)) {
            $INFO("Sending Heartbeat via CAN");
            canif_sendVehMsg(&canHbMsg, BB_CAN_HB_MSG_SIZE, BB_CAN_ID_HEARTBEAT);
        }

        /* Update Display */
        if(timeout_hasElapsed(&displayTimer)) {
            displayDataMessage();
        }

        /* Check for incoming can message */
        if (canif_getRxDataReady(BB_CAN_ID_BOOT_INFO)) {
            uint8_t i;
            for (i = 0; i < 4; i++) {
                if (!canif_getRxData(BB_CAN_ID_BOOT_INFO, &incomingCanData[i])) {
                    break;
                }
            }
            if (i == 4) {
                uint32_t res = incomingCanData[0] | (incomingCanData[1] << 8) | (incomingCanData[2] << 16) | (incomingCanData[3] << 24);
                if (res == BOOT_INFO_REQ_MSG) {
                    sendFwInfo();
                }
            }
        }
    }
    return 0;
}

static void setup(void) {
    /* HAL Setup */
    while(!uart1if_setup()) system_delayMs(1000);
    while(!gpioif_setup()) system_delayMs(1000);
    while(!canif_setup()) system_delayMs(1000);
    while(!i2cif_setup()) system_delayMs(1000);

    /* Driver Setup */
    ssd1306_Init();

    timeout_setup(&updateTimer,  PMB_STATUS_UPDATE_INTVL, true);
    timeout_setup(&CAN_BattTimer,  PMB_CAN_BATT_MSG_INTVL, true);
    timeout_setup(&CAN_BoardTimer,  PMB_CAN_BOARD_MSG_INTVL, true);
    timeout_setup(&CAN_HbTimer,  PMB_CAN_HB_MSG_INTVL, true);
    timeout_setup(&displayTimer,  PMB_OLED_REFRESH_INTVL, true);
    
    $SUCCESS("Setup Completed");
}

/**
 * @brief Get the Pressure from MPXH6250 through ADS1115. 
 * @details 0.0001875 is the ADC resolution.
 * @details Vout = VS * (0.004 * P - 0.040) + error; From MPXH datasheet .
 * @return float 
 */
static float getPressure(void) {
	return (ADS_ReadADC_SingleEnded(1)*0.0001875) / (5 * 0.0040) + 10;
}

/**
 * @brief Power off sequence
 * 
 */
static void poweroff(void) {
    displayOffMessage();
    gpio_clear(PMB_PMOS_ON_GPIO_PORT, PMB_PMOS_ON_GPIO_PIN); // Turn off power to vehicle
    system_delayMs(500);
    gpio_set(PMB_RELAY_OFF_PORT, PMB_RELAY_OFF_PIN); // Turn off power to PMB circuit
}

/**
 * @brief Set the state of the battery (Charging, Discharging or Unknown)
 * 
 * @param status Raw register from BQ chip
 */
static void setState(uint16_t status) {
	if (status & 0x02) {
		snprintf(batt_state, sizeof(batt_state), "CHARGING");
	}
	else if (status & 0x01){
		snprintf(batt_state, sizeof(batt_state), "DISCHARGING");
	}
	else {
		snprintf(batt_state, sizeof(batt_state), "UNKNOWN");
	}
}

/**
 * @brief Send Firmware Information through CAN
 * 
 */
static void sendFwInfo(void) {
    $SUCCESS("Sending FW info");
    canif_sendData((uint8_t *)&fw_info, 8, BB_CAN_ID_BOOT_INFO);
}

/**
 * @brief Encode the current and voltage reading into the CAN msg frame.
 * 
 */
static void encodeCanMsgBattStat() {
    canBattMsg.battStat.u8Reserved1 = 0xFF;
    canBattMsg.battStat.u8Reserved2 = 0xFF;
    canBattMsg.battStat.u8Reserved3 = 0xFF;
    canBattMsg.battStat.u8Reserved4 = 0xFF;
    canBattMsg.battStat.u8CurrentLow = (uint8_t)((uint16_t)(batt_current) & 0xFF);
    canBattMsg.battStat.u8CurrentHigh = (uint8_t)((uint16_t)(batt_current) >> 8);
    canBattMsg.battStat.u8VoltageLow = (uint8_t)((batt_voltage) & 0xFF);
    canBattMsg.battStat.u8VoltageHigh = (uint8_t)((batt_voltage) >> 8);
}

/**
 * @brief Encode the temperature and pressure reading into the CAN msg frame.
 * 
 */
static void encodeCanMsgBoardStat() {
    canBoardMsg.boardStat.u8Reserved1 = 0xFF;
    canBoardMsg.boardStat.u8Reserved2 = 0xFF;
    canBoardMsg.boardStat.u8Reserved3 = 0xFF;
    canBoardMsg.boardStat.u8Reserved4 = 0xFF;
    canBoardMsg.boardStat.u8TempLow = (uint8_t)((uint16_t)(board_temperature) & 0xFF);
    canBoardMsg.boardStat.u8TempHigh = (uint8_t)((uint16_t)(board_temperature) >> 8);
    canBoardMsg.boardStat.u8PressureLow = (uint8_t)((uint16_t)(board_pressure) & 0xFF);
    canBoardMsg.boardStat.u8PressureHigh = (uint8_t)((uint16_t)(board_pressure) >> 8);
}

/**
 * @brief Command SSD1306 to display the ON screen.
 * 
 */
static void displayOnMessage(void) {
    ssd1306_Fill(White);
    ssd1306_SetCursor(2,31);
	ssd1306_WriteString("TURNING ON PMB ...", Font_6x8, Black);
	ssd1306_UpdateScreen();
    system_delayMs(1000);
}

/**
 * @brief Command SSD1306 to display the OFF screen.
 * 
 */
static void displayOffMessage(void) {
    ssd1306_Fill(White);
    ssd1306_SetCursor(2,31);
	ssd1306_WriteString("TURNING OFF PMB ...", Font_6x8, Black);
	ssd1306_UpdateScreen();
    system_delayMs(1000);
}

/**
 * @brief Command SSD1306 to display the telemetry data on screen.
 * 
 */
static void displayDataMessage(void) {
	char buff[64];
	uint16_t integer, fraction;
	ssd1306_Fill(White);
	ssd1306_SetCursor(2,1);
	ssd1306_WriteString("Temp:    ", Font_6x8, Black);
    snprintf(buff, sizeof(buff), "%d C", (int)board_temperature);
    ssd1306_WriteString(buff, Font_6x8, Black);

    ssd1306_SetCursor(96,1);
	ssd1306_WriteString("PMB ", Font_6x8, Black);
    snprintf(buff, sizeof(buff), "%d", PMB_ID);
    ssd1306_WriteString(buff, Font_6x8, Black);

    ssd1306_SetCursor(2,11);
	ssd1306_WriteString("Pres:    ", Font_6x8, Black);
	integer = (int) board_pressure;
	fraction = (int)((board_pressure - integer)*100);
    snprintf(buff, sizeof(buff), "%d.%02d kPa", integer, fraction);
    ssd1306_WriteString(buff, Font_6x8, Black);

    ssd1306_SetCursor(2,21);
	ssd1306_WriteString("Current: ", Font_6x8, Black);
	integer = (int)(batt_current/1000);
	fraction = (int)((batt_current/1000.0 - integer)*100);
    snprintf(buff, sizeof(buff), "%d.%02d A", integer, fraction);
    ssd1306_WriteString(buff, Font_6x8, Black);

    ssd1306_SetCursor(2,31);
	ssd1306_WriteString("Voltage: ", Font_6x8, Black);
	integer = (int)(batt_voltage/1000);
	fraction = (int)((batt_voltage/1000.0 - integer)*100);
    snprintf(buff, sizeof(buff), "%d.%02d V", integer, fraction);
    ssd1306_WriteString(buff, Font_6x8, Black);

    ssd1306_SetCursor(2,51);
	ssd1306_WriteString("State:   ", Font_6x8, Black);
	setState(batt_status);
    snprintf(buff, sizeof(buff), "%s", batt_state);
    ssd1306_WriteString(buff, Font_6x8, Black);

    ssd1306_UpdateScreen();
}