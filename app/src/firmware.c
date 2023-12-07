// Libopencm3 Header Files
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>

// Own Header Files
#include "firmware.h"
#include "system.h"
#include "uart.h"
#include "log.h"
#include "gpio.h"
#include "i2c.h"
#include "ADS1115.h"
#include "retarget.h"
#include "BQ34110.h"
#include "SSD1306/ssd1306.h"
#include "SSD1306/ssd1306_fonts.h"

/*
 * Global Variables 
 */
// Telemetry Data
uint16_t batt_voltage = 0;
int32_t batt_current = 0;
uint16_t batt_status = 0;
char batt_state[15] = "";
float board_temperature = 0;
float board_pressure = 0;

// Timer for "cyclic functions"
uint64_t updateTimer = 0;
uint64_t CAN_BattTimer = 0;
uint64_t CAN_BoardTimer = 0;
uint64_t CAN_HbTimer = 0;
uint64_t displayTimer = 0;

// CAN Messages Placeholder
canMsg_tu canBattMsg, canBoardMsg;
canMsg_tu canHbMsg = {.heartbeatId = BB_HEARTBEAT_ID_PMB};

/*
 * Internal Function Declarations
 */
int main(void);
static void setup(void);
static float getPressure(void);
static void poweroff(void);
static void encodeCanMsgBattStat(void);
static void encodeCanMsgBoardStat(void);
static void displayOnMessage(void);
static void displayOffMessage(void);
static void displayDataMessage(void);
static void setState(uint16_t status);

/*
 * Function Defintions
 */

int main(void) {
    setup();

    displayOnMessage();

    log_pInfo("Power Monitoring Board AUV4");
    log_pInfo("PMB Number: %d", PMB_ID);

    /* Supply power (PMB circuit is already on beforehand, just for completeness) */
    gpio_clear(PMB_RELAY_OFF_PORT, PMB_RELAY_OFF_PIN);      // Power to PMB Circuit
    // TODO Do some verification before supplying power to vehicle
    gpio_set(PMB_PMOS_ON_GPIO_PORT, PMB_PMOS_ON_GPIO_PIN);  // Power to Vehicle
    log_pSuccess("PMOS ON, Battery connected to Vehicle");

#if (PMB_PERFORM_CALIBRATION == 1)
    log_pInfo("Starting PMB BQ Calibration");
    BQ_Init();
    BQ_CalibrateVoltage(PMB_CALIBRATION_VOLTAGE);
    BQ_CalibrateCurrent(PMB_CALIBRATION_CURRENT / PMB_CURRENT_SCALE);
    log_pSuccess("Completed PMB BQ Calibration");
#endif // PMB_PERFORM_CALIBRATION

    while (true) {
        /* Check for off reed switch activation */
        if (gpio_get(PMB_REED_OFF_PORT, PMB_REED_OFF_PIN)) {
            log_pSuccess("Shutdown Sequence Activated");
            poweroff();
            while (1); // Will not reach here
        }

        /* Update PMB and Battery Stats */
        if (PMB_system_getTicks() - updateTimer > PMB_STATUS_UPDATE_INTVL) {
            log_pInfo("Updating internal data");
            updateTimer = PMB_system_getTicks();
            batt_status = BQ_GetBattStatus();
            batt_current = BQ_GetCurrent() * PMB_CURRENT_SCALE;
            batt_current = (batt_current < 0) ? -1 * batt_current : batt_current;
            batt_voltage = BQ_GetVoltage();
            board_temperature = BQ_GetTemp();
            board_pressure = getPressure();
        }

        /* Send Battery Stats via CAN */
        if (PMB_system_getTicks() - CAN_BattTimer > PMB_CAN_BATT_MSG_INTVL) {
            log_pInfo("Sending Battery Statistics via CAN");
            CAN_BattTimer = PMB_system_getTicks();

            encodeCanMsgBattStat();
            can_sendCanMsg(&canBattMsg, BB_CAN_ID_BATT_STAT);
        }

        /* Send Board Stats via CAN */
        if (PMB_system_getTicks() - CAN_BoardTimer > PMB_CAN_BOARD_MSG_INTVL) {
            log_pInfo("Sending Board Statistics via CAN");
            CAN_BoardTimer = PMB_system_getTicks();

            encodeCanMsgBoardStat();
            can_sendCanMsg(&canBoardMsg, BB_CAN_ID_PMB_STAT);
        }
        
        /* Send Heartbeat via CAN */
        if (PMB_system_getTicks() - CAN_HbTimer > PMB_CAN_HB_MSG_INTVL) {
            log_pInfo("Sending Heartbeat via CAN");
            CAN_HbTimer = PMB_system_getTicks();
            can_sendCanMsg(&canHbMsg, BB_CAN_ID_HEARTBEAT);
        }

        /* Update Display */
        if (PMB_system_getTicks() - displayTimer > PMB_OLED_REFRESH_INTVL) {
            displayTimer = PMB_system_getTicks();
            displayDataMessage();
        }
    }
    return 0;
}

static void setup(void) {
    PMB_system_init();    
    PMB_uart_init();
    retarget_init();
    PMB_gpio_init();

    while(!PMB_can_init()) PMB_system_delayMs(1000);
    while(!PMB_i2c_init()) PMB_system_delayMs(1000);

    ssd1306_Init();

    log_pSuccess("Setup Completed");
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
    PMB_system_delayMs(500);
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
    PMB_system_delayMs(1000);
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
    PMB_system_delayMs(1000);
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