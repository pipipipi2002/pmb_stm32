#ifndef INC_BQ34110_H
#define INC_BQ34110_H

#include <math.h>
#include <string.h>
#include <inttypes.h>

#include "system.h"			// Systick`
#include "log.h"			

#ifndef FIRMWARE_DISABLE_DEBUG
#if defined (BQ_DISABLE_DEBUG)
	#define $INFO(fmt, ...)
	#define $ERROR(fmt, ...) 
	#define $SUCCESS(fmt, ...) 
#elif defined (USE_LOGGER)
	#define $INFO(fmt, ...) log_pInfo(fmt, ##__VA_ARGS__)
	#define $ERROR(fmt, ...) log_pError(fmt, ##__VA_ARGS__)
	#define $SUCCESS(fmt, ...) log_pSuccess(fmt, ##__VA_ARGS__)
#else
	#include <stdio.h>
	#define $INFO(fmt, ...) printf(fmt, ##__VA_ARGS__)
	#define $ERROR(fmt, ...) printf(fmt, ##__VA_ARGS__)
	#define $SUCCESS(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif // USE_LOGGER
#endif

#define BQ_BATT_CAPACITY			(15000UL)		// mAH capacity
#define BQ_NO_OF_CELL				(4UL)			// Battery Cell Count
#define BQ_VOLTAGE_DIVIDER			(19000UL) 		// Maximum expected voltage

#define BQ34110_ADDRESS 0x55		// ADDR: 1010101
/*=========================================================================
    DATA FLASH ADDRESS
    -----------------------------------------------------------------------*/
	#define BQ34110_DF_CC_GAIN 		   (0x4000)
	#define BQ34110_DF_CC_DELTA 	   (0x4004)
	#define BQ34110_DF_CC_OFFSET	   (0x4008)
	#define BQ34110_DF_BOARD_OFFSET	   (0x400C)
	#define BQ34110_DF_PACK_V_OFFSET   (0x400F)
	#define BQ34110_DF_VOLTAGE_DIVIDER (0x4010)
	#define BQ34110_DF_SERIES_CELLS    (0x4155)
	#define BQ34110_DF_FLASH_UOV 	   (0x4157)
	#define BQ34110_DF_MAX_PACK_V 	   (0x4088)
	#define BQ34110_DF_MIN_PACK_V 	   (0x408A)
	#define BQ34110_DF_DESIGN_CAP 	   (0x41F5)
	#define BQ34110_DF_PIN_CNTL_CONFIG (0x413D)
	#define BQ34110_DF_LFCC 			0x40C0
	#define BQ34110_DF_CEDV_CONFIG 	   (0x424B)

	#define BQ34110_DF_SOC_LOW_TH 	   (0x418E)
	#define BQ34110_DF_SOC_LOW_RECOV   (0x418F)
	#define BQ34110_DF_FIXED_EDV0       0x425A
	#define BQ34110_DF_FIXED_EDV1      BQ34110_DF_FIXED_EDV0+3
	#define BQ34110_DF_FIXED_EDV2      BQ34110_DF_FIXED_EDV0+6
	#define BQ34110_DF_NEAR_FULL        0x4298
	#define BQ34110_DF_VOLTAGE_ZERO_DOD 0x4263
	#define BQ34110_DF_DESIGN_VOLTAGE   0x41F9
	#define BQ34110_DF_MFG_STATUS_INIT  0x40D7
	#define CEDV_CONFIG_FIXED_EDV0_BIT  5
	#define OP_STATUS_EDV2_BIT 			3
	#define OP_STATUS_VDQ_BIT 			4

/*=========================================================================*/

/*=========================================================================
    DATA COMMAND ADDRESS
    -----------------------------------------------------------------------*/
	#define BQ34110_REG_CNTL 		0x00
	#define BQ34110_REG_TEMP 		0X06
	#define BQ34110_REG_VOLT 		0x08
	#define BQ34110_REG_BSTAT 		0x0A
	#define BQ34110_REG_CURR 		0x0C
	#define BQ34110_REG_RC 			0x10
	#define BQ34110_REG_FCC 		0x12
	#define BQ34110_REG_AVG_CURR 	0x14
	#define BQ34110_REG_TTE 		0x16
	#define BQ34110_REG_TTF 		0x18
	#define BQ34110_REG_ACC_C 		0x1A
	#define BQ34110_REG_ACC_T 		0x1C
	#define BQ34110_REG_LAST_ACC_C 	0x1E
	#define BQ34110_REG_LAST_ACC_T 	0x20
	#define BQ34110_REG_AVG_PWR 	0x24
	#define BQ34110_REG_INT_TEMP 	0x28
	#define BQ34110_REG_CYCLE_C 	0x2A
	#define BQ34110_REG_RSOC 		0x2C
	#define BQ34110_REG_SOH 		0x2E
	#define BQ34110_REG_CV 			0x30
	#define BQ34110_REG_CHARGING_C 	0x32
	#define BQ34110_REG_OP_STATUS	0x3A
	#define BQ34110_REG_DESIGN_CAP 	0x3C
	#define BQ34110_REG_MAC 		0x3E
	#define BQ34110_REG_MAC_DATA 	0x40 // through 0x5F
	#define BQ34110_REG_MAC_DATA_SUM 	0x60
	#define BQ34110_REG_MAC_DATA_LEN 	0x61
	#define BQ34110_REG_ANALOG_COUNT	0x79
	#define BQ34110_REG_RAW_CURR 		0x7A
	#define BQ34110_REG_RAW_VOLT 		0x7C
	#define BQ34110_REG_RAW_INT_TEMP 	0x7E
	#define BQ34110_REG_RAW_EXT_TEMP 	0x80

/*=========================================================================*/

/*=========================================================================
    CONTROL SUBCOMMAND
    All control subcommand data are 2 bytes
    High bytes: 0x00, Low bytes: as defined here
    -----------------------------------------------------------------------*/
#define BQ34110_CNTL_CONTROL_STATUS 0x00
#define BQ34110_CNTL_BOARD_OFFSET 	0x09
#define BQ34110_CNTL_CC_OFFSET 		0x0A
#define BQ34110_CNTL_CC_OFFSET_SAVE 0x0B
#define BQ34110_CNTL_CAL_TOGGLE 	0x2D
#define BQ34110_CNTL_SECURITY_KEYS	0x35
#define BQ34110_CNTL_RESET 			0x41
#define BQ34110_CNTL_MANUF_STATUS	0X57
#define BQ34110_CNTL_PIN_VEN_SET	0x6C
#define BQ34110_CNTL_PIN_VEN_RESET  0x6D
/*=========================================================================*/

extern uint8_t BQ_dataW[10];
extern uint8_t BQ_dataR[10];
extern uint8_t keys[8];

// Test function
void BQ_test(void);

// Functions to read/write BQ internal registers and flash.
uint32_t BQ_ReadRegister(uint8_t add, uint8_t len);
void BQ_WriteControl(uint8_t cntl_data);
int BQ_ReadControl(uint8_t cntl_data);
void BQ_WriteMAC(uint16_t cntl_data);
uint32_t BQ_ReadMAC(uint16_t cntl_data, int bytes);
uint32_t BQ_ReadFlash(uint16_t addr, uint8_t bytes);
void BQ_WriteFlash(uint16_t addr, uint8_t bytes, uint16_t flashDataW);
void BQ_SendCheckSum(uint16_t addr, uint32_t flashDataW);
uint8_t BQ_CalculateCheckSum(uint16_t addr, uint32_t flashData);


// Telemetry data from BQ 
int BQ_AnalogCount(void);
float BQ_GetTemp(void);
uint16_t BQ_GetBattStatus(void);
uint16_t BQ_GetVoltage(void);
uint16_t BQ_GetRawVoltage(void);
int16_t BQ_GetCurrent(void);
int16_t BQ_GetRawCurrent(void);
uint8_t BQ_GetRSOC(void);
uint8_t BQ_GetSOH(void);
int BQ_GetFullCapacity(void);
int BQ_GetRemainingCapacity(void);

// Initialisation and Calibration command for BQ
void BQ_Reset(void);
void BQ_EnterCalibration(void);
void BQ_ExitCalibration(void);
void BQ_Init(void);
void BQ_Calibrate_CCOffset_BoardOffset(void);
void BQ_CalibrateVoltage(uint16_t vApplied);
void BQ_CalibrateVoltageDivider(uint16_t  vApplied);
void BQ_CalibrateVoltagePackOffset(uint16_t vApplied);
void BQ_CalibrateCurrent(int16_t forcedLoadCurrent);
void floatConversion(float val, int* data);
void BQ_WriteCCFlash(uint16_t addr, int rawData[]);
void BQ_RestoreCCSettings(void);
void BQ_SetPinCntlConfig(void);
void BQ_CalibrateNumOfCells(uint32_t num_of_cells);
void BQ_SetVoltageDivider(uint32_t voltage_divider);
void BQ_SetDesignCap(uint32_t designCap);

// Unused Functions from legacy
#ifdef BQ_UNUSED
void BQ_SetFlashUOV(void);
void BQ_SetLFCC(uint32_t LFCC);
void BQ_SetMaxPackV(uint32_t voltage);
void BQ_SetMinPackV(uint32_t voltage);
void BQ_CEDVConfig(void);
void BQ_Learning(void);
void BQ_ReadKeys(void);
void BQ_Unseal(void);
#endif // BQ_UNUSED
#endif // INC_BQ34110_H