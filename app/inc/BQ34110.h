#ifndef INC_BQ34110_H
#define INC_BQ34110_H

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdio.h>
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_i2c.h"
#include "stm32f0xx_hal_cortex.h"
#include "../pmb_define.h"

#define BQ34110_ADDRESS 0xAA
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

extern I2C_HandleTypeDef hi2c1;

uint8_t BQ_dataW[10];
uint8_t BQ_dataR[10];
uint8_t keys[8];

// Test function
void BQ_test();

// Functions to read/write BQ internal registers and flash.
uint32_t BQ_ReadRegister(uint8_t add, uint8_t len);
void BQ_WriteControl(uint8_t cntl_data);
int BQ_ReadControl(uint8_t cntl_data);
void BQ_WriteMAC(uint16_t cntl_data);
uint32_t BQ_ReadMAC(uint16_t cntl_data, int bytes);
uint32_t BQ_ReadFlash(uint16_t addr, uint8_t bytes);
void BQ_WriteFlash(uint16_t addr, uint8_t bytes, uint16_t flashDataW) ;
void BQ_CheckSum(uint16_t addr, uint8_t bytes, uint16_t flashDataW) ;

// Telemetry data from BQ 
int BQ_AnalogCount();
float BQ_GetTemp();
uint16_t BQ_GetBattStatus();
uint32_t BQ_GetVoltage();
uint32_t BQ_GetRawVoltage();
int16_t BQ_GetCurrent();
int16_t BQ_GetRawCurrent();
uint8_t BQ_GetRSOC();
uint8_t BQ_GetSOH();
int BQ_GetFullCapacity();
int BQ_GetRemainingCapacity();

// Initialisation and Calibration command for BQ
void BQ_Reset();
void BQ_EnterCalibration();
void BQ_ExitCalibration();
void BQ_Init();
void BQ_Calibrate_CCOffset_BoardOffset();
void BQ_CalibrateVoltage(uint16_t vApplied);
void BQ_CalibrateVoltageDivider(uint16_t  vApplied);
void BQ_CalibrateVoltagePackOffset(uint16_t vApplied);
void BQ_CalibrateCurrent(int16_t forcedLoadCurrent);
void floatConversion(float val, int* data);
void BQ_WriteCCFlash(uint16_t addr, int rawData[]);
void BQ_RestoreCCSettings();
void BQ_SetPinCntlConfig();
void BQ_CalibrateNumOfCells(uint32_t num_of_cells);
void BQ_SetVoltageDivider(uint32_t voltage_divider);

// Unused Functions from legacy
void BQ_SetFlashUOV();
void BQ_SetLFCC(uint32_t LFCC);
void BQ_SetMaxPackV(uint32_t voltage);
void BQ_SetMinPackV(uint32_t voltage);
void BQ_SetDesignCap(uint32_t designCap);
void BQ_CEDVConfig();
void BQ_Learning();
void BQ_ReadKeys();
void BQ_Unseal();
#endif // INC_BQ34110_H