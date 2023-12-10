//
// 	BQ34110 Library 
//
#include <libopencm3/stm32/i2c.h>

#include "log.h"
#include "system.h"

#include "BQ34110.h"

uint8_t BQ_dataW[10] = {0};
uint8_t BQ_dataR[10] = {0};
uint8_t keys[8] = {0};

/**
 * Test function to call BQ library functions
*/
void BQ_test() {
	log_pInfo("I am printing from inside BQ");
}

//
//	Functions to read/write BQ34110 internal registers and flash
//

/**
 * @brief Read standard command function in Byte Little Endian format.
 * @param addr Register address.
 * @param len Length of bytes to read. Max 4 bytes.
*/
uint32_t BQ_ReadRegister(uint8_t addr, uint8_t len) {
    uint32_t returnVal = 0;

	BQ_dataW[0] = addr;

	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 1, BQ_dataR, len);
	// HAL_I2C_Mem_Read(&hi2c1, BQ34110_ADDRESS, addr, 1, BQ_dataR, len, 10);

	for (int i=0; i < len; i++) {
		returnVal |= BQ_dataR[i] << (8*i);
	}
    return returnVal;
}

/**
 * @brief Write standard sub-command function.
 * @param cntl_data data to write. Max 1 byte.
 * @paragraph This function can be renamed as BQ_WriteSubcommand also.
 * 	Writing to Control() or ManufacturerAccessControl() has the same effect
 *  Control() will also be sent back to MAC() EXCEPT for CONTROL_STATUS(), which must 
 *  be read back from Control(). 
 * @details From legacy: Fails with HAL_I2C_Mem_Write.
*/
void BQ_WriteControl(uint8_t cntl_data) {
	BQ_dataW[0] = BQ34110_REG_CNTL;
	BQ_dataW[1] = cntl_data;
	BQ_dataW[2] = 0x00;
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 3, 0, 0);
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 3, 10);
}

/**
 * @brief Read standard sub-command function. It will only returns CONTROL_STATUS() most of the time.
 *  To read data that subcommand will return data, refer to BQ_ReadMAC()
 * @param cntl_data sub-command function to read.
 * @details In our use case, this function is only used once. There is a caviat to this function. 
 * 	It will only return the control status datafield except after the DEVICE_TYPE() and FW_VERSION() subcommands.
 * 	After this subcommands, CONTROL_STATUS() will report the value 0xFFA5 one time before reverting to normal
 *  data response. 
 */
int BQ_ReadControl(uint8_t cntl_data) {
	BQ_WriteControl(cntl_data);
	system_delayMs(30);
	return BQ_ReadRegister(BQ34110_REG_CNTL, 2); // Refer to @details above
}

/**
 * @brief Write to ManufacturerAccessControl(). This function only calls the particular MAC function 
 * 	and do not pass any data through. Refer to BQ_WriteFlash() to do that. 
 * @param cntl_data subcommand to access. 2 Bytes Maximum.
 * @details data is sent in little endian (LSB sent first)
 */
void BQ_WriteMAC(uint16_t cntl_data) {
	BQ_dataW[0] = (uint8_t)BQ34110_REG_MAC;
	BQ_dataW[1] = (uint8_t)(cntl_data & 0xFF); 		// LSB
	BQ_dataW[2] = (uint8_t)(cntl_data >> 8); 		// MSB
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 3, 0, 0);
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 3, HAL_MAX_DELAY);
}

/**
 * @brief This function will call the specific MAC function and 
 *  returns the value stored in the MACData() function (or register).
 * @param cntl_data MAC function to read.
 * @param bytes Number of bytes to read. Max 4 Bytes.
 */
uint32_t BQ_ReadMAC(uint16_t cntl_data, int bytes) {
	BQ_WriteMAC(cntl_data);
	system_delayMs(100);
	return BQ_ReadRegister(BQ34110_REG_MAC_DATA, bytes);
}

/**
 * @brief This function will access the DataFlash
 *  triggering the BQ to offload data to MACData() which we will read from.
 * @param addr Flash address to read from.
 * @param bytes Number of bytes to read from.
 * @details Its almost similar to ReadMAC() but follows the datasheet recommended protocol.
 */
uint32_t BQ_ReadFlash(uint16_t addr, uint8_t bytes) {
	uint32_t flashDataR = 0x0000;
	uint16_t mac = 0x0000;

	do {
		BQ_WriteMAC(addr);
		system_delayMs(50);

		// Check if MAC contains the correct subcommand
		mac = BQ_ReadRegister(BQ34110_REG_MAC, 2) & 0xFFFF;
		// mac = BQ_dataR[0] | (BQ_dataR[1] << 8); 
		// HAL_I2C_Mem_Read(&hi2c1, BQ34110_ADDRESS, 0x3E, 1, BQ_dataR, 1, 10);
		// MAC = BQ_dataR[0];
		// HAL_I2C_Mem_Read(&hi2c1, BQ34110_ADDRESS, 0x3F, 1, BQ_dataR, 1, 10);
		// MAC |= BQ_dataR[0] << 8;
		system_delayMs(500);
	}  while (mac != addr);

	// HAL_I2C_Mem_Read(&hi2c1, BQ34110_ADDRESS, BQ34110_REG_MAC_DATA, 1, BQ_dataR, bytes, 10);
	BQ_ReadRegister(BQ34110_REG_MAC_DATA, bytes);
	for (int i = 0; i < bytes; i++) {
		flashDataR |= BQ_dataR[i] << (8*(bytes-i-1)); // Bytes received in Big Endian
	}
	uint8_t readChecksum = BQ_CalculateCheckSum(BQ34110_ADDRESS, flashDataR);
	if (readChecksum != (BQ_ReadRegister(BQ34110_REG_MAC_DATA_SUM, 1) & 0xff)) {
		log_pError("Read Flash Checksum Error");
	}

	if (bytes != (BQ_ReadRegister(BQ34110_REG_MAC_DATA_LEN, 1) & 0xff)) {
		log_pError("Read Flash Len Error");
	}

	return flashDataR;
}

/**
 * @brief This function will access the DataFlash and send data to be written to MACData() in Byte Big Endian,
 *  Does the Checksum and send to MACDataSum(), send (4 + length of bytes) to MACDataLen().
 * @param addr Data Flash Address
 * @param bytes Length of bytes, range = [1,2] 
 * @param flashDataW Data to be sent
 * @details Data is send in big endian (MSB sent first). WriteMac step is sent in little endian (LSB sent first)
 */
void BQ_WriteFlash(uint16_t addr, uint8_t bytes, uint16_t flashDataW) {
	BQ_WriteMAC(addr);
	system_delayMs(50);

	// Sent in big endian
	BQ_dataW[0] = BQ34110_REG_MAC_DATA;
	if (bytes == 2) {
		BQ_dataW[1] = (uint8_t)(flashDataW >> 8);
		BQ_dataW[2] = (uint8_t)(flashDataW & 0xFF);
	}
	else if (bytes == 1) {
		BQ_dataW[1] = (uint8_t)(flashDataW & 0xFF);
	}
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 1 + bytes, 0, 0);
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 1 + bytes, HAL_MAX_DELAY);
	system_delayMs(300);

	BQ_SendCheckSum(addr, flashDataW);

	BQ_dataW[0] = BQ34110_REG_MAC_DATA_LEN;
	BQ_dataW[1] = 0x04 + bytes;
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 2, 0, 0);
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 2, 10);
	system_delayMs(30);
}

/**
 * @brief Calculate the checksum and writes to the device.
 * Checksum is the complement of the sum of the data and data address bytes or
 * ManufacturerAccessControl() and MACData() bytes.
 * @param addr data address bytes.
 * @param bytes Number of bytes in the data (not used).
 * @param flashDataW data to be sent.
*/

void BQ_SendCheckSum(uint16_t addr, uint32_t flashDataW) {

    BQ_dataW[0] = BQ34110_REG_MAC_DATA_SUM;
    BQ_dataW[1] = BQ_CalculateCheckSum(addr, flashDataW);
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 2, 10);
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 2, 0, 0);
	system_delayMs(1000); // bq needs time here!!!
}


/**
 * @brief Calculate Checksum of maximum 2 bytes flash data
 * 
 * @param addr data address bytes
 * @param flashDataW data 
 * @return uint16_t 
 */
uint8_t BQ_CalculateCheckSum(uint16_t addr, uint32_t flashData) {
    uint8_t sum;
    sum = ~ ( (uint8_t)(addr >> 8) + (uint8_t)(addr & 0xFF) +
				(uint8_t)((flashData >> 24) & 0xFF) + (uint8_t)((flashData >> 16) & 0xFF) + 
    		   	(uint8_t)((flashData >> 8) & 0xFF) + (uint8_t)(flashData & 0xFF) );

	return sum;
}

//
//	Get telemetry from BQ34110
//

/**
 * @brief Returns analog calibration counter .
*/
int BQ_AnalogCount() {
	return BQ_ReadRegister(BQ34110_REG_ANALOG_COUNT, 1);
}

/**
 * @brief Get temperature reading. 0x06 and 0x07 0.1K R.
*/
float BQ_GetTemp() {
    return BQ_ReadRegister(BQ34110_REG_TEMP, 2)*0.1 - 273;
}

/**
 * @brief Get battery statuses. Used mainly for Charging/Discharging status.
*/
uint16_t BQ_GetBattStatus(){
	return BQ_ReadRegister(BQ34110_REG_BSTAT, 2);
}

/**
 * @brief Get Voltage Reading. 0x08 and 0x09 mV R
*/
uint16_t BQ_GetVoltage() {
    return BQ_ReadRegister(BQ34110_REG_VOLT, 2);
}

/**
 * @brief Get raw voltage reading. 0x7C and 0x7D.
*/
uint16_t BQ_GetRawVoltage() {
	return BQ_ReadRegister(BQ34110_REG_RAW_VOLT, 2);
}

/**
 * @brief Get current reading. 0x0C and 0x0D mA R
*/
int16_t BQ_GetCurrent() {
    return BQ_ReadRegister(BQ34110_REG_CURR, 2);
}

/**
 * @brief Get raw current reading. 0x7A and 0x7B 
*/
int16_t BQ_GetRawCurrent() {
	return BQ_ReadRegister(BQ34110_REG_RAW_CURR, 2);
}

/**
 * @brief Relative State of Charge. 0x2C & 0x2D % R
 * @details Not Used
*/
uint8_t BQ_GetRSOC() {
    return BQ_ReadRegister(BQ34110_REG_RSOC, 2);
}

/**
 * @brief State of Health. 0x2E & 0x2F % R
 * @details Not Used
*/
uint8_t BQ_GetSOH() {
	return BQ_ReadRegister(BQ34110_REG_SOH, 2);
}

/**
 * @brief Get Full charge capacity. 0x12 and 0x13 mAh R
 * @details Not Used
*/
int BQ_GetFullCapacity() {
    return BQ_ReadRegister(BQ34110_REG_FCC, 2);
}

/**
 * @brief Get remaining capacity. 0x10 and 0x11 mAh R
 * @details Not Used
*/
int BQ_GetRemainingCapacity() {
    return BQ_ReadRegister(BQ34110_REG_RC, 2) ;
}


//
//	Initialisation / Calibration Commands
//

/**
 * @brief Performs full reset
 * 
*/
void BQ_Reset() {
	BQ_WriteControl(BQ34110_CNTL_RESET);

	// Check whether initialisation is complete. 0x3B is the MSB of Operation status.
	uint16_t res;
	do {
		system_delayMs(1000);
		res = BQ_ReadRegister(BQ34110_REG_OP_STATUS, 2); // TODO: Check
		// res = BQ_dataR[0] | (BQ_dataR[1] << 8);
	} while (!((res >> 8) & (1 << 1))); // Check for INITCOMP

}

/**
 * @brief Enter calibration mode. 
 * 	Check whether the CAL_EN bit is set in Manufacturer Status. 
 * @details Similar to the ExitCalibration() function, just that we are checking for set bit.
 */
void BQ_EnterCalibration() {
	uint32_t buffer;
	do {
		BQ_WriteControl(BQ34110_CNTL_CAL_TOGGLE);
		system_delayMs(300);
		
		buffer = BQ_ReadMAC(BQ34110_CNTL_MANUF_STATUS, 2);
	} while (!((buffer >> 8) & (1 << 7))); // Check for CAL_EN bit to be set
}

/**
 * @brief Exit Calibration mode.
 * 	Check whether the CAL_EN bit is cleared.
 * @details Similar to the EnterCalibration() function, just that we are checking for cleared bit.
*/
void BQ_ExitCalibration() {
	uint32_t buffer;
	do {
		BQ_WriteControl(BQ34110_CNTL_CAL_TOGGLE);
		system_delayMs(300);

		buffer = BQ_ReadMAC(BQ34110_CNTL_MANUF_STATUS, 2);
	} while (((buffer >> 8) & (1 << 7))); // Check for CAL_EN bit to be cleared
}

/**
 * @brief This initialisation is for a new battery setup. 
 * The reason for entering the calibration mode is such that programming wouldnt be
 * affected by the voltage drop (in case it happens).
*/
void BQ_Init() {
	log_pInfo("Initialising BQ Init");

	uint32_t batt_capacity, num_of_cells, voltage_divider;
	batt_capacity = BQ_BATT_CAPACITY;
	num_of_cells = BQ_NO_OF_CELL;
	voltage_divider = BQ_VOLTAGE_DIVIDER;

	log_pInfo("Resetting.");
	BQ_Reset();

	log_pInfo("Enter Calibration.");
	BQ_EnterCalibration();

	// BQ_SetFlashUOV();
	
	log_pInfo("Setting Pin Ctl Config.");
	BQ_SetPinCntlConfig(); 					// Enable pin control and set VEN

	log_pInfo("Setting Number of Cells.");
	BQ_CalibrateNumOfCells(num_of_cells);	// Set number of cells (ie. 4)

	log_pInfo("Setting Voltage Divider.");
	BQ_SetVoltageDivider(voltage_divider);	// Set Voltage Divider

	// BQ_SetMaxPackV();
	// BQ_SetMinPackV();

	log_pInfo("Setting Battery Capacity.");
	BQ_SetDesignCap(batt_capacity); 		// Set Battery Capacity

	// BQ_SetLFCC(batt_capacity);	   		// Set Learned Full Charge Capacity at 12000 mAh for first test without learning
	// BQ_CEDVConfig();

	log_pInfo("Exiting Calibration.");
	BQ_ExitCalibration();

	log_pInfo("Calibrating Board Offset.");
	BQ_Calibrate_CCOffset_BoardOffset();

	log_pSuccess("Completed BQ init");	
}


/**
 * @brief Does board and coulomb counter calibration, and saves it.
 * 
*/
void BQ_Calibrate_CCOffset_BoardOffset() {
	BQ_EnterCalibration();
	
	uint32_t buffer;

	do {
		BQ_WriteControl(BQ34110_CNTL_BOARD_OFFSET);
		system_delayMs(1000);
		
		buffer = BQ_ReadControl(BQ34110_CNTL_CONTROL_STATUS);
	} while ( !(buffer & (1 << 5)) || !(buffer & (1 << 4)) ); // Bit 5: CCA; Bit 4: BCA

	do {
		system_delayMs(1000);
		buffer = BQ_ReadControl(BQ34110_CNTL_CONTROL_STATUS);
	} while ( (buffer & (1 << 5)) || (buffer & (1 << 4)) );
	
	BQ_WriteControl(BQ34110_CNTL_CC_OFFSET_SAVE);

	BQ_ExitCalibration();
}

/**
 * @brief Wrapper function to calibrate both voltage divider and offset.
 * 
 * @param vApplied Voltage applied during calibration.
 */
void BQ_CalibrateVoltage(uint16_t vApplied) {
	log_pInfo("Initialising Voltage Calibration");	

	BQ_CalibrateVoltageDivider(vApplied);
	BQ_CalibrateVoltagePackOffset(vApplied);
	
	log_pSuccess("Completed Voltage Calibration");	
}

/**
 * @brief function to calibrate voltage divider.
 * Takes 50 samples of the voltage input and 
 * calculate the gain to get the applied voltage.
 * Saves to the data flash.
 * 
 * @param vApplied Voltage applied during calibration
 */
void BQ_CalibrateVoltageDivider(uint16_t vApplied) {
	log_pInfo("Calibrating Voltage Divider");
	
	/* Start Raw Data Reading */
	uint8_t samplesToAvg = 50;
	uint32_t avgRawVoltage = 0;

	uint8_t loopCount = 0;
	uint32_t rawDataSum = 0;
	uint16_t counterNow = BQ_AnalogCount();
	uint16_t counterPrev = counterNow;

	uint16_t wait = 200;

	BQ_EnterCalibration();

	log_pInfo("Sampling Raw Data.");

	while(loopCount < samplesToAvg) {
		if (counterNow != counterPrev) {
			rawDataSum += BQ_GetRawVoltage();
			loopCount++;
			counterPrev = counterNow;
			log_pInfo("Raw Data: %ld", rawDataSum);
		} else {
			system_delayMs(wait);
			counterNow = BQ_AnalogCount();
		}
	}

	BQ_ExitCalibration();

	avgRawVoltage = rawDataSum / samplesToAvg;
	/* Stop Raw Data Reading */

	/* Start write new Voltage Divider to DF */
	uint32_t data;

	// Round new_divider as needed to an unsigned 16- bit value.
	// This value cannot exceed 65535.
	uint32_t curr_divider = BQ_ReadFlash(BQ34110_DF_VOLTAGE_DIVIDER, 2);
	uint16_t new_divider = (uint16_t)(vApplied * curr_divider / avgRawVoltage);
	log_pInfo("Curr Divider: %ld", curr_divider);
	log_pInfo("New Divider: %d", new_divider);
	do {
		BQ_WriteFlash(BQ34110_DF_VOLTAGE_DIVIDER, 0x02, new_divider);
		data = BQ_ReadFlash(BQ34110_DF_VOLTAGE_DIVIDER, 2);
	} while (data != new_divider);
	log_pSuccess("Completed Voltage divider calibration ");
	/* Stop write new Voltage Divider to DF */
}

/**
 * @brief Calibrate voltage pack offset.
 * @param vApplied voltage applied during calibration.
*/
void BQ_CalibrateVoltagePackOffset(uint16_t vApplied) {
	log_pInfo("Calibrating Voltage Pack Offset.");
	/* Start Raw */
	uint8_t samplesToAvg = 50;
	uint32_t avgRawVoltage = 0;

	/* Start Raw Data Reading */
	uint8_t loopCount = 0;
	uint32_t rawDataSum = 0;
	uint16_t counterNow = BQ_AnalogCount();
	uint16_t counterPrev = counterNow;

	uint16_t wait = 200;

	BQ_EnterCalibration();
	log_pInfo("Sampling Raw Data.");

	for (loopCount = 0; loopCount < samplesToAvg; ) {
		if (counterNow != counterPrev) {
			rawDataSum += BQ_GetRawVoltage();
			loopCount++;
			counterPrev = counterNow;
			log_pInfo("Raw Data: %ld", rawDataSum);
		} else {
			system_delayMs(wait);
			counterNow = BQ_AnalogCount();
		}
	}

	BQ_ExitCalibration();
	avgRawVoltage = rawDataSum / samplesToAvg;
	/* Stop Raw Data Reading */

	/* Start writing pack offset voltage */
	// Original Equation: 
	// int vOffset = (uint8_t)(vApplied - avgRawVoltage);
	// But since vOffset is a signed one byte data, its value ranges from -128 to 127
	
	int vOffset;
	if (vApplied - avgRawVoltage > 127) {vOffset = 127;}
	else if (avgRawVoltage - vApplied > 128) {vOffset = -128;}
	else {vOffset = (vApplied - avgRawVoltage);}
	
	log_pInfo("Voffset: %d", vOffset);

	BQ_WriteMAC(BQ34110_DF_PACK_V_OFFSET);
	system_delayMs(50);

	// We dont use the writeFlash() function as we might be writing negative values.
	BQ_dataW[0] = BQ34110_REG_MAC_DATA;
	BQ_dataW[1] = vOffset;
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 2, HAL_MAX_DELAY);
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 2, 0, 0);
	system_delayMs(300);

	BQ_SendCheckSum(BQ34110_DF_PACK_V_OFFSET, vOffset);

	BQ_dataW[0] = BQ34110_REG_MAC_DATA_LEN;
	BQ_dataW[1] = 0x05;
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 2, 10);
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 2, 0, 0);
	
	system_delayMs(30);
	/* Stop writing pack offset voltage */
	log_pSuccess("Completed V offset calibration.");
}

void BQ_CalibrateCurrent(int16_t forcedLoadCurrent) {
	log_pInfo("Starting Current Calibration in 5s");
	system_delayMs(5000);
	log_pInfo("Starting Current Calibration");

	int16_t ccOffset;
	int8_t boardOffset;
	ccOffset = BQ_ReadFlash(BQ34110_DF_CC_OFFSET, 2);
	boardOffset = BQ_ReadFlash(BQ34110_DF_BOARD_OFFSET, 1);

	/* Start Raw Current Reading */
	uint8_t samplesToAvg = 15;
	float avgRawCurrent = 0;
	
	uint8_t loopCount = 0;
	int16_t rawDataSum = 0;
	uint16_t counterNow = BQ_AnalogCount(); 
	uint16_t counterPrev = counterNow;
	
	uint16_t wait = 200;

	BQ_EnterCalibration();

	log_pInfo("Sampling Raw Data.");

	while (loopCount < samplesToAvg) {
		if (counterNow != counterPrev) {
			rawDataSum += BQ_GetRawCurrent();
			loopCount++;
			counterPrev = counterNow;
			log_pInfo("Raw Data: %d", rawDataSum);
		} else {
			system_delayMs(wait);
			counterNow = BQ_AnalogCount();
		}
	}
	BQ_ExitCalibration();

	avgRawCurrent = ((float)rawDataSum)/samplesToAvg;
	/* Stop Raw Current reading */

	/* Start Current calculation */
	float ccGain, ccDelta;
	ccGain = (forcedLoadCurrent / (avgRawCurrent - (ccOffset + boardOffset) / 16.0));
	ccDelta = (ccGain * 1193046);
	log_pInfo("avgRawCurrent: %f", avgRawCurrent);
	log_pInfo("ccOffset: %d", ccOffset);
	log_pInfo("boardOffset: %d", boardOffset);
	log_pInfo("CC Gain: %f", ccGain);
	log_pInfo("CC Delta: %f", ccDelta);

	int ccGain_df[4], ccDelta_df[4];
	for (int i = 0; i<4; i++) {
		ccGain_df[i] = 0;
		ccDelta_df[i] = 0;
	}
	log_pInfo("Converting Data points");
	floatConversion(ccGain, ccGain_df);
	floatConversion(ccDelta, ccDelta_df);
	/* Stop Current Calculation */

	/* Start Writing to Data flash */
	log_pInfo("Writing Data to DF");
	uint32_t data1, data2;

	do {
		BQ_WriteCCFlash(BQ34110_DF_CC_GAIN, ccGain_df);
		data1 = BQ_ReadFlash(BQ34110_DF_CC_GAIN, 2);
		data2 = BQ_ReadFlash(BQ34110_DF_CC_GAIN + 2, 2);
	} while (data1 != (uint32_t)((ccGain_df[0] << 8)| ccGain_df[1]) &&
			 data2 != (uint32_t)(ccGain_df[2] << 8 | ccGain_df[3]));

	do {
		BQ_WriteCCFlash(BQ34110_DF_CC_DELTA, ccDelta_df);
		data1 = BQ_ReadFlash(BQ34110_DF_CC_DELTA, 2);
		data2 = BQ_ReadFlash(BQ34110_DF_CC_DELTA + 2, 2);
	} while (data1 != (uint32_t)(ccDelta_df[0] << 8 | ccDelta_df[1]) &&
			 data2 != (uint32_t)(ccDelta_df[2] << 8 | ccDelta_df[3]));

	system_delayMs(300);

	log_pSuccess("Current Calibration Complete");
	/* Stop Writing to data flash */
}

/**
 * @brief Float conversion for CCGain and CCDelta.
 * 
 * @param val CC Gain or CC Delta value.
 * @param data Array of 4 bytes of converted values.
 */
void floatConversion(float val, int* data) {
	float tmpVal, mod_val;
	uint16_t byte2, byte1, byte0;
	int exp;

	exp = 0;

	mod_val = (val < 0) ? val * (-1) : val;

	tmpVal = mod_val;
	tmpVal *= (1 + pow(2, -25));

	if (tmpVal < 0.5) {
		while (tmpVal < 0.5) {
			tmpVal *= 2;
			exp--;
		}
	}
	else if (tmpVal >= 1.0) {
		while (tmpVal >= 1.0) {
			tmpVal /= 2;
			exp++;
		}
	}

	if (exp > 127) exp = 127;
	else if (exp < -128) exp = -128;

	tmpVal = pow(2, 8 - exp) * mod_val - 128;
	byte2 = (int)tmpVal;
	tmpVal = pow (2, 8) * (tmpVal - byte2);
	byte1 = (int)tmpVal;
	tmpVal = pow(2, 8) * (tmpVal - byte1);
	byte0 = (int)tmpVal;

	if (val < 0) {
		byte2 |= 0x80;
	}

	data[0] = exp + 128;
	data[1] = byte2;
	data[2] = byte1;
	data[3] = byte0;
}

/**
 * @brief For Current Calibration CCGain and CCDelta. 
 * Similar to Write Flash, just that we are writing 4 bytes here.
*/
void BQ_WriteCCFlash(uint16_t addr, int rawData[]) {
	BQ_WriteMAC(addr);
	system_delayMs(50);

	// Sent in big endian
	BQ_dataW[0] = BQ34110_REG_MAC_DATA;
	BQ_dataW[1] = (uint8_t)(rawData[0]);
	BQ_dataW[2] = (uint8_t)(rawData[1]);
	BQ_dataW[3] = (uint8_t)(rawData[2]);
	BQ_dataW[4] = (uint8_t)(rawData[3]);
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 5, HAL_MAX_DELAY);
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 5, 0, 0);
	system_delayMs(300);

	// Checksum
	uint32_t data = (rawData[0] | (rawData[1] << 8) | (rawData[2] << 16) | (rawData[3] << 24));
	BQ_SendCheckSum(addr, data);
    // uint16_t summ = 0;
    // summ = ~ ( (uint8_t)(addr >> 8) + (uint8_t)(addr & 0xFF) +
    // 		   (uint8_t)(rawData[0]) + (uint8_t)(rawData[1]) +
	// 		   (uint8_t)(rawData[2]) + (uint8_t)(rawData[3]) );
    // BQ_dataW[0] = BQ34110_REG_MAC_DATA_SUM;
    // BQ_dataW[1] = summ;
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 2, 10);
	// system_delayMs(500); //bq needs time here!!!

	BQ_dataW[0] = BQ34110_REG_MAC_DATA_LEN;
	BQ_dataW[1] = 0x04 + 0x04;
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 2, 10);
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 2, 0, 0);

	system_delayMs(30);
}

/**
 * @brief To restore Current Calibration Gain and Delta to default value.
 * 
 */
void BQ_RestoreCCSettings() {
	int ccGain_df[4], ccDelta_df[4];
	uint32_t data1, data2;
	for (int i = 0; i<4; i++) {
		ccGain_df[i] = 0;
		ccDelta_df[i] = 0;
	}

	floatConversion(0.47095, ccGain_df);
	floatConversion(5.677446*pow(10,5), ccDelta_df);

	BQ_EnterCalibration();

	do {
		BQ_WriteCCFlash(BQ34110_DF_CC_GAIN, ccGain_df);
		data1 = BQ_ReadFlash(BQ34110_DF_CC_GAIN, 2);
		data2 = BQ_ReadFlash(BQ34110_DF_CC_GAIN + 2, 2);
	} while (data1 != (uint32_t)(ccGain_df[0] << 8 | ccGain_df[1]) &&
			 data2 != (uint32_t)(ccGain_df[2] << 8 | ccGain_df[3]));

	BQ_WriteCCFlash(BQ34110_DF_CC_DELTA, ccDelta_df);

	BQ_ExitCalibration();

	system_delayMs(300);
}

/**
 * @brief Enable pin control and set VEN for multi cell applications
 * @details manufacturer status init: Type H2; Min 0x0; Max 0xFFF; Hex
 * @details pin control config: Type H1; Min 0x0; Max 0x1F; Hex
 */
void BQ_SetPinCntlConfig() {
	uint32_t pinCntlConfig, mfgStat;

	do {
		// Set PCTL_EN to unlock more subcommands
		mfgStat = BQ_ReadFlash(BQ34110_DF_MFG_STATUS_INIT, 2);
		BQ_WriteFlash(BQ34110_DF_MFG_STATUS_INIT, 0x02, mfgStat | (1 << 4));

		// Set VEN to enable multicell
		pinCntlConfig = BQ_ReadFlash(BQ34110_DF_PIN_CNTL_CONFIG, 1);
		BQ_WriteFlash(BQ34110_DF_PIN_CNTL_CONFIG, 0x01, pinCntlConfig | (1 << 4));

		pinCntlConfig = BQ_ReadFlash(BQ34110_DF_PIN_CNTL_CONFIG, 1) & (1<<4);
		log_pInfo("pin Cntl config: 0x%X", pinCntlConfig);
	} while (pinCntlConfig != 0x10);
}

/**
 * @brief Set the number of cells of the LiPo battery.
 * @details Type U1; Min 1; Max 100; Units Num
*/
void BQ_CalibrateNumOfCells(uint32_t numOfCells) {
	uint32_t currNumOfCells = 0;
	currNumOfCells = BQ_ReadFlash(BQ34110_DF_SERIES_CELLS, 1);

	while (currNumOfCells != numOfCells) {
		BQ_WriteFlash(BQ34110_DF_SERIES_CELLS, 0x01, numOfCells);
		currNumOfCells = BQ_ReadFlash(BQ34110_DF_SERIES_CELLS, 1);
	}
}

/**
 * @brief Set the voltage divider gain.
 * @details Type U2; Min 0; Max 65535; Units mV
*/
void BQ_SetVoltageDivider(uint32_t voltage_divider) {
	uint32_t curr_voltage_divider;
	curr_voltage_divider = BQ_ReadFlash(BQ34110_DF_VOLTAGE_DIVIDER, 2);

	while (curr_voltage_divider != voltage_divider) {
		BQ_WriteFlash(BQ34110_DF_VOLTAGE_DIVIDER, 0x02, voltage_divider);
		curr_voltage_divider = BQ_ReadFlash(BQ34110_DF_VOLTAGE_DIVIDER, 2);
	}
}

/**
 * @brief Set battery design capacity in mAh
 * @details Type I2; Min 0; Max 32767; unit mAh
*/
void BQ_SetDesignCap(uint32_t designCap) {
	uint32_t flashDesignCap;

	//BQ_WriteFlash(BQ34110_DF_DESIGN_CAP, 0x02, 0x0898);
	do {
		BQ_WriteFlash(BQ34110_DF_DESIGN_CAP, 0x02, designCap);
		flashDesignCap = BQ_ReadFlash(BQ34110_DF_DESIGN_CAP, 2);
	} while (flashDesignCap != designCap);
}

//
//	UNUSED FUNCTIONS FROM LEGACY CODE by Nathania
//

#ifdef BQUNUSED

//  0x4157 Flash Update OK Voltage I2 0 5000 2800 mV
void BQ_SetFlashUOV() {
	int32_t flashUOV = 0;
//	BQ_WriteFlash(BQ34110_DF_FLASH_UOV, 0x02, 0x0AF0);
	BQ_WriteFlash(BQ34110_DF_FLASH_UOV, 0x02, 0x0064);

	flashUOV = BQ_ReadFlash(BQ34110_DF_FLASH_UOV, 2);
	system_delayMs(10);
}

/**
 * @brief Set learned full charge capacity.
 * @details Type I2; Min 0; Max 32767; unit mAh
*/
void BQ_SetLFCC(uint32_t LFCC) {
	uint32_t curr_LFCC = 0;
	curr_LFCC = BQ_ReadFlash(BQ34110_DF_LFCC, 2);
	if (curr_LFCC != LFCC) {
		do {
			BQ_WriteFlash(BQ34110_DF_LFCC, 0x02, LFCC);
			curr_LFCC =  BQ_ReadFlash(BQ34110_DF_LFCC, 2);
		} while (curr_LFCC != LFCC);
	}
}

/**
 * @brief Set maximum Pack Voltage
 * @param voltage The maximum pack voltage
 * @details Type I2; Min 0; Max 32767; Unit 20mV
*/
void BQ_SetMaxPackV(uint32_t voltage) {
	uint32_t maxPackV;
	do {
		BQ_WriteFlash(BQ34110_DF_MAX_PACK_V, 0x02, voltage);
		maxPackV = BQ_ReadFlash(BQ34110_DF_MAX_PACK_V, 2);
	} while (maxPackV != voltage);
	// BQ_WriteFlash(BQ34110_DF_MAX_PACK_V, 0x02, 0x00A0);
	// BQ_WriteFlash(BQ34110_DF_MAX_PACK_V, 0x02, 0x0348);
	// maxPackV = BQ_ReadFlash(BQ34110_DF_MAX_PACK_V, 2);
}

/**
 * @brief Set minimum pack voltage
 * @param voltage The minimum pack voltage
 * @details Type I2; Min 0; Max 32768; unit 20mV
*/
void BQ_SetMinPackV(uint32_t voltage) {
	uint32_t minPackV;
	do {
		BQ_WriteFlash(BQ34110_DF_MIN_PACK_V, 0x02, voltage);
		minPackV = BQ_ReadFlash(BQ34110_DF_MIN_PACK_V, 2);
	} while (minPackV != voltage);
	// BQ_WriteFlash(BQ34110_DF_MIN_PACK_V, 0x02, 0x00AF);
	// BQ_WriteFlash(BQ34110_DF_MIN_PACK_V, 0x02, 0x02E6);
	// minPackV = BQ_ReadFlash(BQ34110_DF_MIN_PACK_V, 2);
}




void BQ_CEDVConfig() {
	uint32_t data = 0;
	int16_t curr_voltageDOD[11];
	int16_t voltageDOD[11] = {4136, 4018, 3922, 3845, 3775, 3730, 3716, 3697, 3614, 3467, 2759};
	// int16_t voltageDOD[11] = {4173, 4043, 3925, 3821, 3725, 3656, 3619, 3582, 3515, 3438, 2713};
	// int16_t fixedEDV[3] = { 1768, 1975, 2042 };
	// int16_t fixedEDV[3] = {3031, 3385, 3501};
	int16_t fixedEDV[3] = {3738, 4175, 4318};

//Set CEDV Gauging Config
//	BQ_WriteFlash(BQ34110_DF_CEDV_CONFIG, 0x01, (1 << FIXED_EDV0_BIT));
//	data = BQ_ReadFlash(BQ34110_DF_DESIGN_VOLTAGE, 2);
//	system_delayMs(10);

	//Set Design Voltage to 3700

	do {
		BQ_WriteFlash(BQ34110_DF_DESIGN_VOLTAGE, 0x02, 3700);
		data = BQ_ReadFlash(BQ34110_DF_DESIGN_VOLTAGE, 2);
	} while (data != 3700);

	//Set SOC Low Threshold to 25%
	do {
		BQ_WriteFlash(BQ34110_DF_SOC_LOW_TH, 0x01, 10);
		BQ_WriteFlash(BQ34110_DF_SOC_LOW_TH, 0x01, 25);
		data = BQ_ReadFlash(BQ34110_DF_SOC_LOW_TH, 1);
	} while (data != 25);


	//Set SOC Low Recovery to 35%
	do {
		BQ_WriteFlash(BQ34110_DF_SOC_LOW_RECOV, 0x01, 35);
		data = BQ_ReadFlash(BQ34110_DF_SOC_LOW_RECOV, 1);
	} while (data != 35);

	//Set Voltage (0-100) % DOD
	for (int i = 0; i < 11; i++) {
		curr_voltageDOD[i] = BQ_ReadFlash(BQ34110_DF_VOLTAGE_ZERO_DOD + 2*i, 2);
		do {
			BQ_WriteFlash(BQ34110_DF_VOLTAGE_ZERO_DOD + 2*i, 0x02, voltageDOD[i]);
			data = BQ_ReadFlash(BQ34110_DF_VOLTAGE_ZERO_DOD + 2*i, 2);
		} while (data != voltageDOD[i]);
	}

	//Set Fixed EDV 0, 1, 2
	for (int i = 0; i < 3; i++) {
		do {
			BQ_WriteFlash(BQ34110_DF_FIXED_EDV0 + 3*i, 0x02, fixedEDV[i]);
			data = BQ_ReadFlash(BQ34110_DF_FIXED_EDV0 + 3*i, 2);
		} while (data != fixedEDV[i]);
	}

	// Battery Low Set Threshold
	do {
		BQ_WriteFlash(0x4184, 0x02, 1838);
		data = BQ_ReadFlash(0x4184, 2);
	} while (data != 1838);

	// Battery Low Clear Threshold
	do {
		BQ_WriteFlash(0x4187, 0x02, 1938);
		data = BQ_ReadFlash(0x4187, 2);
	} while (data != 1938);

	// Operation Config A
	do {
		BQ_WriteFlash(0x413A, 0x02, 0x8200);
		data = BQ_ReadFlash(0x413A, 2);
	} while (data != 0x8200);
}

void BQ_Learning() {
	uint16_t opStatus, FCC, LFCC, RC;
	uint8_t VDQbit, EDV2bit;
//	do {
//		opStatus = BQ_ReadRegister(BQ34110_REG_OP_STATUS, 2);
//		VDQbit = opStatus & (1 << OP_STATUS_VDQ_BIT);
//	} while ( VDQbit == 0);

	do {
		opStatus = BQ_ReadRegister(BQ34110_REG_OP_STATUS, 2);
		VDQbit = opStatus & (1 << OP_STATUS_VDQ_BIT);
		EDV2bit = opStatus & (1 << OP_STATUS_EDV2_BIT);
		FCC = BQ_ReadRegister(BQ34110_REG_FCC, 2);
		LFCC = BQ_ReadFlash(BQ34110_DF_LFCC, 2);
		RC = BQ_ReadRegister(BQ34110_REG_RC,2);
	} while (VDQbit == 1 && EDV2bit == 0);
}

void BQ_ReadKeys() {
	BQ_WriteControl(BQ34110_CNTL_SECURITY_KEYS);
	system_delayMs(10);

	// HAL_I2C_Mem_Read(&hi2c1, BQ34110_ADDRESS, BQ34110_REG_MAC_DATA, 1, keys, 8, 10);
	BQ_dataW[0] = BQ34110_REG_MAC_DATA;
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 1, keys, 8);
}

void BQ_Unseal() {
	uint8_t status;

	status = BQ_ReadRegister(BQ34110_REG_OP_STATUS, 1) & 0x06;
	if (status == 0x04) printf("Unsealed Initial Status");
	else if (status == 0x06) printf("Sealed Initial Status");
	else if (status == 0x02) printf("Full Access Initial Status");
	else if (status == 0x00) printf("Invalid Initial Status");

	BQ_ReadKeys();

	BQ_dataW[0] = BQ34110_REG_CNTL;
	BQ_dataW[1] = keys[0];
	BQ_dataW[2] = keys[1];
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 3, 10);
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 3, 0, 0);
	system_delayMs(10);

	BQ_dataW[0] = BQ34110_REG_CNTL;
	BQ_dataW[1] = keys[2];
	BQ_dataW[2] = keys[3];
	// HAL_I2C_Master_Transmit(&hi2c1, BQ34110_ADDRESS, BQ_dataW, 3, 10);
	i2c_transfer7(I2C1, BQ34110_ADDRESS, BQ_dataW, 3, 0, 0);
	system_delayMs(10);

	status = BQ_ReadRegister(BQ34110_REG_OP_STATUS, 1) & 0x06;
	if (status == 0x04) printf("Unsealed Status");
	else if (status == 0x06) printf("Sealed Status, Failed to Unseal");
	else if (status == 0x02) printf("Full Access Status, No Need to Unseal");
	else if (status == 0x00) printf("Invalid Status, No State was Loaded");
}

#endif // BQ_UNUSED