/*
 * W25Qxxx.c
 *
 *  Created on: Aug 11, 2024
 *      Author: marka
 */

#ifndef W25QXXX_H_
#include "../Inc/W25Qxxx.h"
#endif


uint8_t W25Q_Init(W25Q *wq, GPIO_TypeDef *nCSPort, GPIO_TypeDef *nWPPort, GPIO_TypeDef *nHOLDPort,
					uint32_t nCSPin, uint32_t nWPPin, uint32_t nHOLDPin, SPI_HandleTypeDef *hspi,
					uint8_t devID, uint8_t isQuadChip, uint8_t driveStrength) {

	// Map struct GPIO Ports & Pins to passed parameters
	wq->nCSPort = nCSPort;
	wq->nWPPort = nWPPort;
	wq->nHOLDPort = nHOLDPort;

	wq->nCSPin = nCSPin;
	wq->nWPPin = nWPPin;
	wq->nHOLDPin = nHOLDPin;

	wq->hspi = hspi;

	wq->quadEnable = isQuadChip;		// Default state depends on PN of chip.
	wq->writeEnable = 0;

	// Set CS pin high
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);

	// Set hold pin high (Disable)
	HAL_GPIO_WritePin(wq->nHOLDPort, wq->nHOLDPin, GPIO_PIN_SET);

	// Set WP pin high (disable)
	HAL_GPIO_WritePin(wq->nWPPort, wq->nWPPin, GPIO_PIN_SET);

	// HAL status handle used to indicate error messages
	HAL_StatusTypeDef halRet = HAL_OK;

	uint8_t returnInc = 1;			// Integer error code return, incremented after successful things.

	uint8_t errorsEnabled = 1;		// Use for debugging

	// Release from power down mode
	halRet = W25Q_ReleasePowerDown(wq);
	if(halRet != HAL_OK && errorsEnabled)
		return returnInc;
	else
		returnInc++;

	// Reset Device
	halRet = W25Q_ChipReset(wq);
	if(halRet != HAL_OK && errorsEnabled)
		return returnInc;
	else
		returnInc++;

	// Read initial Status Registers
	halRet = W25Q_ReadStatusRegs(wq);
	if(halRet != HAL_OK && errorsEnabled)
		return returnInc;
	else
		returnInc++;

	// Increase driver strength?
	halRet = W25Q_SetDriverStrength(wq, driveStrength);
	if(halRet != HAL_OK && errorsEnabled)
		return returnInc;
	else
		returnInc++;

	// Not doable on IQ chips - they seem to be OTP
	if(wq->quadEnable) {
		halRet = W25Q_QuadEnable(wq, 0);
		if(halRet != HAL_OK && errorsEnabled)
			return returnInc;
		else
			returnInc++;
	}


	// Get + Store IDs
	halRet = W25Q_GetIDs(wq, devID);
	if(halRet != HAL_OK && errorsEnabled)
		return returnInc;
	else
		returnInc++;

	// Get + Store Status Register Contents
	halRet = W25Q_ReadStatusRegs(wq);
	if(halRet != HAL_OK && errorsEnabled)
		return returnInc;
	else
		returnInc++;

	// Disable Write Access to Memory
	if(wq->writeEnable != 0) {
		halRet = W25Q_DisableWrite(wq);
		if(halRet != HAL_OK && errorsEnabled)
			return returnInc;
		else
			returnInc++;
	}

	// Get + Store Status Register Contents
	halRet = W25Q_ReadStatusRegs(wq);
	if(halRet != HAL_OK && errorsEnabled)
		return returnInc;
	else
		returnInc++;

	// Set WP pin low (Enable)
	HAL_GPIO_WritePin(wq->nWPPort, wq->nWPPin, GPIO_PIN_RESET);

	return 0;

}

HAL_StatusTypeDef W25Q_ReleasePowerDown(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// TX release power-down instruction
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[1]){CMD_READ_RELEASE_ID}, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);

	// Delay for at least 3 us
	for(uint8_t i = 0; i < 15; i++) {
		__NOP();
	}

	wq->powerUp = 1; 		// We have now successfully exited power-down state

	return halRet;

}

HAL_StatusTypeDef W25Q_GetIDs(W25Q *wq, uint8_t devID_passed) {


	HAL_StatusTypeDef halRet = HAL_OK;

	/*
	 * Release power down + read device ID
	 */
		HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
		// TX return device ID instruction
		halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[4]){CMD_READ_RELEASE_ID, 0x00, 0x00, 0x00}, 4, HAL_MAX_DELAY);

		if(halRet != HAL_OK) {
			HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);
			return halRet;
		}

		// RX device ID into struct variable
		uint8_t devIDRet[1] = {0x00};
		halRet = HAL_SPI_Receive(wq->hspi, devIDRet, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);
		wq->devID = devIDRet[0];


		 // Check to ensure that received device ID is equal to that passed by the user in the init function
		if((halRet != HAL_OK) || (wq->devID != devID_passed))
			return HAL_ERROR;


	/*
	 * Read and check MFR ID
	 */
		// TX read mfr + device ID command
		HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
		halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[4]){CMD_READ_MFR_ID, 0x00, 0x00, 0x00}, 4, HAL_MAX_DELAY);
		if(halRet != HAL_OK) {
			return halRet;
			HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);
		}



		// RX device and mfr ID, store in struct variable
		uint8_t retIDs[2] = {0x00, 0x00};
		halRet = HAL_SPI_Receive(wq->hspi, retIDs, 2, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);
		wq->mfrID = retIDs[0];


		// Check to ensure that received mfr ID is correct.
		if((halRet != HAL_OK) || (wq->mfrID != 0xEF))
			return HAL_ERROR;

	/*
	 * Leaving out for now: Read JEDEC ID
	 */

		return halRet;
}

HAL_StatusTypeDef W25Q_EnableWrite(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// TX write enable command
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[1]){CMD_WRITE_ENABLE}, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);

	if(halRet != HAL_OK)
		return halRet;

	// Read back written value and confirm
	W25Q_ReadStatusReg(wq, 1);

	// Bit 1 of status register 1 should now be 1
	if(wq->writeEnable != 1)
		return HAL_ERROR;

	return halRet;

}

HAL_StatusTypeDef W25Q_DisableWrite(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// TX write disable command
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[1]){CMD_WRITE_DISABLE}, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);

	if(halRet != HAL_OK)
		return halRet;

	// Read back written value and confirm
	W25Q_ReadStatusReg(wq, 1);

	// Bit 1 of status register 1 should still be 0
	if(wq->writeEnable != 0)
		return HAL_ERROR;

	return halRet;

}

HAL_StatusTypeDef W25Q_ReadStatusReg(W25Q *wq, uint8_t regNum) {

	// Check register number is valid
	if(regNum < 1 || regNum > 3) {
		return HAL_ERROR;
	}

	HAL_StatusTypeDef halRet = HAL_OK;

	// Array of commands for status registers 1-3
	uint8_t statRegCMDs[3] = {CMD_READ_STAT_1, CMD_READ_STAT_2, CMD_READ_STAT_3};

	// TX read status register command
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[1]){statRegCMDs[regNum - 1]}, 1, HAL_MAX_DELAY);
	if(halRet != HAL_OK) {
		HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);
		return halRet;
	}

	// RX status register
	uint8_t statRegRet[1] = {0x00};
	halRet = HAL_SPI_Receive(wq->hspi, statRegRet, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);

	switch(regNum) {
	case 1:
		wq->statReg1 = statRegRet[0];
		wq->writeEnable = (wq->statReg1 & 0b00000010) >> 1;
		break;
	case 2:
		wq->statReg2 = statRegRet[0];
		wq->quadEnable = (wq->statReg2 & 0b00000010) >> 1;
		break;
	case 3:
		wq->statReg3 = statRegRet[0];
		wq->driverStrength = (wq->statReg3 & 0b01100000) >> 5;
		break;
	}

	return halRet;

}

HAL_StatusTypeDef W25Q_ReadStatusRegs(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	for(uint8_t i = 1; i <=3; i++) {

		W25Q_ReadStatusReg(wq, i);

	}

	return halRet;

}

HAL_StatusTypeDef W25Q_WriteStatusReg(W25Q *wq, uint8_t regNum, uint8_t regVal) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// Enable Write Access to Memory
	// Write access will disable after status register write
	if(wq->writeEnable != 1) {
		halRet = W25Q_EnableWrite(wq);
		if(halRet != HAL_OK)
			return HAL_ERROR;
	}

	// Array of commands for status registers 1-3
	uint8_t statRegCMDs[3] = {CMD_WRITE_STAT_1, CMD_WRITE_STAT_2, CMD_WRITE_STAT_3};

	// TX write status reg with value
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[2]){statRegCMDs[regNum - 1], regVal}, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);

	// WEL bit will now be set back to 0
	halRet = W25Q_ReadStatusReg(wq, 1);
	if(halRet != HAL_OK)
		return halRet;

	// Delay maximum status register write time (15ms), as given by chip datasheet.
	// ~235ns per i
	for(uint32_t i = 0; i < 70000; i++) {
		__NOP();
	}

	if(halRet != HAL_OK)
		return halRet;

	// Read back written value and confirm
	halRet = W25Q_ReadStatusReg(wq, regNum);
	switch(regNum) {
		case 1:
			if(wq->statReg1 != regVal)
				return HAL_ERROR;
			break;
		case 2:
			if(wq->statReg2 != regVal)
				return HAL_ERROR;
			break;
		case 3:
			if(wq->statReg3 != regVal)
				return HAL_ERROR;
			break;
	}

	return halRet;


}

HAL_StatusTypeDef W25Q_readData(W25Q *wq, uint32_t startAddress, uint32_t dataSize, uint8_t *dataLocation) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// Create read data array with start address
	uint8_t readDataArr[4] = {CMD_READ_DATA, ((startAddress >> 16) & 0xFF), ((startAddress >> 8) & 0xFF), ((startAddress) & 0xFF)};

	// TX read data in single-SPI mode command
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	halRet = HAL_SPI_Transmit(wq->hspi, readDataArr, 4, HAL_MAX_DELAY);
	if(halRet != HAL_OK) {
		HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);
		return halRet;
	}

	// RX data
	halRet = HAL_SPI_Receive(wq->hspi, dataLocation, dataSize, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);


	return halRet;

}

HAL_StatusTypeDef W25Q_readData_DMA(W25Q *wq, uint32_t startAddress, uint32_t dataSize, uint8_t *dataLocation) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// Create read data array with start address
	uint8_t readDataArr[4] = {CMD_READ_DATA, ((startAddress >> 16) & 0xFF), ((startAddress >> 8) & 0xFF), ((startAddress) & 0xFF)};

	// TX read data in single-SPI mode command
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	halRet = HAL_SPI_Transmit(wq->hspi, readDataArr, 4, HAL_MAX_DELAY);
	if(halRet != HAL_OK) {
		HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);
		return halRet;
	}

	/*
	 * Ready to RX data
	 */


	return halRet;

}

HAL_StatusTypeDef W25Q_ChipErase(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// TX chip erase command
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[1]){CMD_CHIP_ERASE}, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);

	// Takes max. 200s

	return halRet;

}

HAL_StatusTypeDef W25Q_ChipPowerDown(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// TX power down command
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[1]){CMD_POWER_DOWN}, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);

	return halRet;
}

HAL_StatusTypeDef W25Q_ChipReset(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	/*
	 * Send both enable reset an reset device commands
	 */
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	// TX enable reset instruction
	halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[1]){CMD_RESET_ENABLE}, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);
	if(halRet != HAL_OK)
		return halRet;

	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_RESET);
	// TX reset device instruction
	halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[1]){CMD_RESET_DEVICE}, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(wq->nCSPort, wq->nCSPin, GPIO_PIN_SET);

	// Delay at least 30 us
	for(uint8_t i = 0; i < 180; i++) {
		__NOP();
	}


	return halRet;

}

HAL_StatusTypeDef W25Q_QuadEnable(W25Q *wq, uint8_t quadBool) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// Get current contents of status register 2
	halRet = W25Q_ReadStatusReg(wq, 2);
	if(halRet != HAL_OK)
		return halRet;

	/*
	 * Change bit 1 and TX to status register 2
	 */
	uint8_t newRegVal = (wq->statReg2) & (0b11111101);
	newRegVal = (newRegVal) | (quadBool << 1);
	halRet = W25Q_WriteStatusReg(wq, 2, newRegVal);
	if(halRet != HAL_OK)
		return halRet;

	// Read back status register 2 and confirm successful write
	halRet = W25Q_ReadStatusReg(wq, 2);
	if(wq->quadEnable != quadBool)
		return HAL_ERROR;

	return halRet;

}

HAL_StatusTypeDef W25Q_SetDriverStrength(W25Q *wq, uint8_t driverStrength) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// Check to ensure passed driver strength value lies in range
	if(driverStrength < 0 || driverStrength > 3)
		return HAL_ERROR;

	// Read current value in status register 3
	halRet = W25Q_ReadStatusReg(wq, 3);

	// Isolate bits 5-6 and change value to reflect new driver strength
	uint8_t newRegVal = (wq->statReg3) & (0b10011111);
	newRegVal = (newRegVal) | (driverStrength << 5);

	// Write status register 3 with new value
	halRet = W25Q_WriteStatusReg(wq, 3, newRegVal);
	if(halRet != HAL_OK)
		return halRet;

	// Read back status register 3 and confirm successful write
	halRet = W25Q_ReadStatusReg(wq, 3);
	if(wq->driverStrength != driverStrength)
		return HAL_ERROR;

	return halRet;

}



