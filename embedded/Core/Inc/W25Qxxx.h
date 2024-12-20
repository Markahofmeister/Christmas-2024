/*
 * W25Qxxx.h
 *
 *  Created on: Aug 11, 2024
 *      Author: marka
 */

#ifndef INC_W25QXXX_H_
#define INC_W25QXXX_H_

#ifndef  STM32G0XX_HAL_H_
#include "stm32g0xx_hal.h"
#endif

/*
 * Command table
 */
#define CMD_WRITE_ENABLE 		0x06
#define CMD_VOL_SR_WE 			0x50
#define CMD_WRITE_DISABLE		0x04
#define CMD_READ_RELEASE_ID		0xAB
#define CMD_READ_MFR_ID			0x90
#define CMD_READ_JEDEC_ID		0x9F
#define CMD_READ_UID			0x4B
#define CMD_READ_DATA			0x03
#define CMD_FAST_READ			0x0B
#define CMD_PAGE_PROG			0x02
#define CMD_SECT_ERASE_4KB		0x20
#define CMD_BLOCK_ERASE_32KB	0x52
#define CMD_BLOCK_ERASE_64KB	0xD8
#define CMD_CHIP_ERASE			0x60
#define CMD_READ_STAT_1			0x05
#define CMD_READ_STAT_2			0x35
#define CMD_READ_STAT_3			0x15
#define CMD_WRITE_STAT_1		0x01
#define CMD_WRITE_STAT_2		0x31
#define CMD_WRITE_STAT_3		0x11
#define CMD_READ_SFDP			0x5A
#define CMD_ERASE_SEC_REG		0x44
#define CMD_PROG_SEC_REG		0x42
#define CMD_READ_SEC_REG		0x48
#define CMD_GLOB_BLOCK_LOCK		0x7E
#define CMD_GLOB_BLOCK_UNLOCK	0x98
#define CMD_READ_BLOCK_LOCK		0x3D
#define CMD_IND_BLOCK_LOCK		0x36
#define CMD_IND_BLOCK_UNLOCK	0x39
#define CMD_PROG_SUSPEND		0x75
#define CMD_PROG_RESUME			0x7A
#define CMD_POWER_DOWN			0xB9
#define CMD_RESET_ENABLE		0x66
#define CMD_RESET_DEVICE		0x99


/*
 * Struct to serve as object for memory chip
 */
typedef struct {

	/*
	 * GPIO Ports for:
	 * 	- SPI Chip Select Pin
	 * 	- Write Protect Pin
	 * 	- Reset Pin
	 */
	GPIO_TypeDef *nCSPort;
	GPIO_TypeDef *nWPPort;
	GPIO_TypeDef *nHOLDPort;

	/*
	 * GPIO Pin mapping for:
	 * - SPI Chip Select Pin
	 * - Write Protect Pin
	 * - Reset Pin
	 */
	uint32_t nCSPin;
	uint32_t nWPPin;
	uint32_t nHOLDPin;

	// STM32 handle for SPI bus
	SPI_HandleTypeDef *hspi;

	// Device IDs and part numbers
	uint8_t mfrID;
	uint8_t devID;
	uint16_t JEDEC_ID;

	// Status Registers
		/*
		 * Status Register 1
		 * 0b76543210
		 * Bit 7: Status Register Protect (SRP) - Controls Write access to this status register
		 * 										  This bit works in conjunction with bit 0 of Status register 2
		 * 										  Refer to table in section 7.1.7 of datasheet
		 * Bit 6: Sector Protect (SEC) - R/W, Controls if block protect bits protect 4KB sectors (1) or 64KB blocks (0)
		 * Bit 5: Top/Bottom Protect (TB) - R/W, Controls whether block protect bits protect top (1) or bottom (0)
		 * 							   of array
		 * Bits 4-2: Block Protect (BP) - R/W, these 3 bits can be used to protect all/none/portions
		 * 							 of the chip from program/erase instructions. See protection table in datasheet
		 * Bit 1: Write Enable Latch (WEL) - read-only, set to 1 after executing write enable instruction
		 * Bit 0: Write in Progress (BUSY) - read-only, set to 1 when device
		 * 		  is performing actions described in section 7.1.1 of datasheet
		 *
		 */
		uint8_t statReg1;

		/*
		 * Status Register 2
		 * 0b76543210
		 *
		 * Bit 7: Suspend Status (SUS) - read-only, set to 1 after executing an erase/program suspend instruction
		 * 								 cleared to 0 by erase/program resume instruction or power up/down
		 * Bit 6: Complement Protect (CMP) - R/W, Used to complement blocks of protected memory. See table in datasheet
		 * Bits 5-3: Security Register Lock (LB) - R/W, Provide write protect control to security registers.
		 * 										   Each bit set to 1 locks the security register writes. OTP!!
		 * Bit 2: Reserved
		 * Bit 1: Quad Enable (QE) - Enable quad SPI (1).
		 * Bit 0: Status Register Lock (SRL) - R/W, Controls Write access to this status register
		 * 									   This bit works in conjunction with bit 7 of Status register 1
		 * 									   Refer to table in section 7.1.7 of datasheet
		 *
		 */
		uint8_t statReg2;

		/*
		 * Status Register 3
		 * 0b76543210
		 *
		 * Bit 7: Hold/Reset (HOLD/RST) - Software hold/reset fuction bit?
		 * Bits 6-5: Output Driver Strength (DRV1, DRV0) - 00 = 100%, 01 = 75%, 10 = 50%, 11 = 25%
		 * Bit 4: Reserved
		 * Bit 3: Reserved
		 * Bit 2: Write Protect Selection (WPS) - Used to select block protect scheme,
		 * 										  see table in section 7.1.5 of datasheet
		 * Bit 1: Reserved
		 * Bit 0: Reserved
		 *
		 */
		uint8_t statReg3;

	// Other config registers

	uint8_t writeEnable; // 0 = disable, 1 = enable
	uint8_t powerUp; 	 // 0 = powered down, 1 = powered down
	uint8_t quadEnable;  // Should be initialized to 0
	uint8_t driverStrength;

	/*
	 * Not implemented:
	 * 		- SFDP Reg
	 * 		- Security Reg
	 * 		- Block Protect/Erase
	 * 		- Dual/Quad SPI stuff
	 */


} W25Q;

/*
 * Initializes SPI memory chip object
 *
 * devID = device ID given by Winbond datasheet. This depends on the type/capacity of memory.
 * isQuadChip = (1) if the chip PN ends in "IQ/IN" or "JQ", quad mode is enabled
 * 					Seems unchangeable for these PNs of chips
 * 				(0) if the chip PN ends in "IM" or "JM" quad mode default off
 *
 */
uint8_t W25Q_Init(W25Q *wq, GPIO_TypeDef *nCSPort, GPIO_TypeDef *nWPPort, GPIO_TypeDef *nHOLDPort,
					uint32_t nCSPin, uint32_t nWPPin, uint32_t nHOLDPin, SPI_HandleTypeDef *hspi,
					uint8_t devID, uint8_t isQuadChip, uint8_t driveStrength);

/*
 * Releases chip from power down mode
 */
HAL_StatusTypeDef W25Q_ReleasePowerDown(W25Q *wq);

/*
 * Will release device from power-down and fetch all of the IDs
 */
HAL_StatusTypeDef W25Q_GetIDs(W25Q *wq, uint8_t devID);

/*
 * Enables writing to memory
 */
HAL_StatusTypeDef W25Q_EnableWrite(W25Q *wq);
/*
 * Disables writing to memory
 */
HAL_StatusTypeDef W25Q_DisableWrite(W25Q *wq);

/*
 * Get/Set methods for write status register
 */
HAL_StatusTypeDef W25Q_ReadStatusReg(W25Q *wq, uint8_t regNum);		// Read single status register
HAL_StatusTypeDef W25Q_ReadStatusRegs(W25Q *wq);					// Read all status registers
HAL_StatusTypeDef W25Q_WriteStatusReg(W25Q *wq, uint8_t regNum, uint8_t regVal);

/*
 * Read data from memory
 * startAddress = address to begin reading memory from, 24 bits
 * dataSize = number of bits to be read
 * dataLocation = pointer to array where data is to be placed
 */
HAL_StatusTypeDef W25Q_readData(W25Q *wq, uint32_t startAddress, uint32_t dataSize, uint8_t *dataLocation);

/*
 * Read data from memory over DMA
 * ONLY INITIALIZES DMA RX, does not actually begin transaction.
 * startAddress = address to begin reading memory from, 24 bits
 * dataSize = number of bits to be read
 * dataLocation = pointer to array where data is to be placed
 */
HAL_StatusTypeDef W25Q_readData(W25Q *wq, uint32_t startAddress, uint32_t dataSize, uint8_t *dataLocation);

/*
 * Erases Entire memory
 */
HAL_StatusTypeDef W25Q_ChipErase(W25Q *wq);

/*
 * Powers down the chip to a low-power mode
 */
HAL_StatusTypeDef W25Q_ChipPowerDown(W25Q *wq);

/*
 * Resets the chip
 */
HAL_StatusTypeDef W25Q_ChipReset(W25Q *wq);

/*
 * Enables (1) or disables (0) quad SPI mode
 */
HAL_StatusTypeDef W25Q_QuadEnable(W25Q *wq, uint8_t quadBool);

/*
 * Sets the river strength of the W25Q output drivers
 * Options:
 * 		0 = 100% strength
 * 		1 = 75% strength
 * 		2 = 50% strength
 * 		3 = 25% strength (default)
 */
HAL_StatusTypeDef W25Q_SetDriverStrength(W25Q *wq, uint8_t driverStrength);


#endif /* INC_W25QXXX_H_ */
