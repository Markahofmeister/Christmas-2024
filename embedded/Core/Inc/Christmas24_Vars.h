/*
 * vars.h
 *
 *  Created on: Jul 5, 2023
 *      Author: marka
 *
 *  No functions - just variable declarations specifically tailored to sinking clock.
 *  Written for STM32G0xx.
 */

#ifndef INC_CHRISTMAS24_VARS_H_
#define INC_CHRISTMAS24_VARS_H_

#ifndef STM32G0XX_HAL_H
#include "stm32g0xx_hal.h"
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

#ifndef STDBOOL_H
#include <stdbool.h>
#endif

#ifndef STRING_H
#include <string.h>
#endif

/*
 * SPI Flash memory settings
 */
GPIO_TypeDef *nCSPort = GPIOA;
GPIO_TypeDef *nWPPort = GPIOA;
GPIO_TypeDef *nHOLDPort = GPIOA;

uint32_t nCSPin = GPIO_PIN_5;
uint32_t nWPPin = GPIO_PIN_6;
uint32_t nHOLDPin = GPIO_PIN_7;

uint8_t spiFlash_devID = 0x17;
uint8_t spiFlash_isQuadChip = 0;
uint8_t spiFlash_driveStrength = 1;

/*
 * I2S Amplifier Settings
 */
GPIO_TypeDef *i2sAmp_enablePort = GPIOD;
uint32_t i2sAmp_enablePin = GPIO_PIN_0;

/*
 * Audio/DMA things
 */

#define BUFFER_SIZE 64

// Offset of flash memory (first byte to read)
#define initialMemoryOffset 0

// End of audio data in flash memory
#define audioAddr_END 0x890E0E




#endif /* INC_CHRISTMAS24_VARS_H_ */
