/*
 * NAU8315YG.h
 *
 *  Created on: Aug 15, 2024
 *      Author: marka
 */

#ifndef INC_NAU8315YG_H_
#define INC_NAU8315YG_H_

#ifndef STM32G0XX_HAL_H_
#include "stm32g0xx_hal.h"
#endif

typedef struct {

	I2S_HandleTypeDef *hi2s;

	GPIO_TypeDef *enablePort;

	uint32_t enablePin;

	uint8_t ampEnableFlag;		// 1 = enabled, 0 = disabled

} NAU8315YG;

/*
 * Initialize Amplifier
 */
void NAU8315YG_Init(NAU8315YG *nau, I2S_HandleTypeDef *hi2s, GPIO_TypeDef *enablePort, uint32_t enablePin);

/*
 * Enables amplifier
 */
void NAU8315YG_AmpEnable(NAU8315YG *nau);

/*
 * Disables amplifier
 */
void NAU8315YG_AmpDisable(NAU8315YG *nau);

#endif /* INC_NAU8315YG_H_ */
