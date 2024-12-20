/*
 * NAU8315YG.c
 *
 *  Created on: Aug 15, 2024
 *      Author: marka
 */


#ifndef NAU8315YG_H_
#include "../Inc/NAU8315YG.h"
#endif

void NAU8315YG_Init(NAU8315YG *nau, I2S_HandleTypeDef *hi2s, GPIO_TypeDef *enablePort, uint32_t enablePin) {

	nau->hi2s = hi2s;

	nau->enablePort = enablePort;
	nau->enablePin = enablePin;

	NAU8315YG_AmpDisable(nau);

}

void NAU8315YG_AmpEnable(NAU8315YG *nau) {

	// Set enable pin high
	HAL_GPIO_WritePin(nau->enablePort, nau->enablePin, GPIO_PIN_SET);

	nau->ampEnableFlag = 1;

}


void NAU8315YG_AmpDisable(NAU8315YG *nau) {

	// Set enable pin high
	HAL_GPIO_WritePin(nau->enablePort, nau->enablePin, GPIO_PIN_RESET);

	nau->ampEnableFlag = 0;

}
