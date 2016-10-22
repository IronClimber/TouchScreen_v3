/*
 * ADC1.c
 *
 *  Created on: 19 זמגע. 2016 נ.
 *      Author: Home
 */

#include "stm32f4xx_hal.h"
#include "adc.h"

uint32_t ADC1_GetValue(uint32_t channel){

	ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
    	Error_Handler();
    }

    // start conversion
    HAL_ADC_Start(&hadc1);
    // wait until finish
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

