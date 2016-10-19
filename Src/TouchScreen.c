/*
 * TouchScreen.c
 *
 *  Created on: 19 ����. 2016 �.
 *      Author: Home
 */
#include "stm32f4xx_hal.h"
#include "TouchScreen.h"
#include "ADC1.h"

GPIOStruct x_left, x_right, y_up, y_down;

/*static x = 0;
static y = 0;*/

TouchScreenStruct ts_h;
/*Set TouchScreen Pins
	  	  	(Y-)
	  	  	 |
	  ----------------
	  |              |
	  |              |
	  |              |
	  |              |
(X-)--|              |--(X+)
	  |              |
	  |              |
	  |              |
	  |              |
	  ----------------
	         |
	        (Y+)
*/

void TouchScreen_Init() {
	// X-
	x_left.Port = GPIOA;
	x_left.Pin = GPIO_PIN_4;

	// X+
	x_right.Port = GPIOB;
	x_right.Pin = GPIO_PIN_10;

	// Y-
	y_up.Port = GPIOA;
	y_up.Pin = GPIO_PIN_8;

	// Y+
	y_down.Port = GPIOA;
	y_down.Pin = GPIO_PIN_1;



}

void TouchScreen_Calib(uint32_t x_border, uint32_t y_border) {
	uint32_t x1 = 10;
	uint32_t y1 = 100;
	uint32_t x2 = 37;
	uint32_t y2 = 200;
	uint32_t x3 = 200;
	uint32_t y3 = 15;

	SetPins(TOUCH_OFF);
	LCD_SetCursor(0,0);
	LCD_Printf("Set first point");
	LCD_DrawPixel(x1,y1);
	SetPins(TOUCH_DETECT);
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != 0) HAL_Delay(50); //?? ����� �������
	SetPins(TOUCH_MEASURE_X);
	uint32_t tx1 = ADC1_GetValue(ADC_CHANNEL_4);
	SetPins(TOUCH_MEASURE_Y);
	uint32_t ty1 = ADC1_GetValue(ADC_CHANNEL_1);
	SetPins(TOUCH_OFF);
	LCD_SetCursor(0,0);
	LCD_Printf("Set second point");
	LCD_DrawPixel(x2,y2);
	SetPins(TOUCH_DETECT);
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != 0) HAL_Delay(50); //?? ����� �������
	SetPins(TOUCH_MEASURE_X);
	uint32_t tx2 = ADC1_GetValue(ADC_CHANNEL_4);
	SetPins(TOUCH_MEASURE_Y);
	uint32_t ty2 = ADC1_GetValue(ADC_CHANNEL_1);
	SetPins(TOUCH_OFF);
	LCD_SetCursor(0,0);
	LCD_Printf("Set third point");
	LCD_DrawPixel(x3,y3);
	SetPins(TOUCH_DETECT);
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != 0) HAL_Delay(50); //?? ����� �������
	SetPins(TOUCH_MEASURE_X);
	uint32_t tx3 = ADC1_GetValue(ADC_CHANNEL_4);
	SetPins(TOUCH_MEASURE_Y);
	uint32_t ty3 = ADC1_GetValue(ADC_CHANNEL_1);

	// define parameters
	int32_t det = (tx1-tx3)*(ty2-ty3)-(tx2-tx3)*(ty1-ty3);
	int32_t det_x1 = (x1-x3)*(ty2-ty3)-(x2-x3)*(ty1-ty3);
	int32_t det_x2 = (tx1-tx3)*(x2-x3)-(tx2-tx3)*(x1-x3);
	int32_t det_x3 = x1*(tx2*ty3-tx3*ty2)-x2*(tx1*ty3-tx3*ty1)+x2*(tx1*ty2-tx2*ty1);
	int32_t det_y1 = (y1-y3)*(ty2-ty3)-(y2-y3)*(ty1-ty3);
	int32_t det_y2 = (tx1-tx2)*(y2-y3)-(tx2-tx3)*(y1-y3);
	int32_t det_y3 = y1*(tx2*ty3-tx3*ty2)-y2*(tx1*ty3-tx3-ty1)+ y3*(tx1*ty2-tx2*ty1);

	ts_h.ax = det_x1/det;
	ts_h.bx = det_x2/det;
	ts_h.dx = det_x3/det;

	ts_h.ay = det_y1/det;
	ts_h.by = det_y2/det;
	ts_h.dy = det_y3/det;
}

HAL_StatusTypeDef SetGPIOState(GPIOStruct* str, GPIOState state) {
	HAL_GPIO_DeInit(str->Port, str->Pin);
	GPIO_InitTypeDef GPIO_InitStruct;

	switch (state) {
	case OUTPUT_RESET:
		GPIO_InitStruct.Pin = str->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;                         //??
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;				//??
		HAL_GPIO_Init(str->Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(str->Port, str->Pin, GPIO_PIN_RESET);
		return HAL_OK;
	case OUTPUT_SET:
		GPIO_InitStruct.Pin = str->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;                         //??
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;				//??
		HAL_GPIO_Init(str->Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(str->Port, str->Pin, GPIO_PIN_SET);
		return HAL_OK;
	case INPUT_PULLUP_EXTI:
		GPIO_InitStruct.Pin = str->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(str->Port, &GPIO_InitStruct);
		HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0); //EXTI4 because we use PA4 (X-) to detect touch
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);			//If you use other pin, you need change EXTI
		return HAL_OK;
	case INPUT_NOPULL:
		GPIO_InitStruct.Pin = str->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(str->Port, &GPIO_InitStruct);
		return HAL_OK;
	case INPUT_ADC:
	    GPIO_InitStruct.Pin = str->Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(str->Port, &GPIO_InitStruct);
	    return HAL_OK;
	default:
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef ResetTouchScreenPinsState() {
	HAL_GPIO_DeInit(x_left.Port, x_left.Pin);
	HAL_GPIO_DeInit(x_right.Port, x_right.Pin);
	HAL_GPIO_DeInit(y_up.Port, y_up.Pin);
	HAL_GPIO_DeInit(y_down.Port, y_down.Pin);
	return HAL_OK;
}

HAL_StatusTypeDef SetPins(TouchScreenState state) {
	switch (state) {
		case TOUCH_OFF:
			GPIO_Init();
			return HAL_OK;
		case TOUCH_DETECT:
			SetGPIOState(&x_left, INPUT_PULLUP_EXTI);
			SetGPIOState(&x_right, INPUT_NOPULL);
			SetGPIOState(&y_up, OUTPUT_RESET);

			SetGPIOState(&y_down, INPUT_NOPULL);
			return HAL_OK;
		case TOUCH_MEASURE_X:
			SetGPIOState(&x_left, INPUT_ADC);
			SetGPIOState(&x_right, INPUT_NOPULL);
			SetGPIOState(&y_up, OUTPUT_SET);
			SetGPIOState(&y_down, OUTPUT_RESET);
			return HAL_OK;
		case TOUCH_MEASURE_Y:
			SetGPIOState(&x_left, OUTPUT_RESET);
			SetGPIOState(&x_right, OUTPUT_SET);
			SetGPIOState(&y_up, INPUT_NOPULL);
			SetGPIOState(&y_down, INPUT_ADC);
			return HAL_OK;
		default:
			return HAL_ERROR;
	}
}

uint32_t GetTouch_X() {
	int32_t x_val = (X_BORDER-(ADC1_GetValue(ADC_CHANNEL_4)-X_B)/(X_A));
	if (x_val>X_BORDER) return (uint32_t) X_BORDER;
	else if (x_val<0) return 0;
	else return (uint32_t) x_val;
}

uint32_t GetTouch_Y() {
	int32_t y_val = ((ADC1_GetValue(ADC_CHANNEL_1)-Y_B)/(Y_A));
	if (y_val>Y_BORDER) return (uint32_t) Y_BORDER;
	else if (y_val<0) return 0;
	else return (uint32_t) y_val;
}

/*uint32_t GetTouch_X_calib() {
	int32_t x_val = ts_h.x_border-(ADC1_GetValue(ADC_CHANNEL_4)*);
	if (x_val>ts_h.x_border) return (uint32_t) ts_h.x_border;
	else if (x_val<0) return 0;
	else return (uint32_t) x_val;
}
uint32_t MeasureTouch_X() {

}*/