/*
 * TouchScreen.c
 *
 *  Created on: 19 жовт. 2016 р.
 *      Author: Home
 */
#include "stm32f4xx_hal.h"
#include "TouchScreen.h"
#include "ADC1.h"
#include "display\lcd.h"

GPIOStruct x_left, x_right, y_up, y_down;

volatile static int32_t x = 0;
volatile static int32_t y = 0;

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

X- -> PA1
X+ -> PA8
Y- -> PB10
Y+ -> PA4

*/

void TouchScreen_Init() {
	// X-
	x_left.Port = GPIOA;
	x_left.Pin = GPIO_PIN_1;

	// X+
	x_right.Port = GPIOA;
	x_right.Pin = GPIO_PIN_8;

	// Y-
	y_up.Port = GPIOB;
	y_up.Pin = GPIO_PIN_10;

	// Y+
	y_down.Port = GPIOA;
	y_down.Pin = GPIO_PIN_4;

	ts_h.ax = -0.076;
	ts_h.bx = 0;
	ts_h.dx = 270;

	ts_h.ay = 0;
	ts_h.by = 0.097;
	ts_h.dy = -33;
}

void TouchScreen_Calib() {

	int32_t x1 = 10;
	int32_t y1 = 100;
	int32_t x2 = 77;
	int32_t y2 = 300;
	int32_t x3 = 200;
	int32_t y3 = 15;

	SetPins(TOUCH_OFF);
	LCD_SetCursor(20,150);
	LCD_Printf("Starting calibration process...");

	HAL_Delay(1500);

	LCD_SetCursor(0,0);
	LCD_FillScreen(BLACK);
	LCD_Printf("SET FIRST POINT   ");
	DrawTarget(x1,y1,WHITE);
	SetPins(TOUCH_DETECT);
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != 0) HAL_Delay(50); //?? общая функция
	SetPins(TOUCH_MEASURE_X);
	uint32_t tx1 = ADC1_GetValue(ADC_CHANNEL_4);
	SetPins(TOUCH_MEASURE_Y);
	uint32_t ty1 = ADC1_GetValue(ADC_CHANNEL_1);
	SetPins(TOUCH_OFF);
	DrawTarget(x1,y1,RED);
	HAL_Delay(500);
	CleanTarget(x1,y1);


	HAL_Delay(1000);

	LCD_SetCursor(0,0);
	LCD_Printf("SET SECOND POINT   ");
	DrawTarget(x2,y2,WHITE);
	SetPins(TOUCH_DETECT);
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != 0) HAL_Delay(50); //?? общая функция
	SetPins(TOUCH_MEASURE_X);
	uint32_t tx2 = ADC1_GetValue(ADC_CHANNEL_4);
	SetPins(TOUCH_MEASURE_Y);
	uint32_t ty2 = ADC1_GetValue(ADC_CHANNEL_1);
	SetPins(TOUCH_OFF);
	DrawTarget(x2,y2,RED);
	HAL_Delay(500);
	CleanTarget(x2,y2);

	HAL_Delay(1000);

	LCD_SetCursor(0,0);
	LCD_Printf("SET THIRD POINT   ");
	DrawTarget(x3,y3,WHITE);
	SetPins(TOUCH_DETECT);
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != 0) HAL_Delay(50); //?? общая функция
	SetPins(TOUCH_MEASURE_X);
	uint32_t tx3 = ADC1_GetValue(ADC_CHANNEL_4);
	SetPins(TOUCH_MEASURE_Y);
	uint32_t ty3 = ADC1_GetValue(ADC_CHANNEL_1);
	SetPins(TOUCH_OFF);
	DrawTarget(x3,y3,RED);
	HAL_Delay(500);
	CleanTarget(x3,y3);

	HAL_Delay(1000);
	LCD_FillScreen(BLACK);
	LCD_SetCursor(100,150);
	LCD_Printf("Waiting...");

	//LCD_SetCursor(0,50);
	//LCD_Printf("%d %d %d %d\n%d %d %d %d\n%d %d %d %d\n",x1,y1,tx1,tx2,x2,y2,tx2,ty2,x3,y3,tx3,ty3);

	// define parameters
	int32_t det =    (tx1-tx3)*(ty2-ty3)-(tx2-tx3)*(ty1-ty3);
	int32_t det_x1 = (x1-x3)*(ty2-ty3)-(x2-x3)*(ty1-ty3);
	int32_t det_x2 = (tx1-tx3)*(x2-x3)-(tx2-tx3)*(x1-x3);
	int32_t det_x3 = x1*(tx2*ty3-tx3*ty2)-x2*(tx1*ty3-tx3*ty1)+x3*(tx1*ty2-tx2*ty1);
	int32_t det_y1 = (y1-y3)*(ty2-ty3)-(y2-y3)*(ty1-ty3);
	int32_t det_y2 = (tx1-tx3)*(y2-y3)-(tx2-tx3)*(y1-y3);
	int32_t det_y3 = y1*(tx2*ty3-tx3*ty2)-y2*(tx1*ty3-tx3*ty1)+y3*(tx1*ty2-tx2*ty1);

	//LCD_Printf("%d\n%d %d %d\n%d %d %d\n", det,det_x1,det_x2,det_x3,det_y1,det_y2,det_y3);

	ts_h.ax = (float)det_x1/det;
	ts_h.bx = (float)det_x2/det;
	ts_h.dx = (float)det_x3/det;

	ts_h.ay = (float)det_y1/det;
	ts_h.by = (float)det_y2/det;
	ts_h.dy = (float)det_y3/det;

	//LCD_SetCursor(0,200);
	//LCD_Printf("ax = %f\nbx = %f\ndx = %f\nay = %f\nby = %f\ndy = %f", ts_h.ax, ts_h.bx, ts_h.dx, ts_h.ay, ts_h.by, ts_h.dy);

	HAL_Delay(1500);
	LCD_FillScreen(BLACK);
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
	HAL_Delay(1);
	switch (state) {
		case TOUCH_OFF:
			GPIO_Init(); //Refresh LCD GPIOs
			return HAL_OK;
		case TOUCH_DETECT:
			SetGPIOState(&y_down, INPUT_PULLUP_EXTI);
			SetGPIOState(&y_up, INPUT_NOPULL);
			SetGPIOState(&x_right, OUTPUT_RESET);
			SetGPIOState(&x_left, INPUT_NOPULL);
			HAL_Delay(1);
			return HAL_OK;
		case TOUCH_MEASURE_X:
			SetGPIOState(&x_left, OUTPUT_RESET);
			SetGPIOState(&x_right, OUTPUT_SET);
			SetGPIOState(&y_up, INPUT_NOPULL);
			SetGPIOState(&y_down, INPUT_ADC);
			HAL_Delay(1);
			return HAL_OK;
		case TOUCH_MEASURE_Y:
			SetGPIOState(&x_left, INPUT_ADC);
			SetGPIOState(&x_right, INPUT_NOPULL);
			SetGPIOState(&y_up, OUTPUT_SET);
			SetGPIOState(&y_down, OUTPUT_RESET);
			HAL_Delay(1);
			return HAL_OK;
		default:
			return HAL_ERROR;
	}
}

//Old method
/*int32_t GetTouch_X() {
	int32_t x_val = (X_BORDER-(ADC1_GetValue(ADC_CHANNEL_4)-X_B)/(X_A));
	return (int32_t) x_val;
}

int32_t GetTouch_Y() {
	int32_t y_val = ((ADC1_GetValue(ADC_CHANNEL_1)-Y_B)/(Y_A));
	return (int32_t) y_val;
}*/

void MeasureCalibXY() {

	  SetPins(TOUCH_MEASURE_X);
	  uint32_t x_t = ADC1_GetValue(ADC_CHANNEL_4);
	  SetPins(TOUCH_MEASURE_Y);
	  uint32_t y_t = ADC1_GetValue(ADC_CHANNEL_1);
	  SetPins(TOUCH_OFF);

	  x = ts_h.ax*x_t + ts_h.bx*y_t + ts_h.dx;
	  y = ts_h.ay*x_t + ts_h.by*y_t + ts_h.dy;

}

int32_t GetCalib_X() {
	return x;
}

int32_t GetCalib_Y() {
	return y;
}

void DrawTarget(int32_t target_x, int32_t target_y, uint16_t color) {
	LCD_DrawFastHLine(target_x-4, target_y, 9, color);
	LCD_DrawFastVLine(target_x, target_y-4, 9, color);
}

void CleanTarget(int32_t target_x, int32_t target_y) {
	LCD_DrawFastHLine(target_x-4, target_y, 9, BLACK);
	LCD_DrawFastVLine(target_x, target_y-4, 9, BLACK);
}

