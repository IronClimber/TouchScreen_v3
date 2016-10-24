/*
 * TouchScreen.h
 *
 *  Created on: 19 זמגע. 2016 נ.
 *      Author: Home
 */

#ifndef TOUCHSCREEN_H_
#define TOUCHSCREEN_H_

#include "stm32f4xx_hal.h"

#define X_A 13.18
#define X_B 450
#define Y_A 10.3
#define Y_B 340

#define X_BORDER 240
#define Y_BORDER 320

typedef struct {
	GPIO_TypeDef* Port;
	uint16_t Pin;
} GPIOStruct;

typedef struct {
	volatile int32_t x, y;
	float ax, ay;
	float bx, by;
	float dx, dy;
} TouchScreenStruct;

typedef enum {
	OUTPUT_RESET = 0,
	OUTPUT_SET = 1,
	INPUT_PULLUP_EXTI = 2,
	INPUT_NOPULL = 3,
	INPUT_ADC= 4
} GPIOState;

typedef enum {
	TOUCH_OFF = 0,
	TOUCH_ON = 1,
	TOUCH_MEASURE_X = 2,
	TOUCH_MEASURE_Y = 3
} TouchScreenState;

void TouchScreen_Init();
void TouchScreen_Calib();

HAL_StatusTypeDef SetGPIOState(GPIOStruct* str, GPIOState state);
HAL_StatusTypeDef ResetTouchScreenPinsState();
HAL_StatusTypeDef SetPins(TouchScreenState state);

//Old method
/*
int32_t GetTouch_X();
int32_t GetTouch_Y();
*/

void MeasureCalibXY(int32_t* tx, int32_t* ty);

int32_t GetCalib_X();
int32_t GetCalib_Y();

void DrawTarget(int32_t target_x, int32_t target_y, uint16_t color);
void CleanTarget(int32_t target_x, int32_t target_y);

#endif /* TOUCHSCREEN_H_ */
