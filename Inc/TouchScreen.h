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

#define CALIB_BORDER_SPACING 20
#define CALIB_POINTS_QUANTITY 5

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

void TouchScreen_ReadXY(int32_t* tx, int32_t* ty);
int32_t TouchScreen_GetX();
int32_t TouchScreen_GetY();

HAL_StatusTypeDef TouchScreen_SetGPIOState(GPIOStruct* str, GPIOState state);
HAL_StatusTypeDef TouchScreen_ResetPinsState();
HAL_StatusTypeDef TouchScreen_SetPins(TouchScreenState state);

void TouchScreen_Calib_3Points();
void TouchScreen_Calib_5Points();

static uint32_t Sum(uint32_t* matrix);
static uint32_t MultiplyAndSum(uint32_t*m1, uint32_t* m2);

static void TouchScreen_CalculateCoefficients(int64_t delta,
		int64_t delta_x1, int64_t delta_x2, int64_t delta_x3,
		int64_t delta_y1, int64_t delta_y2, int64_t delta_y3);

static void TouchScreen_ReadTarget(int32_t x, int32_t y, uint32_t* tx, uint32_t* ty);
static void TouchScreen_DrawTarget(int32_t target_x, int32_t target_y, uint16_t color);
static void TouchScreen_CleanTarget(int32_t target_x, int32_t target_y);

#endif /* TOUCHSCREEN_H_ */
