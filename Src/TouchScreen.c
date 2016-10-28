/*
 * TouchScreen.c
 *
 *  Created on: 19 זמגע. 2016 נ.
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

	//default coefficients
	ts_h.ax = -0.076;
	ts_h.bx = 0;
	ts_h.dx = 270;

	ts_h.ay = 0;
	ts_h.by = 0.097;
	ts_h.dy = -33;
}

void TouchScreen_Calib_3Points() {

    int32_t x1 = 10;
	int32_t y1 = 100;

	int32_t x2 = 77;
	int32_t y2 = 300;

	int32_t x3 = 200;
	int32_t y3 = 15;

	uint32_t tx1, tx2, tx3, ty1, ty2, ty3;

	TouchScreen_SetPins(TOUCH_OFF);

	LCD_SetCursor(20,150);
	LCD_Printf("Starting calibration process...");

	HAL_Delay(1500);

	LCD_SetCursor(0,0);
	LCD_FillScreen(BLACK);
	LCD_Printf("SET FIRST POINT   ");

	TouchScreen_ReadTarget(x1, y1, &tx1, &ty1);

	HAL_Delay(1000);

	LCD_SetCursor(0,0);
	LCD_Printf("SET SECOND POINT   ");

	TouchScreen_ReadTarget(x2, y2, &tx2, &ty2);

	HAL_Delay(1000);

	LCD_SetCursor(0,0);
	LCD_Printf("SET THIRD POINT   ");

	TouchScreen_ReadTarget(x3, y3, &tx3, &ty3);

	HAL_Delay(1000);

	LCD_FillScreen(BLACK);
	LCD_SetCursor(100,150);
	LCD_Printf("Waiting...");

	//LCD_SetCursor(0,50);
	//LCD_Printf("%d %d %d %d\n%d %d %d %d\n%d %d %d %d\n",x1,y1,tx1,tx2,x2,y2,tx2,ty2,x3,y3,tx3,ty3);

	// define parameters
	int32_t det    = (tx1-tx3)*(ty2-ty3)-(tx2-tx3)*(ty1-ty3);
	int32_t det_x1 = (x1-x3)*(ty2-ty3)-(x2-x3)*(ty1-ty3);
	int32_t det_x2 = (tx1-tx3)*(x2-x3)-(tx2-tx3)*(x1-x3);
	int32_t det_x3 = x1*(tx2*ty3-tx3*ty2)-x2*(tx1*ty3-tx3*ty1)+x3*(tx1*ty2-tx2*ty1);
	int32_t det_y1 = (y1-y3)*(ty2-ty3)-(y2-y3)*(ty1-ty3);
	int32_t det_y2 = (tx1-tx3)*(y2-y3)-(tx2-tx3)*(y1-y3);
	int32_t det_y3 = y1*(tx2*ty3-tx3*ty2)-y2*(tx1*ty3-tx3*ty1)+y3*(tx1*ty2-tx2*ty1);

	//LCD_Printf("%d\n%d %d %d\n%d %d %d\n", det,det_x1,det_x2,det_x3,det_y1,det_y2,det_y3);

	TouchScreen_CalculateCoefficients(det, det_x1, det_x2, det_x3, det_y1, det_y2, det_y3);

	//LCD_SetCursor(0,200);
	//LCD_Printf("ax = %f\nbx = %f\ndx = %f\nay = %f\nby = %f\ndy = %f", ts_h.ax, ts_h.bx, ts_h.dx, ts_h.ay, ts_h.by, ts_h.dy);

	HAL_Delay(1500);
	LCD_FillScreen(BLACK);
}

void TouchScreen_Calib_5Points() {

	//XY test points
	uint32_t kx[CALIB_POINTS_QUANTITY] = {CALIB_BORDER_SPACING, X_BORDER-CALIB_BORDER_SPACING, X_BORDER/2, CALIB_BORDER_SPACING, X_BORDER-CALIB_BORDER_SPACING};
	uint32_t ky[CALIB_POINTS_QUANTITY] = {CALIB_BORDER_SPACING, CALIB_BORDER_SPACING, Y_BORDER/2, Y_BORDER-CALIB_BORDER_SPACING, Y_BORDER-CALIB_BORDER_SPACING};

	//XY ADC result array
	uint32_t kx_t[CALIB_POINTS_QUANTITY];
	uint32_t ky_t[CALIB_POINTS_QUANTITY];

	TouchScreen_SetPins(TOUCH_OFF);

	LCD_SetCursor(20,150);
	LCD_Printf("Starting calibration process...");

	HAL_Delay(1500);

	uint8_t i;

	for (i=0; i<CALIB_POINTS_QUANTITY; i++) {

		LCD_SetCursor(0,0);
		LCD_FillScreen(BLACK);
		LCD_Printf("SET POINT %d ", i+1);

		TouchScreen_ReadTarget(kx[i], ky[i], &kx_t[i], &ky_t[i]);

		HAL_Delay(1000);
	}

	LCD_FillScreen(BLACK);
	LCD_SetCursor(100,150);
	LCD_Printf("Waiting...");

	uint8_t n = CALIB_POINTS_QUANTITY;

	uint64_t a = MultiplyAndSum(kx_t, kx_t);
	uint64_t b = MultiplyAndSum(ky_t, ky_t);
	uint64_t c = MultiplyAndSum(kx_t, ky_t);
	uint64_t d = Sum(kx_t);
	uint64_t e = Sum(ky_t);

	//LCD_Printf("\n\n%d %d %d %d %d", a,b,c,d,e);

	uint64_t X1 = MultiplyAndSum(kx_t, kx);
	uint64_t X2 = MultiplyAndSum(ky_t, kx);
	uint64_t X3 = Sum(kx);
	uint64_t Y1 = MultiplyAndSum(kx_t, ky);
	uint64_t Y2 = MultiplyAndSum(ky_t, ky);
	uint64_t Y3 = Sum(ky);

	// define parameters
	int64_t det    = n*(a*b-c*c)+2*c*d*e-a*e*e-b*d*d;
	int64_t det_x1 = n*(X1*b-X2*c)+e*(X2*d-X1*e)+X3*(c*e-b*d);
	int64_t det_x2 = n*(X2*a-X1*c)+d*(X1*e-X2*d)+X3*(c*d-a*e);
	int64_t det_x3 = X3*(a*b-c*c)+X1*(c*e-b*d)+X2*(c*d-a*e);
	int64_t det_y1 = n*(Y1*b-Y2*c)+e*(Y2*d-Y1*e)+Y3*(c*e-b*d);
	int64_t det_y2 = n*(Y2*a-Y1*c)+d*(Y1*e-Y2*d)+Y3*(c*d-a*e);
	int64_t det_y3 = Y3*(a*b-c*c)+Y1*(c*e-b*d)+Y2*(c*d-a*e);

	TouchScreen_CalculateCoefficients(det, det_x1, det_x2, det_x3, det_y1, det_y2, det_y3);

	HAL_Delay(1500);
	LCD_FillScreen(BLACK);
}

HAL_StatusTypeDef TouchScreen_SetGPIOState(GPIOStruct* str, GPIOState state) {

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

HAL_StatusTypeDef TouchScreen_ResetPinsState() {
	HAL_GPIO_DeInit(x_left.Port, x_left.Pin);
	HAL_GPIO_DeInit(x_right.Port, x_right.Pin);
	HAL_GPIO_DeInit(y_up.Port, y_up.Pin);
	HAL_GPIO_DeInit(y_down.Port, y_down.Pin);
	return HAL_OK;
}

HAL_StatusTypeDef TouchScreen_SetPins(TouchScreenState state) {
	switch (state) {
		case TOUCH_OFF:
			GPIO_Init(); //Refresh LCD GPIOs. This function from LCD driver.
			HAL_Delay(1);
			return HAL_OK;
		case TOUCH_ON:
			TouchScreen_SetGPIOState(&y_down, INPUT_PULLUP_EXTI);
			TouchScreen_SetGPIOState(&y_up, INPUT_NOPULL);
			TouchScreen_SetGPIOState(&x_right, OUTPUT_RESET);
			TouchScreen_SetGPIOState(&x_left, INPUT_NOPULL);
			HAL_Delay(1);
			return HAL_OK;
		case TOUCH_MEASURE_X:
			TouchScreen_SetGPIOState(&x_left, OUTPUT_RESET);
			TouchScreen_SetGPIOState(&x_right, OUTPUT_SET);
			TouchScreen_SetGPIOState(&y_up, INPUT_NOPULL);
			TouchScreen_SetGPIOState(&y_down, INPUT_ADC);
			HAL_Delay(1);
			return HAL_OK;
		case TOUCH_MEASURE_Y:
			TouchScreen_SetGPIOState(&x_left, INPUT_ADC);
			TouchScreen_SetGPIOState(&x_right, INPUT_NOPULL);
			TouchScreen_SetGPIOState(&y_up, OUTPUT_SET);
			TouchScreen_SetGPIOState(&y_down, OUTPUT_RESET);
			HAL_Delay(1);
			return HAL_OK;
		default:
			return HAL_ERROR;
	}
}

void TouchScreen_ReadXY(int32_t* tx, int32_t* ty) {

	  TouchScreen_SetPins(TOUCH_MEASURE_X);
	  uint32_t x_t = ADC1_GetValue(ADC_CHANNEL_4);
	  TouchScreen_SetPins(TOUCH_MEASURE_Y);
	  uint32_t y_t = ADC1_GetValue(ADC_CHANNEL_1);

	  x = ts_h.ax*x_t + ts_h.bx*y_t + ts_h.dx;
	  y = ts_h.ay*x_t + ts_h.by*y_t + ts_h.dy;

	  *tx = x;
	  *ty = y;
}

int32_t TouchScreen_GetX() {
	return x;
}

int32_t TouchScreen_GetY() {
	return y;
}

static void TouchScreen_ReadTarget(int32_t x, int32_t y, uint32_t* tx, uint32_t* ty) {
	TouchScreen_DrawTarget(x,y,WHITE);
	TouchScreen_SetPins(TOUCH_ON);
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != 0) HAL_Delay(50); // waiting for touch
	TouchScreen_SetPins(TOUCH_MEASURE_X);
	*tx = ADC1_GetValue(ADC_CHANNEL_4);
	TouchScreen_SetPins(TOUCH_MEASURE_Y);
	*ty = ADC1_GetValue(ADC_CHANNEL_1);
	TouchScreen_SetPins(TOUCH_OFF);
	TouchScreen_DrawTarget(x,y,RED);
	HAL_Delay(500);
	TouchScreen_CleanTarget(x,y);
}


static void TouchScreen_CalculateCoefficients(int64_t delta, int64_t delta_x1, int64_t delta_x2,
		int64_t delta_x3, int64_t delta_y1, int64_t delta_y2, int64_t delta_y3) {

	ts_h.ax = (float)delta_x1/delta;
	ts_h.bx = (float)delta_x2/delta;
	ts_h.dx = (float)delta_x3/delta;

	ts_h.ay = (float)delta_y1/delta;
	ts_h.by = (float)delta_y2/delta;
	ts_h.dy = (float)delta_y3/delta;

}

static void TouchScreen_DrawTarget(int32_t target_x, int32_t target_y, uint16_t color) {
	LCD_DrawFastHLine(target_x-4, target_y, 9, color);
	LCD_DrawFastVLine(target_x, target_y-4, 9, color);
}

static void TouchScreen_CleanTarget(int32_t target_x, int32_t target_y) {
	LCD_DrawFastHLine(target_x-4, target_y, 9, BLACK);
	LCD_DrawFastVLine(target_x, target_y-4, 9, BLACK);
}

static uint32_t Sum(uint32_t* matrix) {
	uint32_t sum = 0;
	uint8_t i = 0;
	for (i=0; i<CALIB_POINTS_QUANTITY; i++) {
		sum += matrix[i];
	}
	return sum;
}

static uint32_t MultiplyAndSum(uint32_t* m1, uint32_t* m2) {
	uint32_t sum = 0;
	uint8_t i = 0;
	for (i=0; i<CALIB_POINTS_QUANTITY; i++) {
		sum += m1[i]*m2[i];
	}
	return sum;
}
