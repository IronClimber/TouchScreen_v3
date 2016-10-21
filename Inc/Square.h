/*
 * Square.h
 *
 *  Created on: 19 זמגע. 2016 נ.
 *      Author: Home
 */

#ifndef SQUARE_H_
#define SQUARE_H_

#include "stm32f4xx_hal.h"
#include "display\lcd.h"

#define BACKGROUND_COLOR BLACK

typedef enum {
	CATCH = 1,
	RELEASE = 0
} SquareCatchStatus;

typedef struct {
	volatile int32_t x;
	volatile int32_t y;
	int32_t edge;
	uint16_t color;
	//uint8_t move;
	//uint32_t touch_point_x;
	//uint32_t touch_point_y;
	SquareCatchStatus state;
} SquareStruct;

void InitSquare(SquareStruct* sq, int32_t sq_x, int32_t sq_y, int32_t sq_edge, uint16_t sq_color);
void PrintSquare(SquareStruct* sq);
void ClearSquare(SquareStruct* sq);
void MoveSquare(SquareStruct* sq, int32_t mx, int32_t my);

int8_t IsSquareSelect(SquareStruct* sq, int32_t x, int32_t y);

SquareCatchStatus IsCatch(SquareStruct* sq);

void CatchSquare(SquareStruct* sq);
void ReleaseSquare(SquareStruct* sq);
#endif /* SQUARE_H_ */
