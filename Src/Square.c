/*
 * Square.c
 *
 *  Created on: 19 זמגע. 2016 נ.
 *      Author: Home
 */
#include "stm32f4xx_hal.h"
#include "Square.h"
#include "TouchScreen.h"

void InitSquare(SquareStruct* sq, int32_t sq_x, int32_t sq_y, int32_t sq_edge, uint16_t sq_color) {
	sq->x = sq_x;
	sq->y = sq_y;
	sq->edge = sq_edge;
	sq->color = sq_color;
	//sq->move = RESET;
	//sq->touch_point_x = 0;
	//sq->touch_point_y = 0;
	PrintSquare(sq);
}

void PrintSquare(SquareStruct* sq) {
	LCD_FillRect(sq->x, sq->y, sq->edge, sq->edge, sq->color);
}

void ClearSquare(SquareStruct* sq) {
	LCD_FillRect(sq->x, sq->y, sq->edge, sq->edge, BACKGROUND_COLOR);
}

void MoveSquare(SquareStruct* sq, int32_t mx, int32_t my) {
	ClearSquare(sq);
	sq->x = mx;
	sq->y = my;
	PrintSquare(sq);
}

int8_t IsSquareSelect(SquareStruct* sq, int32_t x, int32_t y) {
	//uint32_t x = GetTouch_X
	if (x>=sq->x && x<=sq->x+sq->edge && y>=sq->y && y<=sq->y+sq->edge) return 1;
	else return 0;
}

SquareCatchStatus IsCatch(SquareStruct* sq) {
	return sq->state;
}

void CatchSquare(SquareStruct* sq) {
	sq->state = CATCH;
}

void ReleaseSquare(SquareStruct* sq) {
	sq->state = RELEASE;
}
