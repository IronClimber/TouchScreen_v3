/*
 * Square.c
 *
 *  Created on: 19 זמגע. 2016 נ.
 *      Author: Home
 */
#include "stm32f4xx_hal.h"
#include "Square.h"
#include "TouchScreen.h"

void InitSquare(SquareStruct* sq, int32_t sq_x, int32_t sq_y, uint32_t sq_edge, uint16_t sq_color) {
	sq->x = sq_x;
	sq->y = sq_y;
	sq->edge = sq_edge;
	sq->color = sq_color;
	sq->move = RESET;
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

uint8_t IsSquareSelect(SquareStruct* sq, uint32_t x, uint32_t y) {
	//uint32_t x = GetTouch_X
	if (x>=sq->x && x<=sq->x+sq->edge && y>=sq->y && y<=sq->y+sq->edge) return 1;
	else return 0;
}
