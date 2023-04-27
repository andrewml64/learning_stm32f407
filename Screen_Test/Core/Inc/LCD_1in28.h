/*****************************************************************************
* | File      	:   LCD_1inch28.h
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master 
*                and enhance portability
*----------------
* |	This version:   V1.0
* | Date        :   2020-12-06
* | Info        :   Basic version
*
******************************************************************************/
#ifndef __LCD_1IN28_H
#define __LCD_1IN28_H	
	
#include <stdint.h>
#include <stdlib.h>		//itoa()
#include <stdio.h>
#include "stm32f4xx_hal.h"

#define LCD_1IN28_HEIGHT 240
#define LCD_1IN28_WIDTH 240

#define HORIZONTAL 0
#define VERTICAL   1

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

typedef struct{
	UWORD WIDTH;
	UWORD HEIGHT;
	UBYTE SCAN_DIR;
}LCD_1IN28_ATTRIBUTES;
extern LCD_1IN28_ATTRIBUTES LCD_1IN28;

/********************************************************************************
function:	
			Macro definition variable name
********************************************************************************/
int Module_Init(void);
void Module_Exit(void);
void LCD_1IN28_Init(UBYTE Scan_dir);
void LCD_1IN28_Clear(UWORD Color);
void LCD_1IN28_Display(UWORD *Image);
void LCD_1IN28_DisplayWindows(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend, UWORD *Image);
void LCD_1IN28_DisplayPoint(UWORD X, UWORD Y, UWORD Color);
void LCD_1IN28_DrawPaint(UWORD x, UWORD y, UWORD Color);
//void LCD_1IN28_SetBackLight(UWORD Value);
#endif
