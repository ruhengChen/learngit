/*
 * File:		nokia.h
 * Purpose:		Serial Input/Output routines
 *
 */

#ifndef _OELD_H
#define _OELD_H

#include "MK60_gpio.h"

/********************************************************************/
/*-----------------------------------------------------------------------
LCD_init          : OLED初始化

编写日期          ：2012-11-01
最后修改日期      ：2012-11-01
-----------------------------------------------------------------------*/
 extern uint8 lanzhou96x64[768];
 extern void OLED_Init(void);
 extern void OLED_CLS(void);
 extern void OLED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
 extern void OLED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
 extern void OLED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
 extern void OLED_Print(uint8 x, uint8 y, uint8 ch[]);
 extern void OLED_PutPixel(uint8 x,uint8 y);
 extern void OLED_Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 gif);
 extern void OLED_Set_Pos(uint8 x, uint8 y);
 extern void OLED_WrDat(uint8 data);
 extern void Draw_LibLogo(void);
 extern void Draw_Landzo(void);
 extern void Draw_BMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]);
 extern void OLED_Fill(uint8 dat);
 extern void Dly_ms(uint16 ms);
 extern void OLED_Prints(uint8 x, uint8 y, uint8 ch[]);
 extern void draw_line(uint8 startx,uint8 starty,uint8 endx,uint8 endy);
 uint8 my_abs(int a);

/********************************************************************/

#endif

