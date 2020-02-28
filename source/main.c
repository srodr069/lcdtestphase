/*	Author: steven
 *  Partner(s) Name: 
 *	Lab Section:
 *	Assignment: Lab #lcdtake8  Exercise #
 *	Exercise Description: [optional - include for your own benefit]
 *
 *	I acknowledge all content contained herein, excluding template or example
 *	code, is my own original work.
 */
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#ifdef _SIMULATE_
#include "simAVRHeader.h"
#endif
#include <avr/pgmspace.h>
#include <util/delay.h> /* might be <avr/delay.h>, depending on toolchain */

#include "ILI9163.h"
#include "demoloop.h"

int main(void) 
    /* Insert DDR and PORT initializations */

    /* Insert your solution below */
{
	//demoloop(); // never returns
	unsigned char x = 0;
	unsigned char y = 0;

	LCD_init();
	LCD_FillScreen( LCD_RGB(0,0,0) );
	LCD_Orientation(LCD_ROT_0);
	_delay_ms(70);
	LCD_Orientation(LCD_ROT_180);
	_delay_ms(70);
	LCD_Orientation(LCD_ROT_0);
	_delay_ms(70);
	LCD_Orientation(LCD_ROT_180);
	_delay_ms(70);
	

	

	for(int i = 50; i > 0; i--){
		LCD_SetPixel(x, 0, LCD_MAGENTA );
		x++;
		
		LCD_SetPixel(0, y, LCD_BLUE );
		y++;

		_delay_ms(100);
		

	}
	




	
	return 1;
}