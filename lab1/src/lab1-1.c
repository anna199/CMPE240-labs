/*
===============================================================================
 Name        : DrawLine.c
 Author      : $RJ
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include <cr_section_macros.h>
#include <NXP/crp.h>
#include "LPC17xx.h"                        /* LPC17xx definitions */
#include "ssp.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>



/* Be careful with the port number and location number, because

some of the location may not exist in that port. */

#define PORT_NUM            0


uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];


#define ST7735_TFTWIDTH 127
#define ST7735_TFTHEIGHT 159

#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_SLPOUT 0x11
#define ST7735_DISPON 0x29



#define swap(x, y) {x = x + y; y = x - y; x = x - y ;}

// defining color values

#define LIGHTBLUE 0x00FFE0
#define GREEN 0x00FF00
#define DARKBLUE 0x000033
#define BLACK 0x000000
#define BLUE 0x0007FF
#define RED 0xFF0000
#define MAGENTA 0x00F81F
#define WHITE 0xFFFFFF
#define PURPLE 0xCC33FF



int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;


void spiwrite(uint8_t c)

{

 int pnum = 0;

 src_addr[0] = c;

 SSP_SSELToggle( pnum, 0 );

 SSPSend( pnum, (uint8_t *)src_addr, 1 );

 SSP_SSELToggle( pnum, 1 );

}



void writecommand(uint8_t c)

{

 LPC_GPIO0->FIOCLR |= (0x1<<21);

 spiwrite(c);

}



void writedata(uint8_t c)

{

 LPC_GPIO0->FIOSET |= (0x1<<21);

 spiwrite(c);

}



void writeword(uint16_t c)

{

 uint8_t d;

 d = c >> 8;

 writedata(d);

 d = c & 0xFF;

 writedata(d);

}



void write888(uint32_t color, uint32_t repeat)

{

 uint8_t red, green, blue;

 int i;

 red = (color >> 16);

 green = (color >> 8) & 0xFF;

 blue = color & 0xFF;

 for (i = 0; i< repeat; i++) {

  writedata(red);

  writedata(green);

  writedata(blue);

 }

}



void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)

{

 writecommand(ST7735_CASET);

 writeword(x0);

 writeword(x1);

 writecommand(ST7735_RASET);

 writeword(y0);

 writeword(y1);

}


void fillrect(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)

{

 int16_t i;

 int16_t width, height;

 width = x1-x0+1;

 height = y1-y0+1;

 setAddrWindow(x0,y0,x1,y1);

 writecommand(ST7735_RAMWR);

 write888(color,width*height);

}



void lcddelay(int ms)

{

 int count = 24000;

 int i;

 for ( i = count*ms; i--; i > 0);

}



void lcd_init()

{

 int i;
 printf("LCD Demo Begins!!!\n");
 // Set pins P0.16, P0.21, P0.22 as output
 LPC_GPIO0->FIODIR |= (0x1<<16);

 LPC_GPIO0->FIODIR |= (0x1<<21);

 LPC_GPIO0->FIODIR |= (0x1<<22);

 // Hardware Reset Sequence
 LPC_GPIO0->FIOSET |= (0x1<<22);
 lcddelay(500);

 LPC_GPIO0->FIOCLR |= (0x1<<22);
 lcddelay(500);

 LPC_GPIO0->FIOSET |= (0x1<<22);
 lcddelay(500);

 // initialize buffers
 for ( i = 0; i < SSP_BUFSIZE; i++ )
 {

   src_addr[i] = 0;
   dest_addr[i] = 0;
 }

 // Take LCD display out of sleep mode
 writecommand(ST7735_SLPOUT);
 lcddelay(200);

 // Turn LCD display on
 writecommand(ST7735_DISPON);
 lcddelay(200);

}




void drawPixel(int16_t x, int16_t y, uint32_t color)

{

 if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))

 return;

 setAddrWindow(x, y, x + 1, y + 1);

 writecommand(ST7735_RAMWR);

 write888(color, 1);

}



/*****************************************************************************


** Descriptions:        Draw line function

**

** parameters:           Starting point (x0,y0), Ending point(x1,y1) and color

** Returned value:        None

**

*****************************************************************************/


void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)

{

 int16_t slope = abs(y1 - y0) > abs(x1 - x0);

 if (slope) {

  swap(x0, y0);

  swap(x1, y1);

 }

 if (x0 > x1) {

  swap(x0, x1);

  swap(y0, y1);

 }

 int16_t dx, dy;

 dx = x1 - x0;

 dy = abs(y1 - y0);

 int16_t err = dx / 2;

 int16_t ystep;

 if (y0 < y1) {

  ystep = 1;

 }

 else {

  ystep = -1;

 }

 for (; x0 <= x1; x0++) {

  if (slope) {

   drawPixel(y0, x0, color);

  }

  else {

   drawPixel(x0, y0, color);

  }

  err -= dy;

  if (err < 0) {

   y0 += ystep;

   err += dx;

  }

 }

}


/*

 Main Function main()

*/
void drawSaver() {
	double x[4],y[4], x_new[4],y_new[4];

			 double lamda = 0.8;
			 x[0] = rand() %100;
			 y[0] = rand() %100;
			 int length = 50 + rand() %50;
			 x[1] = x[0] + length;
			 y[1] = y[0];
			 x[2] = x[1];
			 y[2] = y[0] + length;
			 x[3] = x[0];
			 y[3] = y[2];
			 uint32_t colors[5] = {PURPLE, BLUE, RED, GREEN, BLACK};
			 uint32_t randColor = colors[rand()%5];
			 for (int j = 0; j < 10 ; j++){
				 for (int i = 0; i < 4; i++) {
						 drawLine(x[i],y[i],x[(i+1)%4],y[(i+1)%4], randColor);
				}
				 for (int i = 0; i < 4; i++) {
					  x_new[i] = x[i] + lamda * (x[(i+1)%4] - x[i]);
					  y_new[i] = y[i] + lamda * (y[(i+1)%4] - y[i]);
				 }
				 for (int i = 0; i < 4; i++) {
					 x[i] = x_new[i];
				 	 y[i] = y_new[i];
				 }
			 }
}


int main (void)

{

	 uint32_t pnum = PORT_NUM;

	 pnum = 0 ;

	 if ( pnum == 0 )
		 SSP0Init();

	 else
		 puts("Port number is not correct");

	 lcd_init();

	 fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, WHITE);

//	 int x0,x1,y0,y1;
//
//	 x0 = 30;
//	// x1 = 80;
//	 x1 = 30;
//	 y0 = 20;
//	 y1 = 60;
//
//	drawLine(x0,y0,x1,y1,PURPLE);

//	double alpha = 30.0 / 180 * 3.14;
//	double t11 = cos (alpha);
//	double t12 = -sin(alpha);
//	double t13 = -x0 * cos (alpha) + y0 * sin(alpha) + x0;
//	double t21 = sin (alpha);
//	double t22 = cos(alpha);
//	double t23 = -x0*sin(alpha) - y0 * cos(alpha) + y0;
//
//	double x_prime = t11 *x1 + t12 * y1 + t13;
//	double y_prime = t21 *x1 + t22 * y1 + t23;

	//drawLine(x0,y0,x_prime,y_prime,BLUE);
//	 double x[10];
//	 double x_prime_left[10];
//	 dou
//	 double y[10];
//	 x[0] = 30;
//	 y[0] = 20;
//	 x[1] = 30;
//	 y[1] = 50;
//	 double lamda = 0.8;
//	 drawLine(x[0],y[0],x[1],y[1],PURPLE);
//	 for (int i = 2; i < 5 ; i++){
//		 x[i] = x[i-1] + lamda * (x[i-1] - x[i-2]);
//		 y[i] = y[i-1] + lamda * (y[i-1] - y[i-2]);
//		 drawLine(x[i-1],y[i-1],x[i],y[i],BLUE);
//	 }
	 int i = 0;
	 while (i < 10){
	 	 drawSaver();
	 	 i++;
	 }
//		 double x[4],y[4], x_new[4],y_new[4];
//
//		 double lamda = 0.8;
//		 x[0] = rand() %100;
//		 y[0] = rand() %100;
//		 int length = 50 + rand() %50;
//		 x[1] = x[0] + length;
//		 y[1] = y[0];
//		 x[2] = x[1];
//		 y[2] = y[0] + length;
//		 x[3] = x[0];
//		 y[3] = y[2];
//		// uint32_t color = PURPLE;
//		 int i = 0;
//		 drawLine(x[i],y[i],x[(i+1)%4],y[(i+1)%4], PURPLE);
//		 for (int j = 0; j < 10 ; j++){
//			 for (int i = 0; i < 4; i++) {
//					 drawLine(x[i],y[i],x[(i+1)%4],y[(i+1)%4], PURPLE);
//			}
//			 for (int i = 0; i < 4; i++) {
//				  x_new[i] = x[i] + lamda * (x[(i+1)%4] - x[i]);
//				  y_new[i] = y[i] + lamda * (y[(i+1)%4] - y[i]);
//			 }
//			 for (int i = 0; i < 4; i++) {
//				 x[i] = x_new[i];
//			 	 y[i] = y_new[i];
//			 }
//		 }

	  return 0;

}




