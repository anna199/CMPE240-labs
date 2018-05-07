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
#include "points.h"




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
int _xoffset = 64;
int _yoffset = 80;


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

typedef struct
{
    int x;
    uint32_t color;
} PointInfo;

void drawColorLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color, uint32_t color_end, PointInfo pinfo[], int min) {
 int16_t slope = abs(y1 - y0) > abs(x1 - x0);
 if (slope) {
  swap(x0, y0);
  swap(x1, y1);
 }

 if (x0 > x1) {
  swap(x0, x1);
  swap(y0, y1);
  swap(color, color_end); // swap color value when swap dx and dy.
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

 int combineDeltaColor;


 int flag = 1;
 if (color_end < color) {
	 swap(color_end, color);
	 flag = -1;
 }
  int diffColor_red = ( (color_end & 0xFF0000) >> 16 ) - ( (color & 0xFF0000) >> 16 );
  int diffColor_green = ( (color_end & 0xFF00) >> 8 ) - ( (color & 0xFF00) >> 8 );
  int diffColor_blue = (color_end & 0xFF) - (color & 0xFF);


   int x_step = abs(x1 - x0);
     int y_step = abs(y1 - y0);
     int deltaColor_red = (diffColor_red / x_step + diffColor_red / y_step) / 2;
     int deltaColor_green = (diffColor_green / x_step + diffColor_green / y_step) / 2;
     int deltaColor_blue = (diffColor_blue / x_step + diffColor_blue / y_step) / 2;

     combineDeltaColor = (deltaColor_red << 16) | (deltaColor_green << 8) | (deltaColor_blue);



  if (flag == -1){
	  swap(color_end, color);
  }

  for (; x0 <= x1; x0++) {

   if (slope) {

    drawPixel(y0, x0, color);
    PointInfo pf = {y0, color};
    pinfo[x0-min - _yoffset] = pf;
   }
   else {
	  color += flag * combineDeltaColor;
    drawPixel(x0, y0, color);
    PointInfo pf = {x0, color};
    pinfo[y0-min- _yoffset] = pf;
   }
   err -= dy;
   if (err < 0) {
    y0 += ystep;
    err += dx;
   }
  }
  //
}

void drawColorLine2(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color, uint32_t color_end) {
 int16_t slope = abs(y1 - y0) > abs(x1 - x0);
 if (slope) {
  swap(x0, y0);
  swap(x1, y1);
//  swap(color, color_end); // GUESS
 }

 if (x0 > x1) {
  swap(x0, x1);
  swap(y0, y1);
  swap(color, color_end); // swap color value when swap dx and dy.
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
 int combineDeltaColor;

 int flag = 1;
 if (color_end < color) {
	 swap(color_end, color);
	 flag = -1;
 }
  int diffColor_red = ( (color_end & 0xFF0000) >> 16 ) - ( (color & 0xFF0000) >> 16 );

  int diffColor_green = ( (color_end & 0xFF00) >> 8 ) - ( (color & 0xFF00) >> 8 );
  int diffColor_blue = (color_end & 0xFF) - (color & 0xFF);

  int x_step = abs(x1 - x0);
  int y_step = abs(y1 - y0);
  int deltaColor_red = (diffColor_red / x_step + diffColor_red / y_step) / 2;
  int deltaColor_green = (diffColor_green / x_step + diffColor_green / y_step) / 2;
  int deltaColor_blue = (diffColor_blue / x_step + diffColor_blue / y_step) / 2;

  combineDeltaColor = (deltaColor_red << 16) | (deltaColor_green << 8) | (deltaColor_blue);


  if (flag == -1) {
 	 swap(color_end, color);
  }
  for (; x0 <= x1; x0++) {
   if (slope) {
    color += flag * combineDeltaColor;
    drawPixel(y0, x0, color);

   }
   else {
    color += flag * combineDeltaColor;
    drawPixel(x0, y0, color);

   }
   err -= dy;
   if (err < 0) {
    y0 += ystep;
    err += dx;
   }
  }
  //
}



void drawLine2OneColor(Point x0, Point x1, uint32_t color0){
	drawLine(x0.x + _xoffset,  x0.y +_yoffset, x1.x + _xoffset, x1.y + _yoffset, color0);
}

void drawLine2(Point x0, Point x1, uint32_t color0, uint32_t color1, PointInfo pinfo[], int min){
	drawColorLine(x0.x + _xoffset,  x0.y + _yoffset, x1.x + _xoffset, x1.y + _yoffset, color0, color1, pinfo, min);
}

void drawOrigin(double sin_theta, double cos_theta, double sin_phi, double cos_phi, double rho, int D){

	 Point3D x_world = {100, 0 ,0};
	 Point3D y_world = {0, 100 ,0};
	 Point3D z_world = {0, 0, 100};

	 Point x = world2Plane(x_world, sin_theta, cos_theta, sin_phi, cos_phi, rho, D);
		 Point y = world2Plane(y_world, sin_theta, cos_theta, sin_phi, cos_phi, rho, D);
		 Point z = world2Plane(z_world, sin_theta, cos_theta, sin_phi, cos_phi, rho, D);
		 Point origin = {0,0};
		 drawLine2OneColor(x, origin, BLUE);
		 drawLine2OneColor(y, origin, RED);
		 drawLine2OneColor(z, origin, GREEN);
}

void drawCube(Point p_2D[], Point3D P[],double sin_theta, double cos_theta, double sin_phi, double cos_phi, double rho, int D){
	 Point prev = {0,0};
	for (int i = 0; i< 8; i++){
		 p_2D[i] = world2Plane(P[i], sin_theta, cos_theta, sin_phi, cos_phi, rho, D);
		if (i != 4 && i != 1){
		 drawLine2OneColor(p_2D[i], prev, BLUE);
		}
		 prev = p_2D[i];
	}
	drawLine2OneColor(p_2D[1], p_2D[5], BLUE);
	drawLine2OneColor(p_2D[2], p_2D[6], BLUE);
	drawLine2OneColor(p_2D[3], p_2D[7], BLUE);
}

int getDiffuseReflection(Point3D E, Point3D P, double reflection){
	int K = 2000 * 100000;
	Point3D r = {(E.x - P.x), (E.y - P.y), (E.z - P.z)};
	Point3D n = {0, 0, 1};
	double dis = r.x * r.x + r.y * r.y + r.z * r.z;
	double cos_theta = (n.x* r.x + n.y * r.y + n.z* r.z)/ (sqrt(dis));
	return (int)K * 1/(dis) * cos_theta * reflection;
}


void drawL(Point p_2D[], Point3D P[],double sin_theta, double cos_theta, double sin_phi, double cos_phi, double rho, int D){
	 Point prev = {0,0};
	for (int i = 0; i< 6; i++){
		 p_2D[i] = world2Plane(P[i], sin_theta, cos_theta, sin_phi, cos_phi, rho, D);
		 if (i != 0){
			 drawLine2OneColor(p_2D[i], prev, BLUE);
		 }
		 prev = p_2D[i];
	}
	drawLine2OneColor(p_2D[0], p_2D[5], BLUE);
}


void drawSaver(double sin_theta, double cos_theta, double sin_phi, double cos_phi, double rho, int D) {
	double x[4],y[4], x_new[4],y_new[4];

			 double lamda = 0.8;
			 x[0] = 30;
			 y[0] = 130;
			 int length = 150;
			 x[1] = x[0] + length;
			 y[1] = y[0];
			 x[2] = x[1];
			 y[2] = y[0] + length;
			 x[3] = x[0];
			 y[3] = y[2];
			 Point3D P[4] = {{200, x[0], y[0]}, {200, x[1], y[1]}, {200, x[2], y[2]}, {200, x[3], y[3]}};
			 Point p_2D[4]={};
			 uint32_t randColor = GREEN;
			 for (int j = 0; j < 10 ; j++){
					 for (int i = 0; i < 5; i++) {
						 if (i < 4){
							 p_2D[i] = world2Plane(P[i], sin_theta, cos_theta, sin_phi, cos_phi, rho, D);
						 }
						 if (i >= 1){
							 drawLine2OneColor(p_2D[(i-1)%4],p_2D[(i)%4], randColor);
						 }
					 }
				 for (int i = 0; i < 4; i++) {
					  x_new[i] = x[i] + lamda * (x[(i+1)%4] - x[i]);
					  y_new[i] = y[i] + lamda * (y[(i+1)%4] - y[i]);
				 }
				 for (int i = 0; i < 4; i++) {
					 x[i] = x_new[i];
				 	 y[i] = y_new[i];
				 	 Point3D p3= {200, x[i], y[i]};
				 	 P[i]= p3;
				 }
			 }
}

void drawLineOneColor2(Point p1, Point p2, uint32_t color, int x[], int min)

{
	int16_t x0 = p1.x;
	int16_t y0 = p1.y;
	int16_t x1 = p2.x;
	int16_t y1 = p2.y;

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

   //drawPixel(y0, x0, color);
   x[x0-min] = y0;

  }

  else {

 //  drawPixel(x0, y0, color);
   x[y0-min] = x0;

  }

  err -= dy;

  if (err < 0) {

   y0 += ystep;

   err += dx;

  }

 }

}

void drawShade(Point3D P[], Point3D E, double sin_theta, double cos_theta, double sin_phi, double cos_phi, double rho, int D){
	Point3D p_new[4] ={};
	Point p_2d[4] ={};
	for (int i = 4; i < 8; i++){
		double lamda = - (P[i].z)/((E.z - P[i].z));
		Point3D p3D = {P[i].x + lamda * (E.x - P[i].x), P[i].y + lamda * (E.y -P[i].y), 0};
		p_new[i - 4] = p3D;
	}
	for (int i = 0; i < 5; i++) {
		if (i < 4){
			p_2d[i] = world2Plane(p_new[i], sin_theta, cos_theta, sin_phi, cos_phi, rho, D);
		}
		if (i != 0){
			drawLine2OneColor(p_2d[i % 4], p_2d[(i-1) %4], 0x000000);
		}
	}
		 int min = p_2d[0].y;
		 int max = p_2d[0].y;
		 for (int i = 1; i < 4; i++){
			 if (p_2d[i].y < min) {
				 min = p_2d[i].y;
			 }
			 if (p_2d[i].y > max){
				 max = p_2d[i].y;
			 }
		 }

		 int point_left_L[max - min + 1];
		 int point_right_L[max - min + 1];
		 drawLineOneColor2(p_2d[0],p_2d[1], 0x000000, point_right_L, min);
		 	 drawLineOneColor2(p_2d[1],p_2d[2], 0x000000, point_right_L, min);
		 	drawLineOneColor2(p_2d[2],p_2d[3], 0x000000, point_left_L, min);
		 	 drawLineOneColor2(p_2d[3],p_2d[0], 0x000000, point_left_L, min);

		 	for (int i = 0; i < max -min +1; i++){
		 		int tempy = min + i;
		 		 drawLine(point_left_L[i] + _xoffset, tempy + _yoffset, point_right_L[i] + _xoffset, tempy + _yoffset,0x000000);
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

	 double r = 1;
	 Point3D E = {600, 600, 600};
	 int D = 200;
	 int a = 300;
	 int b = 100;
	 Point3D P[8] = {{0, 0, b}, {a-b, 0, b}, {a-b, a-b, b}, {0, a-b, b}, {0, 0, a},{a-b, 0, a}, {a-b, a-b, a}, {0, a-b, a}};


	 double sin_theta = E.y/sqrt(E.x * E.x + E.y * E.y);
	 double cos_theta = E.x/sqrt(E.x * E.x + E.y * E.y);
	 double sin_phi = sqrt(E.x * E.x + E.y * E.y)/sqrt(E.x * E.x + E.y * E.y + E.z * E.z);
	 double cos_phi = E.z/sqrt(E.x * E.x + E.y * E.y + E.z * E.z);
	 double rho = sqrt(E.x * E.x + E.y * E.y + E.z * E.z);

	 drawOrigin(sin_theta, cos_theta, sin_phi, cos_phi, rho, D);

	 //drawShade(P, E, sin_theta, cos_theta, sin_phi, cos_phi, rho, D);
	 Point p_2D[8] = {};
	 drawCube(p_2D, P, sin_theta, cos_theta, sin_phi, cos_phi, rho, D);
	 double I_diff[8] = {};
	 I_diff[4] = getDiffuseReflection(E, P[4], r);
	 I_diff[5] = getDiffuseReflection(E, P[5], r);

	 I_diff[6] = getDiffuseReflection(E, P[6], r);

	 I_diff[7] = getDiffuseReflection(E, P[7], r);
	 PointInfo point_left[p_2D[4].y - p_2D[6].y+1];
	 PointInfo point_right[p_2D[4].y - p_2D[6].y+1];

	 drawLine2(p_2D[5],p_2D[4], (uint32_t)I_diff[5]<<16, (uint32_t)I_diff[4]<<16, point_right, p_2D[6].y );
	 drawLine2(p_2D[6],p_2D[5], (uint32_t)I_diff[6]<<16, (uint32_t)I_diff[5]<<16, point_right, p_2D[6].y );

	 drawLine2(p_2D[6],p_2D[7], (uint32_t)I_diff[6]<<16, (uint32_t)I_diff[7]<<16, point_left, p_2D[6].y );
	 drawLine2(p_2D[4],p_2D[7], (uint32_t)I_diff[4]<<16, (uint32_t)I_diff[7]<<16, point_left, p_2D[6].y );

	 for (int i = 0; i < p_2D[4].y - p_2D[6].y+1; i++){
		 int tempy = p_2D[6].y + i;
		 drawColorLine2(point_left[i].x, tempy + _yoffset, point_right[i].x, tempy + _yoffset, point_left[i].color, point_right[i].color );
	 }

	 int point_left2[p_2D[7].y - p_2D[2].y+1];
	 int point_right2[p_2D[7].y - p_2D[2].y+1];
	 uint32_t colorAmbient = 0x100000;
	 drawLineOneColor2(p_2D[3],p_2D[7], colorAmbient, point_left2, p_2D[2].y);
	 drawLineOneColor2(p_2D[2],p_2D[3], colorAmbient, point_left2, p_2D[2].y);
	 drawLineOneColor2(p_2D[2],p_2D[6], colorAmbient, point_right2, p_2D[2].y);
	 drawLineOneColor2(p_2D[6],p_2D[7], colorAmbient, point_right2, p_2D[2].y);

	 for (int i = 0; i < p_2D[7].y - p_2D[2].y+1; i++){
		 int tempy = p_2D[2].y + i;
		 drawLine(point_left2[i] + _xoffset, tempy + _yoffset, point_right2[i] + _xoffset, tempy + _yoffset,colorAmbient);
	 }

	 	 drawLineOneColor2(p_2D[1],p_2D[5], colorAmbient, point_left2, p_2D[2].y);
		 drawLineOneColor2(p_2D[1],p_2D[2], colorAmbient, point_left2, p_2D[2].y);
		 drawLineOneColor2(p_2D[6],p_2D[5], colorAmbient, point_right2, p_2D[2].y);

		 for (int i = 0; i < p_2D[5].y - p_2D[2].y+1; i++){
			 int tempy = p_2D[2].y + i;
			 drawLine(point_left2[i] + _xoffset, tempy + _yoffset, point_right2[i] + _xoffset, tempy + _yoffset,colorAmbient);
		 }



	 Point3D P_L[6] = {{50, 50, a}, {70, 50, a}, {70, 130, a}, {150, 130, a}, {150, 150, a},{50, 150, a}};
	// I_diff[6] ={};
	 Point p_2D_L[6] = {};
	 drawL(p_2D_L, P_L, sin_theta, cos_theta, sin_phi, cos_phi, rho, D);
	 int min = p_2D_L[0].y;
	 int max = p_2D_L[0].y;
	 for (int i = 1; i < 6; i++){
		 if (p_2D_L[i].y < min) {
			 min = p_2D_L[i].y;
		 }
		 if (p_2D_L[i].y > max){
			 max = p_2D_L[i].y;
		 }
	 }

	 int point_left_L[max - min + 1];
	 int point_right_L[max - min + 1];
	 drawLineOneColor2(p_2D_L[0],p_2D_L[1], BLUE, point_right_L, min);
	 	 drawLineOneColor2(p_2D_L[1],p_2D_L[2], BLUE, point_right_L, min);
	 	drawLineOneColor2(p_2D_L[2],p_2D_L[3], BLUE, point_right_L, min);
	 	drawLineOneColor2(p_2D_L[3],p_2D_L[4], BLUE, point_right_L, min);
	 	drawLineOneColor2(p_2D_L[4],p_2D_L[5], BLUE, point_left_L, min);
	 	 drawLineOneColor2(p_2D_L[5],p_2D_L[0], BLUE, point_left_L, min);

	 	 for (int i = 0; i < max -min +1; i++){
	 		int tempy = min + i;
	 		drawLine(point_left_L[i] + _xoffset, tempy + _yoffset, point_right_L[i] + _xoffset, tempy + _yoffset,BLUE);
	 	 }



	 for (int i = 0; i < 6; i++) {
		 Point3D p3 = {P_L[i].y, 200, P_L[i].x + b};
		 P_L[i] = p3;
	 }
	drawL(p_2D_L, P_L, sin_theta, cos_theta, sin_phi, cos_phi, rho, D);

	 drawSaver(sin_theta, cos_theta,  sin_phi, cos_phi, rho,  D);

	  return 0;

}




