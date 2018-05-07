/*
 * constants.h
 *
 *  Created on: May 1, 2018
 *      Author: Ran
 */

#ifndef POINTS_H_
#define POINTS_H_

typedef struct  Point3D
{
    int x;
    int y;
    int z;
} Point3D;

typedef struct
{
    int x;
    int y;
} Point;


extern Point world2Plane(Point3D x, double sin_theta, double cos_theta, double sin_phi, double cos_phi, double rho, int D);


#endif /* POINTS_H_ */
