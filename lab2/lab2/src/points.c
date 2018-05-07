/*
 * constants.c
 *
 *  Created on: May 1, 2018
 *      Author: Ran
 */

#include "points.h";




Point world2Plane(Point3D x, double sin_theta, double cos_theta, double sin_phi, double cos_phi, double rho, int D)
{
	double x_v = -sin_theta * x.x + cos_theta * x.y;
	double y_v = -cos_phi * cos_theta * x.x -cos_phi*sin_theta * x.y + sin_phi * x.z;
	double z_v = -sin_phi * cos_theta* x.x -sin_phi*cos_theta * x.y + -cos_phi * x.z + rho;
    Point p = {D/z_v * x_v, D/z_v * y_v};
    return p;
}
