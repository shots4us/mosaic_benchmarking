/*
#########################   Interactive Poisson Blending ############################

Copyright (C) 2012 Siddharth Kherada
Copyright (C) 2006-2012 Natural User Interface Group

Source: https://github.com/Siddharthk/CompPhoto-NUIGroup-GSoC-2012/blob/master/Interactive%20Poisson%20Blending/poisson.h

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details. "

#####################################################################################
*/

#ifndef POISSON_H
#define POISSON_H

#include "global.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"

using namespace std;
using namespace cv;

#define pi 3.1416

void display(const char *name, IplImage *img);
void getGradientx( const IplImage *img, IplImage *gx);
void getGradienty( const IplImage *img, IplImage *gy);
void lapx( const IplImage *img, IplImage *gxx);
void lapy( const IplImage *img, IplImage *gyy);
void dst(double *gtest, double *gfinal,int h,int w);
void idst(double *gtest, double *gfinal,int h,int w);
void transpose(double *mat, double *mat_t,int h,int w);
void poisson_solver(const IplImage *img, IplImage *gxx , IplImage *gyy, Mat &result);
IplImage *poisson_blend(IplImage *I, IplImage *mask, int posx, int posy);

#endif // POISSON_H
