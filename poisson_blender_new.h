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

#ifndef POISSON_BLENDER_NEW_H
#define POISSON_BLENDER_NEW_H

#include "global.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/compat.hpp>
#include "math.h"

using namespace std;
using namespace cv;

void mouseHandler1(int event, int x, int y, int flags, void* param);
void mouseHandler(int event, int x, int y, int flags, void* param);
void drawImage(IplImage* target, IplImage* source, int x, int y);
int unit_test();

#endif // POISSON_BLENDER_NEW_H
