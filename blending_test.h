#ifndef BLENDING_TEST_H
#define BLENDING_TEST_H

#include "global.h"

#include <stdio.h>
#include <iostream>
#include <stdint.h>

// https://sourceforge.net/p/emgucv/opencv/ci/026b13b3dbe3d01778d74fd254dfa9f72adec9c3/tree/modules/stitching/test/test_blenders.cpp

using namespace cv;
using namespace cv::detail;
using namespace std;

int blending_test();
int blend(const Mat image1, const Mat image2, Mat &result, Mat &result_mask, const vector<Point> shift, const float blend_strength);
#endif // BLENDING_TEST_H
