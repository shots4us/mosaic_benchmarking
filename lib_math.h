#ifndef LIB_MATH
#define LIB_MATH

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

using namespace std;
using namespace cv;

bool UnitTestCreateCovarianceMatrix();
bool covariance(const Mat in_a, const Mat in_b, float &cov);
bool createCovarianceMatrix(const Mat in, Mat &cov);
void beucherGradient(const Mat gray_src, Mat &gradient);

#endif // LIB_MATH

