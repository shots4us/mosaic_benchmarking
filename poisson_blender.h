#ifndef POISSON_BLENDER_H
#define POISSON_BLENDER_H

//-----------------------------------------------------------
// Poisson Blending
// Using stupid way (just solving Ax = b)
// Author: Eric Yuan
// Blog: http://eric-yuan.me
// You are free to use the following code for ANY purpose.
//
// Source: http://eric-yuan.me/poisson-blending/
// You can improve it by using advanced methods,
// About solving discrete Poisson Equation using Jacobi, SOR, Conjugate Gradients, and FFT
// see here: http://www.cs.berkeley.edu/~demmel/cs267/lecture24/lecture24.html
//
//-----------------------------------------------------------

#include <iostream>
#include <complex>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/flann/flann.hpp>

using namespace std;
using namespace cv;

class PoissonBlender
{
public:
    PoissonBlender();

    // calculate horizontal gradient, img(i, j + 1) - img(i, j)
    Mat getGradientXp(Mat &img);

    // calculate vertical gradient, img(i + 1, j) - img(i, j)
    Mat getGradientYp(Mat &img);

    // calculate horizontal gradient, img(i, j - 1) - img(i, j)
    Mat getGradientXn(Mat &img);

    // calculate vertical gradient, img(i - 1, j) - img(i, j)
    Mat getGradientYn(Mat &img);

    int getLabel(int i, int j, int height, int width);

    // get Matrix A.
    Mat getA(int height, int width);
    Mat getLaplacian();

    // Calculate b
    // using convolution.
    Mat getB1(Mat &img1, Mat &img2, int posX, int posY, Rect ROI);

    // Calculate b
    // using getGradient functions.
    Mat getB2(Mat &img1, Mat &img2, int posX, int posY, Rect ROI);

    // Solve equation and reshape it back to the right height and width.
    Mat getResult(Mat &A, Mat &B, Rect &ROI);

    // img1: 3-channel image, we wanna move something in it into img2.
    // img2: 3-channel image, dst image.
    // ROI: the position and size of the block we want to move in img1.
    // posX, posY: where we want to move the block to in img2
    Mat poisson_blending(Mat &img1, Mat &rgb2, Rect ROI, int posX, int posY);

    // Unit test method
    int unit_test();
};

#endif // POISSON_BLENDER_H
