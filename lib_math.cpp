#include "lib_math.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <math.h>
#include <stdint.h>

using namespace std;
using namespace cv;

//----------------------------------------------------------------------------
// UnitTestCreateCovarianceMatrix
//----------------------------------------------------------------------------
bool UnitTestCreateCovarianceMatrix()
{
    // Source: http://stattrek.com/matrix-algebra/covariance-matrix.aspx
    cv::Mat in = (cv::Mat_<float>(3,5));
    in.at<float>(0, 0) = 90.0;
    in.at<float>(0, 1) = 90.0;
    in.at<float>(0, 2) = 60.0;
    in.at<float>(0, 3) = 60.0;
    in.at<float>(0, 4) = 30.0;

    in.at<float>(1, 0) = 60.0;
    in.at<float>(1, 1) = 90.0;
    in.at<float>(1, 2) = 60.0;
    in.at<float>(1, 3) = 60.0;
    in.at<float>(1, 4) = 30.0;

    in.at<float>(2, 0) = 90.0;
    in.at<float>(2, 1) = 30.0;
    in.at<float>(2, 2) = 60.0;
    in.at<float>(2, 3) = 90.0;
    in.at<float>(2, 4) = 30.0;

    Mat t_in;
    transpose(in, t_in);
    Mat cov;

    // Creates a covariance matrix following the definition
    // Residuals are multiplied by unit matrix instead of summing them up
    cout << "--------------------------------------------------" << endl;
    cout << " 1. Computing Covariance Matrix by definition: " << endl;
    cout << "--------------------------------------------------" << endl;
    createCovarianceMatrix(t_in, cov);

    cout << "--------------------------------" << endl;
    cout << cov << endl;
    cout << "--------------------------------" << endl;

    // Creates a covariance matrix uing the OpenCv native function
    cout << "--------------------------------------------------" << endl;
    cout << " 2. Computing Covariance Matrix by OpenCv function: " << endl;
    cout << "--------------------------------------------------" << endl;
    Mat covar;
    Mat mean;

    // The OpenCv function is not normalized !!
    // The resulting matrix must be divided by N to make it compatible to the most known versions of covariance marix
    calcCovarMatrix(t_in, covar, mean,  CV_COVAR_NORMAL | CV_COVAR_ROWS, CV_32FC1);
    covar /= t_in.rows;

    cout << "--------------------------------" << endl;
    cout << covar << endl;
    cout << "--------------------------------" << endl;

    Mat diff = cov - covar;

    int zero_cnt = diff.rows * diff.cols;
    for (int r = 0; r < diff.rows; r++)
    {
        for (int c = 0; c < diff.cols; c++)
        {
            if (diff.at<float>(r, c) == 0)
            {
                zero_cnt--;
            }
        }
    }
    if (zero_cnt > 0)
    {
        return false;
    }
    return true;
}

//----------------------------------------------------------------------------
// createCovarianceMatrix
//----------------------------------------------------------------------------
bool createCovarianceMatrix(const Mat in, Mat &cov)
{   
   // transpose(in, t_in);
    Mat ones = Mat::ones(in.rows, in.rows, CV_32FC1);
    Mat res = in - ((ones * in/in.rows));
    Mat res_t;
    transpose(res, res_t);
    cov = (res_t * res) / in.rows;
    return true;
}

//----------------------------------------------------------------------------
// covariance
//----------------------------------------------------------------------------
bool covariance(const Mat in_a, const Mat in_b, float &cov)
{
    // Konig covariance theorem application
    float a_mean = 0.0;
    float b_mean = 0.0;

    for (int r = 0; r < in_a.rows; r++)
    {
        for (int c = 0; c < in_a.cols; c++)
        {
            a_mean += in_a.at<float>(r, c);
            b_mean += in_b.at<float>(r, c);
        }
    }
    a_mean /= (in_a.rows * in_a.cols);
    b_mean /= (in_b.rows * in_b.cols);
    Mat out;
    multiply(in_a, in_b, out);

    float c_sum = 0.0;

    for (int r = 0; r < out.rows; r++)
    {
        for (int c = 0; c < out.cols; c++)
        {
            c_sum += out.at<float>(r, c);
        }
    }
    c_sum /= (in_a.rows * in_a.cols);
    cov = c_sum - (a_mean * b_mean);
    return true;
}

//------------------------------------------------------------------------------------------------------------

void beucherGradient(const Mat gray_src, Mat &gradient)
{
    // Creates the structuring element
    Mat se;
    const uint8_t se_size_x = 3;
    const uint8_t se_size_y = 3;
    se = Mat::ones(se_size_x, se_size_y, CV_8U);

    // Applies Erosion
    Mat eroded = Mat::zeros(gray_src.rows, gray_src.cols, CV_8UC1);
    erode(gray_src, eroded, se);
    // Applies Dilation
    Mat dilated = Mat::zeros(gray_src.rows, gray_src.cols, CV_8UC1);
    dilate(gray_src, dilated, se);

    // Calculates the Beucher gradient
    gradient = Mat(dilated - eroded);
    // imwrite("D:/PRJ/Benchmark/Set_0000/Luminance/eroded.png", eroded);
    //  imwrite("D:/PRJ/Benchmark/Set_0000/Luminance/dilated.png", dilated);
    //  imwrite("D:/PRJ/Benchmark/Set_0000/Luminance/beucher.png", gradient);
}
