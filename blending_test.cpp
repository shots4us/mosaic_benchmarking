#include "blending_test.h"
#include "qm.h"

#include <stdio.h>
#include <iostream>
#include <stdint.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ts/ts_gtest.h"
#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

// https://sourceforge.net/p/emgucv/opencv/ci/026b13b3dbe3d01778d74fd254dfa9f72adec9c3/tree/modules/stitching/test/test_blenders.cpp

using namespace cv;
using namespace cv::detail;
using namespace std;
using namespace qm;

//-----------------------------------------------------------------------------

int blending_test()
{
    const string path("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/");

    //Read first image

    Mat image1 = imread(path + string("test_img_11.png"));
    //Read Second Image
    Mat image2 = imread(path + string("test_img_22_64v.png"));

    //ASSERT_EQ(image1.rows, image2.row
    //"ASSERT_EQ(image1.cols, image2.cols);

    Mat result, result_mask;

    vector<Point> err(2);
    err[0].x = 0;
    err[0].y = 0;

    err[1].x = 0;
    err[1].y = 0;

    vector<Point> shift(2);
    shift[0].x = 0 - err[0].x;
    shift[0].y = 0 - err[0].y;

    shift[1].x = 0 - err[1].x;
    shift[1].y = 64 - err[1].y;

    blend(image1, image2, result, result_mask, shift, 80.0);

    //    Mat expected = imread(string(cvtest::TS::ptr()->get_data_path()) + "stitching/baboon_lena.png");
    //    double psnr_val = psnr(expected, result, 64);
    //    EXPECT_GE(psnr, 50);

    imwrite(path + "test_blending.jpg", result);

    //  imshow("result",result);
    //  imshow("result_mask",result_mask);

    //  waitKey(0);//needed for imashow
    return 0;
}

//-----------------------------------------------------------------------------

int blend(const Mat image1, const Mat image2, Mat &result, Mat &result_mask, vector<Point> shift, const float blend_strength)
{
    //  long start, end;
    // start = clock();

    Mat image1s, image2s;
    image1.convertTo(image1s, CV_16S);
    image2.convertTo(image2s, CV_16S);

    const Point pos_min(min(shift[0].x, shift[1].x),
            min(shift[0].y, shift[1].y));

    const Point pos_max(max(shift[0].x + image1.cols, shift[1].x + image2.cols),
            max(shift[0].y + image1.rows, shift[1].y + image2.rows));

    const Size canv_size(pos_max.x - pos_min.x, pos_max.y - pos_min.y);

    image1.convertTo(image1s, CV_16S);
    image2.convertTo(image2s, CV_16S);

    Mat mask1(canv_size, CV_8U);
    mask1.setTo(0);
    mask1(Rect(0, 0, pos_max.x, pos_max.y/2)).setTo(255);

    Mat mask2(canv_size, CV_8U);
    mask2.setTo(0);
    mask2(Rect(0, pos_max.y/2, pos_max.x, pos_max.y/2)).setTo(255);

    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/test_blend_mask1.png", mask1);
    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/test_blend_mask2.png", mask2);

    //MultiBandBlender blender(false, 5);
    // cout << "Shift 0: " << shift[0].x << "  " << shift[0].y << endl;
    // cout << "Shift 1: " << shift[1].x << "  " << shift[1].y << endl;
    FeatherBlender blender(blend_strength);
    blender.prepare(Rect(0, 0, canv_size.width, canv_size.height));
    blender.feed(image1s, mask1, Point(shift[0].x, shift[0].y));
    blender.feed(image2s, mask2, Point(shift[1].x, shift[1].y));

    Mat result_s; // result_mask;
    blender.blend(result_s, result_mask);

    result_s.convertTo(result, CV_8U);

    //end = clock();
    //cout<<"used time: "<<((double)(end - start)) / CLOCKS_PER_SEC<<" second"<<endl;
    return 0;
}
