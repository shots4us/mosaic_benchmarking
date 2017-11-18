#include "imagestitching.h"
#include "image_compose.h"
#include "luc/WatershedMarkerSoille.hpp"
#include "stitch_graphcut.h"
#include "poisson_new.h"
#include "lib_math.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <limits.h>
#include <string>
#include <chrono>
#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/flann/flann.hpp>

#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui/highgui.hpp"
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

#ifdef IS_LINUX
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#endif

using namespace std;
using namespace cv;
using namespace cv::detail;

ImageStitching::ImageStitching()
{

}

ImageStitching::~ImageStitching()
{

}

//--------------------------------------------------------------------------------------------

bool ImageStitching::nominalStitching(const Mat rgb_1, Mat &rgb_2)
{
    for (int r = 0; r < rgb_2.rows; r++)
    {
        for (int c = 0; c < rgb_2.cols; c++)
        {
            rgb_2.at<Vec3b>(r, c)[0] = rgb_1.at<Vec3b>(r, c)[0];
            rgb_2.at<Vec3b>(r, c)[1] = rgb_1.at<Vec3b>(r, c)[1];
            rgb_2.at<Vec3b>(r, c)[2] = rgb_1.at<Vec3b>(r, c)[2];
        }
    }
    return true;
}

//--------------------------------------------------------------------------------------------

bool ImageStitching::blending(const Mat rgb_src_1,
                              const Mat rgb_src_2,
                              const Point corner_1,
                              const Point corner_2,
                              Mat &rgb_stitch,
                              const int blend_type,
                              const float blend_strength)
{
    vector<Point> corners;
    corners.push_back(corner_1);
    corners.push_back(corner_2);

    vector<Size> sizes;
    sizes.push_back(rgb_src_1.size());
    sizes.push_back(rgb_src_2.size());

    const bool try_gpu = false;

    Mat image1s, image2s;
    rgb_src_1.convertTo(image1s, CV_16S);
    rgb_src_2.convertTo(image2s, CV_16S);

    const Point pos_max(max(corners[0].x + rgb_src_1.cols, corners[1].x + rgb_src_2.cols),
            max(corners[0].y + rgb_src_1.rows, corners[1].y + rgb_src_2.rows));

    const Size canv_size(pos_max.x, pos_max.y);

    rgb_src_1.convertTo(image1s, CV_16S);
    rgb_src_2.convertTo(image2s, CV_16S);

    Mat mask1(rgb_src_1.size(), CV_8U);
    mask1.setTo(255);

    Mat mask2(rgb_src_2.size(), CV_8U);
    mask2.setTo(255);

    Ptr<Blender> blender;

    const Size dst_sz = resultRoi(corners, sizes).size();
    float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;

    if (blend_width < 1.f)
    {
        blender = Blender::createDefault(Blender::NO, try_gpu);
#ifdef IS_VERBOSE
        cout << "Blend width too low: " << blend_width << " No Blending performed !!" << endl;
#endif
    }
    else
    {
        blender = Blender::createDefault((blend_type), try_gpu);

        if (blend_type == Blender::MULTI_BAND)
        {
            blender = Blender::createDefault(Blender::MULTI_BAND, try_gpu);
            MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(static_cast<Blender*>(blender));

            mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
#ifdef IS_VERBOSE
            cout << "Multi-band blender, number of bands: " << mb->numBands();
#endif
        }
        else if (blend_type == Blender::FEATHER)
        {
            FeatherBlender* fb = dynamic_cast<FeatherBlender*>(static_cast<Blender*>(blender));
            fb->setSharpness(1.f/blend_width);
#ifdef IS_VERBOSE
            cout << "Feather blender, sharpness: " << fb->sharpness();
#endif
        }
    }
    blender->prepare(Rect(0, 0, canv_size.width, canv_size.height));
    blender->feed(image1s, mask1, Point(0, 0));
    blender->feed(image2s, mask2, Point(corners[1].x, corners[1].y));

    Mat result_s;
    Mat result_mask;

    blender->blend(result_s, result_mask);
    result_s.convertTo(rgb_stitch, CV_8U);
    return true;
}

//--------------------------------------------------------------------------------------------
// Levelset class algorithms
//--------------------------------------------------------------------------------------------
bool ImageStitching::watershed(const Mat rgb_src_1,
                               const Mat rgb_src_2,
                               const Mat marker,
                               Mat &rgb_stitch,
                               const bool remove_seam)
{
    //--------------------------------------------------------------------------
    // Begin Conversion image RGB 1 -> Gray
    //--------------------------------------------------------------------------

    Mat img_float_1 = Mat::zeros(rgb_src_1.size(), CV_32FC3);
    Mat hsv_1 = Mat::zeros(rgb_src_1.size(), CV_32FC3);
    rgb_src_1.convertTo(img_float_1, CV_32FC3);
    cvtColor(img_float_1, hsv_1, COLOR_RGB2HSV);

    Mat ch_1[3];
    ch_1[0] = Mat::zeros(rgb_src_1.size(), CV_32FC1);
    ch_1[1] = Mat::zeros(rgb_src_1.size(), CV_32FC1);
    ch_1[2] = Mat::zeros(rgb_src_1.size(), CV_32FC1);
    split(hsv_1, ch_1);
    Mat gray_1 = ch_1[2];
    Mat grad_1;
    beucherGradient(gray_1, grad_1);
    //--------------------------------------------------------------------------
    // End Conversion image RGB 1 -> Gray
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // Begin Conversion image RGB 2 -> Gray
    //--------------------------------------------------------------------------
    Mat img_float_2 = Mat::zeros(rgb_src_2.size(), CV_32FC3);

    Mat hsv_2 = Mat::zeros(rgb_src_2.size(), CV_32FC3);
    rgb_src_2.convertTo(img_float_2, CV_32FC3);
    cvtColor(img_float_2, hsv_2, COLOR_RGB2HSV);

    Mat ch_2[3];
    ch_2[0] = Mat::zeros(rgb_src_2.size(), CV_32FC1);
    ch_2[1] = Mat::zeros(rgb_src_2.size(), CV_32FC1);
    ch_2[2] = Mat::zeros(rgb_src_2.size(), CV_32FC1);
    split(hsv_2, ch_2);
    Mat gray_2 = ch_2[2];
    Mat grad_2;
    beucherGradient(gray_2, grad_2);
    //--------------------------------------------------------------------------
    // End Conversion image RGB 2 -> Gray
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // Applies the Watershed to the input image
    //--------------------------------------------------------------------------
    ImageCompose img_compo;

    // Marker table
    img_compo.m_ptr_marker = new Mat(marker.size(), marker.type());
    marker.copyTo(*(img_compo.m_ptr_marker));

    // Mask table
    const int canvas_max_rows = max(grad_1.rows, grad_2.rows);
    const int canvas_max_cols = max(grad_1.cols, grad_2.cols);

    Mat mask = Mat::ones(canvas_max_rows, canvas_max_cols, CV_8UC1);
    img_compo.projection(grad_1, grad_2, mask);

#ifdef IS_VERBOSE
    cout << "Computing the optimal seam for the nominal image" << endl;
#endif
    img_compo.m_ptr_marker->convertTo(*img_compo.m_ptr_marker, CV_32SC1);

    WatershedMarkerSoille ws;
    ws.process(mask, *img_compo.m_ptr_marker);
    img_compo.m_ptr_marker->convertTo(*img_compo.m_ptr_marker, CV_8U);

    //--------------------------------------------------------------------------
    // Makes the marker mask visible
    //--------------------------------------------------------------------------
    rgb_stitch = Mat::zeros(rgb_src_1.size(), rgb_src_1.type());
    Mat patch_0 = Mat::zeros(img_compo.m_ptr_marker->size(), img_compo.m_ptr_marker->type());

    if (remove_seam == true)
    {
        RemoveWatersheds rem_wsh;
        rem_wsh.remove(*img_compo.m_ptr_marker);
    }
    img_compo.applyMaskToComposite(rgb_src_1, rgb_src_2, patch_0, rgb_stitch);
    // imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/patch_0.png", patch_0);
    return true;
}

//--------------------------------------------------------------------------------------------

bool ImageStitching::graphcut(const Mat rgb_src_1,
                              const Mat rgb_src_2,
                              const Point corners_1,
                              const Point corners_2,
                              const string seam_find_type,
                              Mat &rgb_stitch)
{
    vector<Point> shift(2);
    shift[0] = corners_1;
    shift[1] = corners_2;

    vector<Mat> images(2);
    images[0] = rgb_src_1;
    images[1] = rgb_src_2;

    if (StitchGraphcut::create_stitch_graphcut(images, shift, seam_find_type, rgb_stitch) == 1)
    {
#ifdef IS_VERBOSE
        cout << "Graph-cut stitching failed !!" << endl;
#endif
        return false;
    }
    return true;
}

//--------------------------------------------------------------------------------------------

bool ImageStitching::poisson(const Mat rgb_src_1,
                             const Mat rgb_src_2,
                             const Point corners_1,
                             const Point corners_2,
                             Mat &rgb_stitch)
{
    if (rgb_src_1.rows == 0 || rgb_src_1.cols == 0)
    {
#ifdef IS_VERBOSE
        cout << "Image 1 failed !!" << endl;
#endif
        exit(-1);
    }

    if (rgb_src_2.rows == 0 || rgb_src_2.cols == 0)
    {
#ifdef IS_VERBOSE
        cout << "Image 2 failed !!" << endl;
#endif
        exit(-1);
    }

    vector<Point> shift;
    shift.push_back(corners_1);
    shift.push_back(corners_2);

    IplImage *img_0 = new IplImage(rgb_src_1);
    IplImage *img_2 = new IplImage(rgb_src_2);
    IplImage *img_1 = NULL;
    IplImage *subimg = NULL;


    const Point pos_min(min(shift[0].x, shift[1].x),
            min(shift[0].y, shift[1].y));

    const Point pos_max(max(shift[0].x + rgb_src_1.cols, shift[1].x + rgb_src_2.cols),
            max(shift[0].y + rgb_src_1.rows, shift[1].y + rgb_src_2.rows));

    const Size canv_size(pos_max.x - pos_min.x, pos_max.y - pos_min.y);

    Mat trg_img = Mat::zeros(canv_size, CV_8UC3);

    Mat mask_0(canv_size, CV_8UC3);
    mask_0 = trg_img(Rect(shift[0].x, shift[0].y, rgb_src_1.cols, rgb_src_1.rows));
    rgb_src_1.copyTo(mask_0);

    Mat mask_2(canv_size, CV_8UC3);
    mask_2 = trg_img(Rect(shift[1].x, shift[1].y, rgb_src_2.cols, rgb_src_2.rows));

    rgb_src_2.copyTo(mask_2);
    img_2 = new IplImage(trg_img);

    CvPoint pt_1;
    pt_1.x = 0;
    pt_1.y = 0;

    CvPoint pt_2;
    pt_2.x = img_0->width - 1;
    pt_2.y = img_0->height - 1;

    // Target position of the selected subimg
    img_1 = cvCloneImage(img_0);
    subimg = cvCreateImage(cvGetSize(img_1), img_1->depth, img_1->nChannels);
    cvCopy(img_1, subimg, NULL);
    cvSetImageROI(img_1,cvRect(pt_2.x, pt_2.y, pt_2.x - pt_1.x, pt_2.y - pt_1.y));

    if(shift[0].x + subimg->width > img_2->width || shift[1].y + subimg->height > img_2->height)
    {
#ifdef IS_VERBOSE
        cout << "Index out of range" << endl;
#endif
        exit(0);
    }
    rgb_stitch = poisson_blend(img_2, subimg, shift[0].y, shift[0].x);

    //cvReleaseImage(&img_0);
    //cvReleaseImage(&img_2);
}
