
#include "image_compose.h"
#include "excelexport.h"
#include "hsvmanager.h"
#include "distortionmanager.h"
#include "luc/WatershedMarkerSoille.hpp"

#include <string>
#include <chrono>
#include <ctime>
#include <iostream>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#ifdef IS_WIN
#include <direct.h>
#endif

#ifdef  IS_LINUX
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#endif

using namespace cv;
using namespace std;

DistortionManager::DistortionManager()
{
}

//------------------------------------------------------------------------------------------------------------

DistortionManager::DistortionManager(const string path, const DirOrder dir_order, const Mat rgb_src, const uint32_t nbr_distortions, const uint32_t nbr_steps)
{
    m_path = path;
    m_dir_order = dir_order;
    m_rgb_src = rgb_src;
    m_rgb_dst = Mat::zeros(rgb_src.size(), rgb_src.type());
    m_nbr_distortions = nbr_distortions;
    m_nbr_distortion_steps = nbr_steps;

    m_set_id = 0;
    m_step_idx = 0;
    m_distortion_id = 0;

    memset(m_vect_range_min, 0, sizeof(float) * DISTORTION_COUNT);
    memset(m_vect_range_max, 0, sizeof(float) * DISTORTION_COUNT);
    memset(m_vect_strength, 0, sizeof(float) * DISTORTION_COUNT);
    memset(m_vect_step, 0, sizeof(float) * DISTORTION_COUNT);

    memset(m_clip_cnt, 0, sizeof(uint32_t) * 3);
    memset(m_clip_ratio, 0, sizeof(uint32_t) * 3);
}

//------------------------------------------------------------------------------------------------------------

DistortionManager::~DistortionManager()
{

}

//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
// Operating range [0.01..0.06]
// 0.001 = Very low
// 0.006 = Very strong
//
// Speed of convergence (in exponential law)
// 1.0 Linear
// 1.2..1.5 Moderate - Most common setting
// 2.0 Hi - Corresponding to very low focal lenses
//------------------------------------------------------------------------------------------------------------
bool DistortionManager::vignetting(const Mat rgb_src, Mat &rgb_dst, const float strength, const float speed, Mat &mask_out, const float quality)
{  
    Mat ch[3];

    ch[0] = Mat::zeros(rgb_src.size(), CV_8U);
    ch[1] = Mat::zeros(rgb_src.size(), CV_8U);
    ch[2] = Mat::zeros(rgb_src.size(), CV_8U);

    split(rgb_src, ch);

    Mat mask = Mat::ones(rgb_src.size(), CV_32FC1);

    Mat img;
    uint32_t r0 = mask.rows / 2.0;
    uint32_t c0 = mask.cols / 2.0;

    float *p_base_mask = (float *)mask.data;
    float *p_mask = p_base_mask;

    // Range [0.0..1.0]
    float norm_strength = (500.0 * strength) / (r0 * c0);

    // Input parameters are normalized by image size
    // The required precision grows logaritmically
    // with the image size on about 16 on a 10 power 7 size images
    // The smaller is the image, the smaller is the sentivity to the quality
    // Quality is [0..1]

    const float nominal_quality = 0.0001;
    float qual_step = nominal_quality * log(r0 * c0) / (quality + 0.000001);
    float fade = 255.0;

    // Creates the mask
    for (float alpha = 0; alpha < 360.0; alpha += qual_step)
    {
        fade = 255.0;
        uint max_dim = max(mask.rows,  mask.cols) * 2;

        for (uint ray = 0; ray < max_dim; ray++)
        {
            const float r = sin(alpha) * ray;
            const float c = cos(alpha) * ray;

            if (r0 + r > 0 && r0 + r < ch[2].rows && c0 + c > 0 && c0 + c < ch[2].cols)
            {
                uint32_t addr = ((int)(r0 + r) * mask.cols) + (int)(c0 + c);
                p_mask = p_base_mask + addr;
                *p_mask = fade;
            }
            fade -= (pow(ray * norm_strength, 1.05));
        }
    }
    mask.convertTo(mask_out, CV_8UC3);

    // Applies the mask
    float *p_base_img = 0;
    float *p_img = 0;

    for (uint32_t k = 0; k < rgb_src.channels(); k++)
    {
        ch[k].copyTo(img);
        img.convertTo(img, CV_32FC1);

        p_base_img = (float *)img.data;
        p_img = p_base_img;

        p_base_mask = (float *)mask.data;
        p_mask = p_base_mask;

        for (int r = 0; r < img.rows; r++)
        {
            for (int c = 0; c < img.cols; c++)
            {
                uint32_t addr = (r * mask.cols) + c;
                p_img = p_base_img + addr;
                p_mask = p_base_mask + addr;

                *p_img *= (*p_mask / 255.0);
            }
        }
        img.convertTo(ch[k], CV_8UC1);
    }
    merge(ch, 3, rgb_dst);
    return true;
}

//----------------------------------------------------------------------------------------------

bool DistortionManager::testVignettingDistortion(const uint32_t cycles, const uint32_t step)
{
    Mat rgb_src = imread("D:/PRJ/Benchmark/Nominal/mire_50.png", CV_LOAD_IMAGE_COLOR);
    Mat rgb_distort = Mat::zeros(rgb_src.size(), CV_8UC3);
    Mat mask_out = Mat::zeros(rgb_src.size(), CV_8U);

    const float speed = 100;
    const float quality = 1;

    for (uint32_t i = 1; i < cycles; i++)
    {
        float strength = (float)i / (float)cycles;

        if (vignetting(rgb_src, rgb_distort, strength, speed, mask_out, 1.0) == false)
        {
            return false;
        }
        char tmp[255];
        sprintf(tmp, "%d", i);
        imwrite("D:/PRJ/Benchmark/Distorted/TestVignetting/Mask_"  + string(tmp) + ".png", mask_out);
        imwrite("D:/PRJ/Benchmark/Distorted/TestVignetting/vignetting_" + string(tmp) + ".png", rgb_distort);
    }
    return true;
}

//----------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------
// Barrel Pincusion optical distortion
//----------------------------------------------------------------------------------------------
bool DistortionManager::barrel(const Mat rgb_src, Mat &rgb_distort, const float strength, const BarrelMode mode)
{
    Mat ch[3];
    float fade = 255.0;

    ch[0] = Mat::zeros(rgb_src.size(), CV_8U);
    ch[1] = Mat::zeros(rgb_src.size(), CV_8U);
    ch[2] = Mat::zeros(rgb_src.size(), CV_8U);

    split(rgb_src, ch);

    Mat mask;
    Mat img;

    float r_0 = rgb_src.rows / 2.0;
    float c_0 = rgb_src.cols / 2.0;

    float r_1 = 0.0;
    float c_1 = 0.0;

    const float nominal_r = 0.2;
    const float nominal_c = 0.2;
    float k_r = (nominal_r * strength) / (rgb_src.rows * rgb_src.cols);
    float k_c = (nominal_c * strength) / (rgb_src.rows * rgb_src.cols);

    float d_r = 0.0;
    float d_c = 0.0;

    float *p_base_mask = 0;
    float *p_mask = 0;

    float *p_base_img = 0;
    float *p_img = 0;

    if (mode == BARREL_CONCAVE)
    {
        k_r *= -1;
        k_c *= -1;
    }

    for (uint32_t k = 0; k < rgb_src.channels(); k++)
    {
        ch[k].copyTo(img);
        img.convertTo(img, CV_32FC1);

        mask = Mat::zeros(ch[k].rows, ch[k].cols, CV_32FC1);

        p_base_mask = (float *)mask.data;
        p_mask = p_base_mask;

        p_base_img = (float *)img.data;
        p_img = p_base_img;

        for (uint32_t r = 0; r < rgb_src.rows; r++)
        {
            for (uint32_t c = 0; c < rgb_src.cols; c++)
            {
                d_r = r - r_0;
                d_c = c - c_0;

                r_1 = r_0 + d_r * (1 + k_r * ((d_r * d_r) + (d_c * d_c)));
                c_1 = c_0 + d_c * (1 + k_c * ((d_r * d_r) + (d_c * d_c)));

                if (r < mask.rows && c < mask.cols && r_1 >= 0 && c_1 >= 0 && r_1 < mask.rows && c_1 < mask.cols)
                {
                    p_img = p_base_img + ((uint64_t)r_1 * mask.cols) + (uint64_t)c_1;
                    p_mask = p_base_mask + (r * mask.cols) + c;
                    *p_mask = *p_img;
                }
            }
        }
        mask.copyTo(img);
        img.convertTo(img, CV_8UC1);
        img.copyTo(ch[k]);
    }
    merge(ch, 3, rgb_distort);
    return true;
}

//----------------------------------------------------------------------------------------------

bool DistortionManager::testBarrelDistortion(const uint32_t cycles, const uint32_t step)
{
    Mat rgb_src = imread("D:/PRJ/Benchmark/Nominal/mire_50.png", CV_LOAD_IMAGE_COLOR);
    Mat rgb_distort = Mat::zeros(m_rgb_src.size(), CV_8UC3);

    for (uint32_t i = 1; i < cycles; i++)
    {
        float strength = i / (float)cycles;

        if (barrel(m_rgb_src, rgb_distort, strength, BARREL_CONVEX) == false)
        {
            return false;
        }
        char tmp[255];
        sprintf(tmp, "%d", i);
        imwrite("D:/PRJ/Benchmark/Distorted/TestBarrel/barrel_" + string(tmp) + ".png", rgb_distort);
    }
    return true;
}

//------------------------------------------------------------------------------------------------------------

void DistortionManager::setDistortionProperties()
{
    const float photo_attenuation = 1.0;
    const float geom_attenuation = 1.0;

    switch(m_distortion_id)
    {
    case DISTORTION_LUMINANCE:
        m_distort_name = "Luminance";
        m_vect_range_min[DISTORTION_LUMINANCE] = -100.0 * photo_attenuation;
        m_vect_range_max[DISTORTION_LUMINANCE] = 100.0 * photo_attenuation;
        break;

    case DISTORTION_SATURATION:
        m_distort_name = "Saturation";
        m_vect_range_min[DISTORTION_SATURATION] = -1.0 * photo_attenuation;
        m_vect_range_max[DISTORTION_SATURATION] = 1.0 * photo_attenuation;
        break;

    case DISTORTION_HUE:
        m_vect_range_min[DISTORTION_HUE] = -180.0 * photo_attenuation;
        m_vect_range_max[DISTORTION_HUE] = 180.0 * photo_attenuation;
        m_distort_name = "Hue";
        break;

    case DISTORTION_GAUSSIAN_BLUR:
        m_vect_range_min[DISTORTION_GAUSSIAN_BLUR] = 2.0 * photo_attenuation;
        m_vect_range_max[DISTORTION_GAUSSIAN_BLUR] = 12.0 * photo_attenuation;
        m_distort_name = "GaussianBlur";
        break;

    case DISTORTION_MOTION_BLUR:
        m_vect_range_min[DISTORTION_MOTION_BLUR] = 2.0 * photo_attenuation;
        m_vect_range_max[DISTORTION_MOTION_BLUR] = 22.0 * photo_attenuation;
        m_distort_name = "MotionBlur";
        break;

    case DISTORTION_SALT_PEPPER_NOISE:
        m_vect_range_min[DISTORTION_SALT_PEPPER_NOISE] = 0.0 * photo_attenuation;
        m_vect_range_max[DISTORTION_SALT_PEPPER_NOISE] = 1000.0 * photo_attenuation;
        m_distort_name = "SaltAndPepperNoise";
        break;

    case DISTORTION_CAMERA_NOISE:
        m_distort_name = "CameraNoise";
        m_vect_range_min[DISTORTION_CAMERA_NOISE] = 2.0 * photo_attenuation;
        m_vect_range_max[DISTORTION_CAMERA_NOISE] = 22.0 * photo_attenuation;
        break;

    case DISTORTION_SHIFT:
        m_distort_name = "Shift";
        m_vect_range_min[DISTORTION_SHIFT] = 0.0 * geom_attenuation;
        m_vect_range_max[DISTORTION_SHIFT] = m_nbr_distortion_steps * geom_attenuation;
        break;

    case DISTORTION_ROTATION:
        m_distort_name = "Rotation";
        m_vect_range_min[DISTORTION_ROTATION] = -10.0 * geom_attenuation;
        m_vect_range_max[DISTORTION_ROTATION] = 10.0 * geom_attenuation;
        break;

    case DISTORTION_RESCALE:
        m_distort_name = "Rescale";
        m_vect_range_min[DISTORTION_RESCALE] = 1; //* geom_attenuation;
        m_vect_range_max[DISTORTION_RESCALE] = 0.8; //2.0 * geom_attenuation;
        break;

        //    case DISTORTION_PERSPECTIVE:
        //        m_distort_name = "Perspective";
        //        m_vect_range_min[DISTORTION_PERSPECTIVE] = 0.0;
        //        m_vect_range_max[DISTORTION_PERSPECTIVE] = 400.0;
        //        break;

    case DISTORTION_PROJECTION_3D:
        m_distort_name = "Projection3D";
        m_vect_range_min[DISTORTION_PROJECTION_3D] = 0.0 * geom_attenuation;
        m_vect_range_max[DISTORTION_PROJECTION_3D] = 400.0 * geom_attenuation;
        break;

        //    case DISTORTION_MOVING_OBJECT:
        //        m_distort_name = "MovingObject";
        //        m_vect_range_min[DISTORTION_MOVING_OBJECT] = 0.0;
        //        m_vect_range_max[DISTORTION_MOVING_OBJECT] = 20.0;
        //        break;

    case DISTORTION_VIGNETTING:
        m_distort_name = "Vignetting";
        m_vect_range_min[DISTORTION_VIGNETTING] = 0.0 * photo_attenuation;
        m_vect_range_max[DISTORTION_VIGNETTING] = 10.0 * photo_attenuation;
        break;

    case DISTORTION_BARREL:
        m_distort_name = "Barrel";
        m_vect_range_min[DISTORTION_BARREL] = -10.0 * photo_attenuation;
        m_vect_range_max[DISTORTION_BARREL] = 10.0 * photo_attenuation;
        break;

        //    case DISTORTION_SHADOW:
        //        m_distort_name = "Shadow";
        //        m_vect_range_min[DISTORTION_SHADOW] = 0.0;
        //        m_vect_range_max[DISTORTION_SHADOW] = 20.0; *
        //        break;
    }
    m_vect_step[m_distortion_id] = (float)((m_vect_range_max[m_distortion_id] - m_vect_range_min[m_distortion_id]) / m_nbr_distortion_steps);
}

//------------------------------------------------------------------------------------------------------------

bool DistortionManager::applyAllDistortionsToImage(const uint32_t set_id)
{
#ifdef IS_VERBOSE
    cout << "--------------------------------------------------------------------" << endl;
    cout << "Begin of Distortion generation" << endl;
    cout << "--------------------------------------------------------------------" << endl;
#endif
    m_set_id = set_id;
    bool status = true;

    for (uint8_t k = 0; k < m_nbr_distortions; k++)
    {

#ifdef HAS_TIMER
        m_distort_oper_ck_start = clock();
#endif

        m_distortion_id = (Distortions)k;
        if (applyMultistepDistortion() == false)
        {
#ifdef IS_VERBOSE
            cout << "Distortion [" << k << "] failed !!" << endl;
#endif
            return false;
        }

#ifdef HAS_TIMER
        m_distort_oper_ck_stop = clock();
        m_distort_oper_time = diffclock(m_distort_oper_ck_stop, m_distort_oper_ck_start);
        cout << "Elapsed time: " << m_distort_oper_time << endl;
#endif

    }
    cout << "--------------------------------------------------------------------" << endl;
    cout << "End of Distortion generation" << endl;
    cout << "--------------------------------------------------------------------" << endl << endl;
    return status;
}

//------------------------------------------------------------------------------------------------------------

void DistortionManager::rotate3D(const Mat &input, Mat &output, float alpha, float beta, float gamma, const float dx, const float dy, const float dz, const float f)
{
    alpha = (alpha - 90.)*CV_PI/180.;
    beta = (beta - 90.)*CV_PI/180.;
    gamma = (gamma - 90.)*CV_PI/180.;
    // get width and height for ease of use in matrices
    float w = (float)input.cols;
    float h = (float)input.rows;

    // Projection 2D -> 3D matrix
    Mat A1 = (Mat_<float>(4,3) <<
              1, 0, -w/2,
              0, 1, -h/2,
              0, 0,    0,
              0, 0,    1);

    // Rotation matrices around the X, Y, and Z axis
    Mat RX = (Mat_<float>(4, 4) <<
              1,          0,           0, 0,
              0, cos(alpha), -sin(alpha), 0,
              0, sin(alpha),  cos(alpha), 0,
              0,          0,           0, 1);
    Mat RY = (Mat_<float>(4, 4) <<
              cos(beta), 0, -sin(beta), 0,
              0, 1,          0, 0,
              sin(beta), 0,  cos(beta), 0,
              0, 0,          0, 1);
    Mat RZ = (Mat_<float>(4, 4) <<
              cos(gamma), -sin(gamma), 0, 0,
              sin(gamma),  cos(gamma), 0, 0,
              0,          0,           1, 0,
              0,          0,           0, 1);

    // Composed rotation matrix with (RX, RY, RZ)
    Mat R = RX * RY * RZ;

    // Translation matrix
    Mat T = (Mat_<float>(4, 4) <<
             1, 0, 0, dx,
             0, 1, 0, dy,
             0, 0, 1, dz,
             0, 0, 0, 1);

    // 3D -> 2D matrix
    Mat A2 = (Mat_<float>(3,4) <<
              f, 0, w/2, 0,
              0, f, h/2, 0,
              0, 0,   1, 0);

    // Final transformation matrix
    Mat mat_warp = A2 * (T * (R * A1));

    // Apply matrix transformation
    warpPerspective(input, output, mat_warp, input.size(), BORDER_DEFAULT); //INTER_LANCZOS4);
}

//------------------------------------------------------------------------------------------------------------

void DistortionManager::perspective()
{
    Mat src_float = Mat::zeros(m_rgb_src.size(), m_rgb_src.type());
    Mat dst_float = Mat::zeros(m_rgb_src.size(), m_rgb_src.type());
    m_rgb_src.convertTo(src_float, CV_32FC3);

    vector<Point> not_a_rect_shape;
    const Point offset(0, 0);

    not_a_rect_shape.push_back(Point(offset));                                                              // Top-Left edge
    not_a_rect_shape.push_back(Point(offset.x, src_float.rows - offset.y - 1));                             // Top-Right edge
    not_a_rect_shape.push_back(Point(src_float.cols - offset.x - 1, src_float.rows - offset.y - 1));        // Bottom-Left edge
    not_a_rect_shape.push_back(Point(src_float.cols - offset.x - 1, offset.y));                             // Botom-Right edge

    // For debugging purposes, draw green lines connecting those points
    // and save it on disk
    const Point* point = &not_a_rect_shape[0];
    int n = (int)not_a_rect_shape.size();
    Mat draw = src_float.clone();
    polylines(draw, &point, &n, 1, true, Scalar(0, 255, 0), 3, CV_AA);
    // imwrite("D:/PRJ/Benchmark/Distorted/draw.png", draw);

    Point2f src_vertices[4];
    src_vertices[0] = not_a_rect_shape[0];
    src_vertices[1] = not_a_rect_shape[1];
    src_vertices[2] = not_a_rect_shape[2];
    src_vertices[3] = not_a_rect_shape[3];

    // Assemble a rotated rectangle out of that info
    RotatedRect box = minAreaRect(Mat(not_a_rect_shape));
#ifdef IS_VERBOSE
    cout << "Rotated box set to (" << box.boundingRect().x << "," << box.boundingRect().y << ") " << box.size.width << "x" << box.size.height << endl;
#endif

    Point2f dst_vertices[4];
    src_vertices[0] = not_a_rect_shape[0] + Point(800, 800);
    src_vertices[1] = not_a_rect_shape[1] + Point(800, 800);
    src_vertices[2] = not_a_rect_shape[2] + Point(800, 800);
    src_vertices[3] = not_a_rect_shape[3] + Point(800, 800);

    Mat warpMatrix = getPerspectiveTransform(src_vertices, dst_vertices);

    Mat rotated;
    warpPerspective(src_float, dst_float, warpMatrix, src_float.size(), INTER_CUBIC, BORDER_CONSTANT);
    dst_float.convertTo(m_rgb_dst, CV_8UC3);
}

//------------------------------------------------------------------------------------------------------------

bool DistortionManager::imageResize(const float size_ratio)
{
    // m_rgb_src.copyTo(m_rgb_dst);
    resize(m_rgb_src, m_rgb_dst, Size(m_rgb_src.cols * size_ratio, m_rgb_src.rows * size_ratio), INTER_CUBIC);
    return true;
}

//------------------------------------------------------------------------------------------------------------

bool DistortionManager::applyMultistepDistortion()
{
    setDistortionProperties();

#ifdef IS_VERBOSE
    cout << "------------------------------------------------------------------------------" << endl;
    cout << "Begin Set [" << m_set_id << "] - Test " << m_distort_name << endl;
    cout << "------------------------------------------------------------------------------" << endl;
#endif

    srand(time(NULL));
    bool res = true;

    for (uint32_t k = 0; k < m_nbr_distortion_steps; k++)
    {
        res &= applyDistortionSteps(k);
    }
    imwrite(setFileName2(m_distort_name, "Img_").data(), m_rgb_dst);

#ifdef IS_VERBOSE
    cout << "------------------------------------------------------------------------------" << endl;
    cout << "End Set [" << m_set_id << "] - Test " << m_distort_name << endl;
    cout << "------------------------------------------------------------------------------" << endl;
#endif
    return res;
}

//------------------------------------------------------------------------------------------------------------

bool DistortionManager::applyDistortionSteps(const int k)
{
    //-----------------------------------------------------------------------------------------------------
    // Unit Test Section
    //-----------------------------------------------------------------------------------------------------
    //    cout << "------------------------------------------------------------------------------" << endl;
    //    setDistortionProperties();

    //    cout << "------------------------------------------------------------------------------" << endl;
    //    cout << "Begin Set [" << m_set_id << "] - Test " << m_distort_name << endl;
    //    cout << "------------------------------------------------------------------------------" << endl;

    //    // m_rgb_src = imread("D:/PRJ/Benchmark/Nominal/mire_50.png", CV_LOAD_IMAGE_COLOR);
    //    // m_distortion_id = DISTORTION_PROJECTION_3D;

    //    srand(time(NULL));


    //    for (uint32_t k = 0; k < m_nbr_distortion_steps; k++)
    //    {

    m_step_idx = k;
    // cout << "  " << m_distort_name << " Step " << k << "/" << m_nbr_distortion_steps << endl;

    switch (m_distortion_id)
    {
    case DISTORTION_LUMINANCE:
    {
        const float strength = 0.8;
        // Input values are not normalized
        HSVManager hsv_man(m_vect_range_min[m_distortion_id], m_vect_range_max[m_distortion_id], m_vect_step[m_distortion_id]);
        hsv_man.modifyLuminance(m_rgb_src, m_rgb_dst, m_vect_range_min[m_distortion_id] + (m_vect_step[m_distortion_id] * k * strength));
    }
        break;

    case DISTORTION_SATURATION:
    {
        const float strength = 0.8;
        // Input values are not normalized
        HSVManager hsv_man(m_vect_range_min[m_distortion_id], m_vect_range_max[m_distortion_id], m_vect_step[m_distortion_id]);
        hsv_man.modifySaturation(m_rgb_src, m_rgb_dst, m_vect_range_min[m_distortion_id] + (m_vect_step[m_distortion_id] * k * strength));

        //        m_clip_cnt[0] += hsv_man.m_clip_cnt[0];
        //        m_clip_cnt[1] += hsv_man.m_clip_cnt[1];
        //        m_clip_cnt[2] += hsv_man.m_clip_cnt[2];
    }
        break;

    case DISTORTION_HUE:
    {
        // Input values are not normalized
        HSVManager hsv_man(m_vect_range_min[m_distortion_id], m_vect_range_max[m_distortion_id], m_vect_step[m_distortion_id] * k);
        hsv_man.modifyHue(m_rgb_src,m_rgb_dst, k);
    }
        break;

    case DISTORTION_GAUSSIAN_BLUR:
    {
        // Applying Gaussian blur
        // Only squared kernels are applied to void moments on the PCF
        int kernel_r = ((m_vect_step[m_distortion_id] + k)); // - 1;
        int kernel_c = ((m_vect_step[m_distortion_id] + k)); // - 1;

        if (kernel_r%2 == 0)
        {
            kernel_r--;
        }

        if (kernel_c%2 == 0)
        {
            kernel_c--;
        }
        GaussianBlur(m_rgb_src, m_rgb_dst, Size(kernel_r, kernel_c), 0, 0);
    }
        break;

    case DISTORTION_MOTION_BLUR:
    {
        Mat kernel(3 + k, 3 + k, CV_32F);
        kernel.setTo(0);
        if (k > 0)
        {
            kernel.at<float>(1, 1) = 0.2;  //kernel.rows/2, kernel.cols/2) = 1;
            kernel.at<float>(1, 1 + k) = 0.8;

            // Initialize arguments for the filter
            Point anchor = Point( -1, -1);
            int delta = 0;
            int depth = -1;

            // The filter2d performs the correlation instead of the convolution.
            filter2D(m_rgb_src, m_rgb_dst, depth , kernel, anchor, delta, BORDER_DEFAULT);
        }
        else
        {
            m_rgb_src.copyTo(m_rgb_dst);
        }
        break;
    }

    case DISTORTION_SALT_PEPPER_NOISE:
    {
        const float strength = 0.001;
        m_rgb_src.copyTo(m_rgb_dst);
        Mat saltpepper_noise = Mat::zeros(m_rgb_src.size(), CV_32F);
        randu(saltpepper_noise, 0, 255);

        Mat black = saltpepper_noise < (k * strength * m_vect_step[m_distortion_id]);
        Mat white = saltpepper_noise > 255 - (k * strength * m_vect_step[m_distortion_id]);

        m_rgb_dst.setTo(255, white);
        m_rgb_dst.setTo(0, black);
        break;
    }

    case DISTORTION_CAMERA_NOISE:
    {
        // This test use a random generator variable, the results are stockastic and the test cannot be reproduced with the same results
        Mat ch[3];

        ch[0] = Mat::zeros(m_rgb_src.size(), CV_8U);
        ch[1] = Mat::zeros(m_rgb_src.size(), CV_8U);
        ch[2] = Mat::zeros(m_rgb_src.size(), CV_8U);

        cv::split(m_rgb_src, ch);
        // Gaussian noise is the Johnson–Nyquist noise thermal noise
        // This is an additive pixel independent noise
        // This noise is more frequent on the blue channel, because color cameras are more sensitive to blue channel

        // out = in/255.0;
        Mat noise = Mat(m_rgb_src.size(),CV_32F);
        Mat result = Mat(m_rgb_src.size(),CV_32F);
        result.setTo(0);

        ch[0].convertTo(ch[0], CV_32F);
        ch[1].convertTo(ch[1], CV_32F);
        ch[2].convertTo(ch[2], CV_32F);

        // normalize(in, result, 0.0, 1.0, CV_MINMAX, CV_32F);
        randn(noise, 0, k); //0.001); // 0.05);

        ch[0] += noise;
        ch[1] += noise;
        ch[2] += noise;

        ch[0].convertTo(ch[0], CV_8U);
        ch[1].convertTo(ch[1], CV_8U);
        ch[2].convertTo(ch[2], CV_8U);

        merge(ch, 3, m_rgb_dst);
        //  normalize(out, out, 0.0, 1.0, CV_MINMAX, CV_32F);
        break;
    }

    case DISTORTION_SHIFT:
    {
        const bool dir_v = 1;
        const bool dir_h = 1;

        const int step = (k + 1) * m_vect_step[m_distortion_id];
        Mat kernel((3 + step), (3 + step), CV_32F);
        kernel.setTo(0);
        kernel.at<float>(1, 1) = 0.0;
        kernel.at<float>(1 + (dir_v * step), 1 + (dir_h * step)) = 1.0;

        // Initialize arguments for the filter
        Point anchor = Point( -1, -1);
        int delta = 0;
        int depth = -1;

        // The filter2d performs the correlation instead of the convolution.
        filter2D(m_rgb_src, m_rgb_dst, depth , kernel, anchor, delta, BORDER_DEFAULT);
    }
        break;

    case DISTORTION_ROTATION:
    {
        Mat img_src;
        Mat img_dst;

        m_rgb_src.convertTo(img_src, CV_32FC3);

        // Point center m_rgb_src = Point(0, 0);
        Point2f center(img_src.cols / 2.0, img_src.rows / 2.0);
        float angle = m_vect_step[m_distortion_id] * k;
        float scale = 1.0;

        // Get the rotation matrix with the specifications above
        Mat warp_mat = getRotationMatrix2D(center, angle, scale);

        // determine bounding rectangle
        // Rect box = RotatedRect(center, tmp.size(), angle).boundingRect();
        // adjust transformation matrix
        // rot_mat.at<float>(0,2) += bbox.width/2.0 - center.x;
        // rot_mat.at<float>(1,2) += bbox.height/2.0 - center.y;

        // Rotate the warped image
        warpAffine(img_src, img_dst, warp_mat, img_src.size(), INTER_CUBIC);

        // Selected inner canvas
        //  const int edge_r = img_src.rows/4 - (img_src.rows/4 * sin(angle/1));
        //  const int edge_c = img_src.cols/4 - (img_src.cols/4 * cos(angle/1));
        //  Mat rot_roi = img_dst(Rect(edge_c/2, edge_r/2, img_dst.cols - edge_c, img_dst.rows - edge_r));
        //  rot_roi.copyTo(dist_man_2.m_rgb_dst);
        //  imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/rot_1.png", rot_roi);
        //  imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/rot_crop_1.png", img_dst);

        img_dst.convertTo(m_rgb_dst, CV_32FC3);
    }
        break;

    case DISTORTION_RESCALE:
        imageResize(m_vect_range_min[DISTORTION_RESCALE] + m_vect_step[m_distortion_id] * k);
        break;

    case DISTORTION_PROJECTION_3D:
    {
        const float alpha = 90.0 - ((k * 100) / m_vect_step[m_distortion_id]);
        const float beta = 90.0 - ((k * 00) / m_vect_step[m_distortion_id]);
        const float gamma = 90.0 - ((k * 0) / m_vect_step[m_distortion_id]);
        const float dx = 0.0 - (k * 0.0);
        const float dy = 0.0 - (k * 0.0);
        const uint64_t n = (m_rgb_src.rows * m_rgb_src.cols);
        float val = (n / 500) - (k * 8.0);
        const float dz = 520.0 + (k * 1.0); //val; //(n / 400) - (k * 8.0);
        const float f = 520.0;

        Mat src_float = Mat::zeros(m_rgb_src.size(), m_rgb_src.type());
        Mat dst_float = Mat::zeros(m_rgb_src.size(), m_rgb_src.type());
        m_rgb_src.convertTo(src_float, CV_32FC3);
        rotate3D(src_float, dst_float, alpha, beta, gamma, dx, dy, dz, f);
        dst_float.convertTo(m_rgb_dst, CV_8UC3);

    }
        break;

        //        case DISTORTION_PERSPECTIVE:
        //        {
        //            perspective();
        //        }
        //            break;

        //        case DISTORTION_MOVING_OBJECT:
        //            break;

    case DISTORTION_VIGNETTING:
    {
        Mat mask_out;
        float strength = ((float)m_vect_step[m_distortion_id] * k) / (float)m_nbr_distortion_steps;
strength *= 1.8;

        if (vignetting(m_rgb_src, m_rgb_dst, strength, 1.0, mask_out, 0.1) == false)
        {
            return false;
        }
    }
        break;

    case DISTORTION_BARREL:
    {
        const float strength = ((float)m_vect_step[m_distortion_id] * k) / (float)m_nbr_distortion_steps;

        if (barrel(m_rgb_src, m_rgb_dst, strength, BARREL_CONCAVE) == false)
        {
            return false;
        }
    }
        break;

        //        case DISTORTION_SHADOW:
        //            break;

    default:
        break;
    }
    return true;
}

//------------------------------------------------------------------------------------------------------------

string DistortionManager::setFileName(const string file_name)
{
    int ret = -1;
    char s_set[255];
    sprintf(s_set, "%04d", m_set_id);

    char s_step[255];
    sprintf(s_step, "%04d", m_step_idx);

    string new_path = "";

    if (m_dir_order == SET_TO_OPERATION)
    {
        new_path = m_path + file_name;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }

        new_path = new_path + "/" + string("Set_") + s_set;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }
    }
    else // OPERATION_TO_SET
    {
        new_path = m_path + string("Set_") + s_set;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }

        new_path = new_path + "/" + file_name;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }
    }

#ifdef IS_WIN
    ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
    ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

    if (ret != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
    }
    return new_path + "/" + "Img_" + s_set + "_" + s_step + ".png";
}

//------------------------------------------------------------------------------------------------

string DistortionManager::setFileName2(const string path_name, const string file_name)
{
    int ret = -1;
    char s_set[255];
    sprintf(s_set, "%04d", m_set_id);

    char s_step[255];
    sprintf(s_step, "%04d", m_step_idx);
    string new_path = "";

    if (m_dir_order == OPERATION_TO_SET)
    {
        new_path = m_path + path_name;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }

        new_path = new_path + "/" + string("Set_") + s_set;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }
    }
    else // SET_TO_OPERATION
    {
        new_path = m_path + string("Set_") + s_set;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }

        new_path = new_path + "/" + path_name;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }
    }

#ifdef IS_WIN
    ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
    ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

    if (ret != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
    }
    return new_path + "/" + file_name + s_set + "_" + s_step + ".png";
}

//------------------------------------------------------------------------------------------------

string DistortionManager::get_distort_name(const int distort_id)
{
    switch(distort_id)
    {
    case DISTORTION_LUMINANCE:
        return "Luminance";

    case DISTORTION_SATURATION:
        return "Saturation";

    case DISTORTION_HUE:
        return "Hue";

    case DISTORTION_GAUSSIAN_BLUR:
        return "GaussianBlur";

    case DISTORTION_MOTION_BLUR:
        return "MotionBlur";

    case DISTORTION_SALT_PEPPER_NOISE:
        return "SaltAndPepperNoise";

    case DISTORTION_CAMERA_NOISE:
        return "CameraNoise";

    case DISTORTION_SHIFT:
        return "Shift";

    case DISTORTION_ROTATION:
        return "Rotation";

    case DISTORTION_RESCALE:
        return "Rescale";

        //    case DISTORTION_PERSPECTIVE:
        //        return "Perspective";

    case DISTORTION_PROJECTION_3D:
        return "Projection3D";

        //    case DISTORTION_MOVING_OBJECT:
        //        return "MovingObject";

    case DISTORTION_VIGNETTING:
        return "Vignetting";

    case DISTORTION_BARREL:
        return "Barrel";

        //    case DISTORTION_SHADOW:
        //        return "Shadow";
    }
}
