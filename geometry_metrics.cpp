////-------------------------------------------------------------------------------------
//// Project name: G MOSAICA
////
//// Creation Date: 20/03/2016
//// Created by: Roberto Giudici - OBELIX, UBS, IRISA, COSTEL, Universiy of Rennes 2
//// Description: Super pFxel time coherent based Mosaicing algorithm
////-------------------------------------------------------------------------------------

//#include "geometry_metrics.h"
//#include "spixtools.h"
//#include "mosaicatools.h"
//#include "robustmatcher.h"
//// #include "gnuplotexport.h"
//// #include "registration.h"
//#include "tools.h"
//#include <queue>
//#include <stack>
//#include <algorithm>

//#ifdef IS_WIN
//#include <direct.h>
//#endif

//#include <stdio.h>      /* printf, scanf, puts, NULL */
//#include <stdlib.h>     /* srand, rand */
//#include <time.h>       /* time */
//#include <iostream>
//#include <fstream>
//#include <string>

//#include "opencv2/opencv_modules.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/stitching/detail/autocalib.hpp"
//#include "opencv2/stitching/detail/blenders.hpp"
//#include "opencv2/stitching/detail/camera.hpp"
//#include "opencv2/stitching/detail/exposure_compensate.hpp"
//#include "opencv2/stitching/detail/matchers.hpp"
//#include "opencv2/stitching/detail/motion_estimators.hpp"
//#include "opencv2/stitching/detail/seam_finders.hpp"
//#include "opencv2/stitching/detail/util.hpp"
//#include "opencv2/stitching/detail/warpers.hpp"
//#include "opencv2/stitching/warpers.hpp"

//#define CmpErr 2.0

//using namespace std;
//using namespace cv;
//using namespace cv::detail;

//bool CompareMaxSGFAscend(const MaxSGF& firstElem, const MaxSGF& secondElem);
//bool CompareMaxSGFAscend(const MaxSGF& firstElem, const MaxSGF& secondElem)
//{
//    return firstElem.m_sgf > secondElem.m_sgf;
//}

////------------------------------------------------------------------------------

//bool pairCompareFloatAscend(const pair<int, int>& firstElem, const pair<int, int>& secondElem);
//bool pairCompareFloatAscend(const pair<int, int>& firstElem, const pair<int, int>& secondElem)
//{
//    return firstElem.second < secondElem.second;
//}

////------------------------------------------------------------------------------

//bool pairCompareFloatDiscend(const pair<int, float>& firstElem, const pair<int, float>& secondElem);
//bool pairCompareFloatDiscend(const pair<int, float>& firstElem, const pair<int, float>& secondElem)
//{
//    return firstElem.second > secondElem.second;
//}
////-----------------------------------------------------------------------------

//bool keyPointCompareSize(const KeyPoint &firstElem, const KeyPoint &secondElem);
//bool keyPointCompareSize(const KeyPoint &firstElem, const KeyPoint &secondElem)
//{
//    return firstElem.size > secondElem.size;
//}

//bool keyPointCompareResponse(const KeyPoint &firstElem, const KeyPoint &secondElem);
//bool keyPointCompareResponse(const KeyPoint &firstElem, const KeyPoint &secondElem)
//{
//    return firstElem.response > secondElem.response;
//}

////-----------------------------------------------------------------------------

//GeometryMetrics::GeometryMetrics(const string path)
//{
//    m_path = path;
//}


////-----------------------------------------------------------------------------
//// Begin Block of Hue detection managment
////-----------------------------------------------------------------------------

//bool GeometryMetrics::createHueHistogram(const int simg_idx, const int spix_idx, Mat src, Mat &hue_histogram)
//{
//    const Size src_size = src.size();
//    Mat vct_channel[3];
//    vct_channel[0] = Mat::zeros(src_size, CV_32FC1);
//    vct_channel[1] = Mat::zeros(src_size, CV_32FC1);
//    vct_channel[2] = Mat::zeros(src_size, CV_32FC1);

//    // Color conversion RGB to HSV
//    Mat hsv = Mat::zeros(src_size, CV_32FC3);
//    cvtColor(src, hsv, COLOR_BGR2HSV);
//    split(hsv, vct_channel);

//    // One Dimension integer Matrix
//    hue_histogram = Mat::zeros(1, 360, CV_32SC1);

//    // Creates a sPix matrix frm the sPix position vector
//    // SuperPixel *p_spix_0 = &(m_vct_simg[simg_idx].m_vct_spix[spix_idx]);

//    for (int r = 0; r < src.rows; r++)
//    {
//        for (int c = 0; c < src.cols; c++)
//        {
//            float hue_val = vct_channel[0].at<int8_t>(r, c) * 2.0;

//            if (hue_val < 0)
//            {
//                hue_val = 180.0 + hue_val;
//            }
//            hue_histogram.at<int>(0, (uint8_t)hue_val)++;
//        }
//    }
//    return true;
//}

////-----------------------------------------------------------------------------

//bool GeometryMetrics::createHueHistogram(const int simg_idx, const int spix_idx, Mat &hue_histogram)
//{
//    const Size src_size = m_vct_rgb_src[simg_idx].size();
//    Mat vct_channel[3];
//    vct_channel[0] = Mat::zeros(src_size, CV_32FC1);
//    vct_channel[1] = Mat::zeros(src_size, CV_32FC1);
//    vct_channel[2] = Mat::zeros(src_size, CV_32FC1);

//    // Color conversion RGB to HSV
//    Mat hsv = Mat::zeros(src_size, CV_32FC3);
//    cvtColor(m_vct_rgb_src[simg_idx], hsv, COLOR_BGR2HSV);
//    split(hsv, vct_channel);

//    // One Dimension integer Matrix
//    hue_histogram = Mat::zeros(1, 360, CV_32SC1);

//    // Creates a sPix matrix frm the sPix position vector
//    SuperPixel *p_spix_0 = &(m_vct_simg[simg_idx].m_vct_spix[spix_idx]);

//    for (int i = 0; i < p_spix_0->m_spix_pos.size(); i++)
//    {
//        const int r = p_spix_0->m_spix_pos[i].m_r;
//        const int c = p_spix_0->m_spix_pos[i].m_c;

//        float hue_val = vct_channel[0].at<int8_t>(r, c) * 2.0;

//        if (hue_val < 0)
//        {
//            hue_val = 256 + (256 + hue_val);
//        }
//        hue_histogram.at<int>(0, (uint8_t)hue_val)++;
//        // int val = hue_histogram.at<int>(0, (uint8_t)hue_val);
//        // cout << "spixIdx: " << i << " - Bean: " << hue_val << "  - Val: " << val << endl;
//    }
//    return true;
//}
//// Template matching : Hue convolutional
//// Alternative : SAD (Sum of Absolute Differences)
//bool GeometryMetrics::SPixHueMatching(const int simg_idx_0, const int simg_idx_1, const int spix_idx_0, const int spix_idx_1, float &hue_similarity){
//    // Mat src_0 = imread("D:/PRJ/ALGO_SLIC/CONVOLUTION/img_2.bmp");
//    // Mat src_1 = imread("D:/PRJ/ALGO_SLIC/CONVOLUTION/img_3.bmp");

//    const int rows = 1;
//    const int cols = 360;

//    Mat hist_0 = Mat::zeros(rows, cols, CV_32FC1);
//    Mat hist_1 = Mat::zeros(rows, cols, CV_32FC1);

//    createHueHistogram(simg_idx_0, spix_idx_0, hist_0);
//    createHueHistogram(simg_idx_1, spix_idx_1, hist_1);

//    hist_0.convertTo(hist_0, CV_32F);
//    hist_1.convertTo(hist_1, CV_32F);

//    // const float alpha = 180.0;
//    const int kern_width = 31;

//    //  for (int i = 0; i < kern_width; i++)
//    // {
//    //   int sz = src_0.rows * src_0.cols;
//    //    hist_0.at<float>(0, alpha) = sz;
//    //   int j = alpha - (float)kern_width/2.0 + i;
//    //    hist_1.at<float>(0, j) = sz;

//    normalize(hist_0, hist_0, 0, 1.0, NORM_MINMAX, CV_32FC1);
//    normalize(hist_1, hist_1, 0, 1.0, NORM_MINMAX, CV_32FC1);

//    double min_val, max_val;
//    Point min_idx, max_idx;
//    minMaxLoc(hist_0, &min_val, &max_val, &min_idx, &max_idx);

//    const Mat sub_hist = hist_1(Rect(max_idx.x - (kern_width/2), 0, kern_width, 1));
//    sub_hist.copyTo(sub_hist);

//    // The kernel vector must be a column vector as required by OpenCv
//    Mat kernel = Mat::zeros(kern_width, 1, CV_32F);
//    kernel = getGaussianKernel(kern_width, 2.0, CV_32F);
//    normalize(kernel, kernel, 0, 1.0, NORM_MINMAX, CV_32FC1);
//    Mat mat_prd = sub_hist * kernel;
//    hue_similarity = mat_prd.at<float>(0, 0);

//    //     cout << "Hue: " << j << " - Product: " << prd << endl;
//    // }
//    return true;
//}

//bool GeometryMetrics::RgbToHue(const int simg_idx, Mat &hue_dst)
//{
//    Mat vct_channel[3];

//    // Color conversion RGB to HSV
//    Mat hsv;
//    cvtColor(m_vct_rgb_src[simg_idx], hsv, COLOR_BGR2HSV);
//    split(hsv, vct_channel);
//    vct_channel[0].copyTo(hue_dst);
//    hue_dst.convertTo(hue_dst, CV_32F);
//    return true;
//}

//bool GeometryMetrics::SPixHueSAD(const int simg_idx_0, const int spix_idx_0, const int simg_idx_1, const int spix_idx_1, float &hue_sad)
//{
//    Mat hue_dst_0;
//    RgbToHue(simg_idx_0, hue_dst_0);

//    // Mat vct_channel[3];

//    // int size_x = shared_max_x - shared_min_x;
//    // int size_y = shared_max_y - shared_min_y;
//    // const Size src_size(size_x, size_y);

//    //  vct_channel[0] = Mat::zeros(size_x, size_y, CV_32FC1);
//    //  vct_channel[1] = Mat::zeros(size_x, size_y, CV_32FC1);
//    //  vct_channel[2] = Mat::zeros(size_x, size_y, CV_32FC1);

//    // Color conversion RGB to HSV
//    // Mat hsv = Mat::zeros(size_x, size_y, CV_32FC3);
//    // cvtColor(m_vct_rgb_src[simg_idx_0](Rect(min_y_0, min_x_0, size_y, size_x)), hsv, COLOR_BGR2HSV);
//    // split(hsv, vct_channel);

//    SuperPixel *p_spix_0 = &(m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0]);
//    int rgb_sum_0 = 0;

//    // vct_channel[0].convertTo(vct_channel[0], CV_32F);

//    int sz_0 = p_spix_0->m_spix_pos.size();
//    int nbr_cmp_px = sz_0;

//    for (int px_idx = 0; px_idx < sz_0; px_idx++)
//    {
//        const int r = p_spix_0->m_spix_pos[px_idx].m_r;
//        const int c = p_spix_0->m_spix_pos[px_idx].m_c;

//        //  const int r1 = r - shared_min_x - 1;
//        //  const int c1 = c - shared_min_y - 1;

//        //  if (r1 >= 0 && c1 >= 0 && r1 < size_x && c1 < size_y)
//        {
//            rgb_sum_0 += hue_dst_0.at<float>(r, c);
//        }

//    }
//    float rgb_mean_0 = rgb_sum_0 / nbr_cmp_px;

//    Mat hue_dst_1;
//    RgbToHue(simg_idx_1, hue_dst_1);

//    SuperPixel *p_spix_1 = &(m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1]);
//    int rgb_sum_1 = 0;

//    const int sz_1 = p_spix_1->m_spix_pos.size();
//    nbr_cmp_px = sz_1;

//    for (int px_idx = 0; px_idx < sz_1; px_idx++)
//    {
//        const int r = p_spix_1->m_spix_pos[px_idx].m_r;
//        const int c = p_spix_1->m_spix_pos[px_idx].m_c;

//        //   const int r1 = r - shared_min_x - 1;
//        //   const int c1 = c - shared_min_y - 1;

//        //if (r1 >= 0 && c1 >= 0 && r1 < size_x && c1 < size_y)
//        {
//            rgb_sum_1 += (hue_dst_1.at<float>(r, c));
//        }
//    }
//    float rgb_mean_1 = rgb_sum_1 / nbr_cmp_px;

//    hue_sad = abs(rgb_mean_1 - rgb_mean_0);
//    const int epsilon = 1;

//    if (abs(hue_sad) > epsilon)
//    {
//        return false;
//    }
//    return true;
//}
////-----------------------------------------------------------------------------
//// End Block of Hue detection managment
////-----------------------------------------------------------------------------

//float GeometryMetrics::compute_keypoint_weight(const KeyPoint &keyp,
//                                               const float resp_weight,
//                                               const float size_weight,
//                                               const float min_response,
//                                               const float max_response,
//                                               const float min_size,
//                                               const float max_size)
//{
//    const float range_ratio = (max_response - min_response) / (max_size - min_size);

//    const float size_norm = (((keyp.size - min_size) / (max_size - min_size)));
//    const float response_norm = (((keyp.response - min_response) / (max_response - min_response)));
//    //   float aa = (response_norm * resp_weight) + (size_norm * size_weight);

//    return (response_norm * resp_weight) + (size_norm * size_weight);
//}


////-----------------------------------------------------------------------------
//// Test: Performes the test
////-----------------------------------------------------------------------------
//void GeometryMetrics::unit_test(const Mat rgb_src_0,
//                                const Mat rgb_src_1,
//                                const Mat rgb_src_mosaic_1,
//                                const Mat rgb_src_mosaic_2)
//{
//    const float gain = 1.0;                         // Sensitivity of the funtion to small errors

//    // reach2prec = 0.0 -> pure precision
//    // reach2prec = 1.0 -> pure reachable
//    const float reach_to_prec = 1.0;                // Reachibility to Precision [0..1]
//    const float nbr_robust_pts = 7;                 // Number of maximum robust keypoints used. [0..1]

//    // imshow("image0", image0);
//    // imshow("image1", image1);

//    ofstream *log_file = new ofstream();
//    log_file->open (m_path + "log/geo_metric.txt");

//    *log_file << "------------------------------------------------------------"  << endl;
//    *log_file << "Begin of Geometry Metrics Benchmark" << endl;
//    *log_file << "------------------------------------------------------------"  << endl;

//    // x -> rows, y -> columns
//    //m_img_shift.x = 0;
//    //m_img_shift.y = -57;

//    // 2. Computes the distance error on the distort mosaic

//    //-----------------------------------------------
//    // Computes the metric with the nominal image
//    //-----------------------------------------------
//    m_vct_rgb_src.push_back(rgb_src_0);
//    m_vct_rgb_src.push_back(rgb_src_1);
//    m_vct_rgb_src.push_back(rgb_src_mosaic_1);
//    const float sgf_distort_nom = stitch_geometry_fidelity(gain, reach_to_prec, nbr_robust_pts);

//    m_vct_rgb_src.clear();

//    //-----------------------------------------------
//    // Computes the metric with the distorted image
//    //-----------------------------------------------
//    m_vct_rgb_src.push_back(rgb_src_0);
//    m_vct_rgb_src.push_back(rgb_src_1);
//    m_vct_rgb_src.push_back(rgb_src_mosaic_2);
//    const float sgf_distort_test = stitch_geometry_fidelity(gain, reach_to_prec, nbr_robust_pts);
//    const float biasless_sgf = sgf_distort_test - sgf_distort_nom;

//    exit(0);
//}

////-----------------------------------------------------------------------------
//// Test: Performes the test
////-----------------------------------------------------------------------------
//void GeometryMetrics::optimal_test(const Mat rgb_src_0, const Mat rgb_src_1, const Mat rgb_src_mosaic_1, const Mat rgb_src_mosaic_2)
//{
//    const float gain = 1.0;                         // Sensitivity of the funtion to small errors

//    // reach2prec = 0.0 -> pure precision
//    // reach2prec = 1.0 -> pure reachable
//    const float reach_to_prec = 0.1;                // Reachibility to Precision [0..1]
//    const float nbr_robust_pts = 1;                 // Number of maximum robust keypoints used. [0..1]

//    // Read first image
//    // const Mat rgb_src_0 = imread(m_path + string("p0.png"));

//    // Read Second Image
//    // const Mat rgb_src_1 = imread(m_path + string("p1_58.png"));

//    // Read mosaic Image 1
//    //  const Mat rgb_src_mosaic_1 = imread(m_path + string("mosaic_nominal.png"));

//    // Read mosaic Image 2
//    // const Mat rgb_src_mosaic_2 = imread(m_path + string("mosaic_distort_64_64_multi.png"));

//    // imshow("image0", image0);
//    // imshow("image1", image1);

//    //    const int nbr_spix = 8;
//    //    const double compactness = 20.0;

//    // x -> rows, y -> columns
//    //  m_img_shift.x = 0;
//    //  m_img_shift.y = -57;

//    const int r2p_steps = 1.0 / reach_to_prec;
//    long start_all, end_all;
//    long start, end;

//    // Statistical SGF distribution variables
//    float sum_r2p  = 0.0;                         // Sum of all values of SGF for a selected value of R2P
//    float mean_r2p  = 0.0;                        // Mean of all values of SGF for a selected value of R2P
//    float min_sgf_r2p = 0.0;                      // Min SGF for a selected value of R2P
//    float max_sgf_r2p = 0.0;                      // Max SGF for a selected value of R2P

//    vector<MaxSGF> vct_max_sgf;

//    ofstream *log_file = new ofstream();
//    log_file->open (m_path + "log/geo_metric.txt");

//    *log_file << "------------------------------------------------------------"  << endl;
//    *log_file << "Begin of Geometry Metrics Benchmark" << endl;
//    *log_file << "------------------------------------------------------------"  << endl;

//    cout << "Start" << endl;
//    start_all = clock();

//    for (int sub_idx = 1; sub_idx < 10; sub_idx++)
//    {
//        for (int r2p_idx = 0; r2p_idx <= r2p_steps; r2p_idx++)
//        {
//            m_vct_rgb_src.push_back(rgb_src_0);
//            m_vct_rgb_src.push_back(rgb_src_1);
//            m_vct_rgb_src.push_back(rgb_src_mosaic_1);

//            // x -> rows, y -> columns
//            // m_img_shift.x = 0;
//            //  m_img_shift.y = -57;

//            // 1. Computes the bias on the nominal mosaic
//            start = clock();
//            const float sgf_nominal = stitch_geometry_fidelity(gain, reach_to_prec * r2p_idx, nbr_robust_pts * sub_idx);
//            end = clock();

//            //   *log_file << "--------------------------------------------------------------------------" << endl;
//            *log_file << "  SUB Step: " << sub_idx << " - R2P Step: " << r2p_idx << " - R2B Value: " << reach_to_prec * r2p_idx << endl;
//            *log_file << "--------------------------------------------------------------------------" << endl;
//            *log_file << " 1. SGF Nominal (Bias): " << sgf_nominal << " (" << end - start << " mSec.)" << endl;

//            m_vct_rgb_src.clear();

//            // 2. Computes the distance error on the distort mosaic
//            m_vct_rgb_src.push_back(rgb_src_0);
//            m_vct_rgb_src.push_back(rgb_src_1);
//            m_vct_rgb_src.push_back(rgb_src_mosaic_2);

//            start = clock();
//            const float sgf_distort = stitch_geometry_fidelity(gain, reach_to_prec * r2p_idx, nbr_robust_pts * sub_idx);
//            end = clock();

//            const float unbiased_sgf = sgf_distort - sgf_nominal;

//            if (unbiased_sgf < 0.0)
//            {
//                *log_file << " Negative SGF !!" << endl;
//            }
//            *log_file << " 2. SGF Distort " << sgf_distort << " (" << end - start << " mSec.)" << endl;
//            *log_file << " 3. SGF Unbiased " << unbiased_sgf << endl;
//            *log_file << "--------------------------------------------------------------------------" << endl;

//            // Update statistics
//            MaxSGF max_sgf(unbiased_sgf, sub_idx, r2p_idx);
//            vct_max_sgf.push_back(max_sgf);

//            if (max_sgf_r2p < unbiased_sgf)
//            {
//                max_sgf_r2p = unbiased_sgf;
//            }
//            m_vct_rgb_src.clear();
//        }

//        *log_file << " Max. value of SGF: " << max_sgf_r2p << endl;
//    }
//    sort(vct_max_sgf.begin(), vct_max_sgf.end(), CompareMaxSGFAscend);

//    *log_file << "SGF: " << vct_max_sgf[0].m_sgf << endl;
//    *log_file << "SUB: " << vct_max_sgf[0].m_sub << endl;
//    *log_file << "R2P: " << vct_max_sgf[0].m_r2p << endl;

//    // 2. Computes the distance error on the distort mosaic
//    m_vct_rgb_src.push_back(rgb_src_0);
//    m_vct_rgb_src.push_back(rgb_src_1);
//    m_vct_rgb_src.push_back(rgb_src_mosaic_2);
//    const float sgf_distort_verify = stitch_geometry_fidelity(gain, reach_to_prec * vct_max_sgf[0].m_r2p, nbr_robust_pts * vct_max_sgf[0].m_sub);

//    *log_file << "Verify SGF: " << sgf_distort_verify << endl;
//    log_file->close();
//    end_all = clock();
//    cout << "Stop" << endl;
//    cout << "Max SGF: " << vct_max_sgf[0].m_sgf << endl;
//    cout << "Total elapsed time: " << (double)(end_all - start_all)/1000  << " sec." << endl;
//}

////-----------------------------------------------------------------------------
//// this function amplifies the difference in position of a keypoint existing in two different images.
//// The basic algorith would already be sensitive to changes in keypoint position where Img1 is partially replaced by Img 2 with
//// distorted points, but the sensitivity would be low.
//// The metric islogaritmically sensitive to small gemetrical distortions and less to high ones.
//float GeometryMetrics::ghosting_fidelity(vector<vector<KeyPoint> >robust_keypoint_img, const int img_idx, Point &v_g)
//{
//    const float w_size = 1.0;
//    const float w_response = 1.0;
//    const float w_angle = 1.0;
//    const float w_dist = 1.0;

//    float d_x = 0.0;
//    float d_y = 0.0;

//    // The two indes are crisscrossed between 0 and 1
//    const int img_idx_2 = (img_idx + 1) % 2;

//    const int max_size = max(robust_keypoint_img[img_idx].size(), robust_keypoint_img[img_idx_2].size());

//    vector <float> d_pos(max_size);
//    vector <float> d_size(max_size);
//    vector <float> d_angle(max_size);
//    vector <float> d_response(max_size);

//    float dist_max = 0.0;
//    float dist_min = 99999999;

//    //    float size_max = 0.0;
//    //    float angle_max = 0.0;
//    //    float response_max = 0.0;

//    vector<pair<int, float> > kpt_affin(max_size);
//    vector<Point> v_kpts;

//    // int i_1 = 0;
//    // Change this parameter to chech the sensitivity to distortioons and phantom effects
//    // robust_keypoint_img[img_idx_2][0].pt.x += 0;
//    // robust_keypoint_img[img_idx_2][0].pt.y += 0;

//    // Change this parameter to check robustness in keypoint matching
//    // robust_keypoint_img[img_idx_2][0].size += 0;

//    Point2f n;

//    for (int i = 0; i < robust_keypoint_img[img_idx].size(); i++)
//    {
//        //    cout << " Image 1 - Kpt: " << i <<  "  " << robust_keypoint_img[img_idx][i].pt.x << ", " << robust_keypoint_img[img_idx][i].pt.y  << endl;
//    }

//    for (int i = 0; i < robust_keypoint_img[img_idx_2].size(); i++)
//    {
//        //    cout << " Image 2 - Kpt: " << i <<  "  " <<  robust_keypoint_img[img_idx_2][i].pt.x << ", " << robust_keypoint_img[img_idx_2][i].pt.y  << endl;
//    }

//    for (int i_1 = 0; i_1 < robust_keypoint_img[img_idx].size(); i_1++)
//    {
//        for (int i_2 = 0; i_2 < robust_keypoint_img[img_idx_2].size(); i_2++)
//        {
//            //vector<KeyPoint> vkp1 = robust_keypoint_img[img_idx];
//            //vector<KeyPoint> vkp2 = robust_keypoint_img[img_idx];

//            int p1_x = robust_keypoint_img[img_idx][i_1].pt.x;
//            int p1_y = robust_keypoint_img[img_idx][i_1].pt.y;

//            int p2_x = robust_keypoint_img[img_idx_2][i_2].pt.x;
//            int p2_y = robust_keypoint_img[img_idx_2][i_2].pt.y;

//            Point shift( m_vct_src_shift[0].x,  m_vct_src_shift[0].y);

//            if (img_idx == 1)
//            {
//                shift.x *= -1;
//                shift.y *= -1;
//            }
//            d_x = robust_keypoint_img[img_idx][i_1].pt.x - robust_keypoint_img[img_idx_2][i_2].pt.x - shift.x;
//            d_y = robust_keypoint_img[img_idx][i_1].pt.y - robust_keypoint_img[img_idx_2][i_2].pt.y - shift.y;
//            d_pos[i_2] = sqrt(pow(d_x, 2) + pow(d_y, 2));

//            d_size[i_2] = abs(robust_keypoint_img[img_idx][i_1].size - robust_keypoint_img[img_idx_2][i_2].size);
//            d_angle[i_2] = abs(robust_keypoint_img[img_idx][i_1].angle - robust_keypoint_img[img_idx_2][i_2].angle);
//            d_response[i_2] = abs(robust_keypoint_img[img_idx][i_1].response - robust_keypoint_img[img_idx_2][i_2].response);

//            //   v_g[i_1].x = (robust_keypoint_img[img_idx][i_1].pt.x + (robust_keypoint_img[img_idx][i_1].pt.x * affin));
//            //   v_g[i_1].y = (robust_keypoint_img[img_idx][i_1].pt.y + (robust_keypoint_img[img_idx][i_1].pt.y * affin));

//            if (d_pos[i_2] > dist_max)
//            {
//                dist_max = d_pos[i_2];
//            }

//            if (d_pos[i_2] <= dist_min)
//            {
//                dist_min = d_pos[i_2];
//            }

//            //            if (d_size[i_2] > size_max)
//            //            {
//            //                size_max = d_size[i_2];
//            //            }

//            //            if (d_response[i_2] > response_max)
//            //            {
//            //                response_max = d_response[i_2];
//            //            }

//            //            if (abs(d_angle[i_2]) > abs(angle_max))
//            //            {
//            //                angle_max = abs(d_angle[i_2]);
//            //            }
//            //            // const float d_octave = robust_keypoint_img[img_idx][i].pt.d_octave - robust_keypoint_img[img_idx_2][i].pt.d_octave;
//            //  }

//            // for (int i_2 = 0; i_2 < robust_keypoint_img[img_idx_2].size(); i_2++)
//            // {
//            // Normalized affinity computation
//            // kpt_affin[i_2] = ((pow(d_size[i_2], 0.2) + pow(d_response[i_2], 0.2) + pow(d_pos[i_2], 0.2) + pow(d_angle[i_2], 0.2)) / 4);

//            // Non normalized affinity computation
//            float aa = 1 / (0.0000001 + exp(d_size[i_2]));
//            float ab = exp(0);

//            //            kpt_affin[i_2] =
//            //                pow(d_size[i_2], 1.2)
//            //                +  pow(d_response[i_2], 1.2)
//            //                +  pow(d_pos[i_2], 1.2)
//            //                +  pow(d_angle[i_2], 1.2);

//            float a1 = (1 / (0.0000001 + exp(d_size[i_2])));
//            float a2 = (1 / (0.0000001 + exp(d_size[i_2])));
//            float a3 = (1 / (0.0000001 + exp(d_pos[i_2])));
//            float a4 = (1 / (0.0000001 + exp(d_angle[i_2])));

//            // The formula avoids that if one pearameter drops, the entire result drops
//            const float val = (
//                        (1 / (0.0000001 + exp(d_size[i_2] * w_size)))
//                        + (1 / (0.0000001 + exp(d_response[i_2] * w_response)))
//                        + (1 / (0.0000001 + exp(d_pos[i_2] * w_dist)))
//                        + (1 / (0.0000001 + exp(d_angle[i_2] * w_angle)))) / 4;

//            pair<int, float> pr;
//            pr.first = i_2;
//            pr.second = val;
//            kpt_affin[i_2] = pr;
//        }

//        vector<float> dist_norm(d_pos.size());

//        for (int i = 0; i < d_pos.size(); i++)
//        {
//            dist_norm[i] = d_pos[i] / dist_max;
//        }

//        // Sorts the vector to get the highest likelikhood at the top
//        sort(kpt_affin.begin(), kpt_affin.end(), pairCompareFloatDiscend);

//        // Likelikhood and distance must be kept separate
//        // The distance is anyway used to compute the likelikhood. If points are too far,
//        // thay may not be the same.

//        //int i_1 = 0;
//        // for (int i_1 = 0; i_1 < robust_keypoint_img[img_idx].size(); i_1++)
//        // {
//        const int dist_idx = kpt_affin[0].first;

//        // Geometrical Phantom effect distortion sentitivity
//        const float sensibility = 1.0;

//        //vector<float> norm;
//        //  normalize(d_pos, norm, dist_min, dist_max);

//        // Operating range [0..9.21]
//        const float log_range = 0.1;
//        // float kpt_offset = pow(norm[dist_idx], 2.0);
//        // Range should not set to too high sensitivity to preserve immunity from float point errors and noise

//        float kpt_offset = (log(pow(dist_norm[dist_idx], sensibility) + log_range)) + 2.302585125;

//        if (kpt_offset < 0.0)
//        {
//            exit(-1);
//        }

//        if (kpt_offset < 0.000001)
//            kpt_offset = 0.0;

//        const int ax = robust_keypoint_img[img_idx][i_1].pt.x;
//        const int ay = robust_keypoint_img[img_idx][i_1].pt.y;

//        //  const int offs_x = robust_keypoint_img[img_idx][i_1].pt.x + kpt_offset;
//        //   const int offs_y = robust_keypoint_img[img_idx][i_1].pt.y + kpt_offset;


//        // Je pourrai alculer la moyenne des pos des pt ou la moyenne des dir et un vecteur virtuel qui peut théoriquement devenir très grand et ne pas rentrer dans l'image
//        // const int offs_max_x = robust_keypoint_img[img_idx][i_1].pt.x / 4;

//        const float new_kpt_x = robust_keypoint_img[img_idx][i_1].pt.x + kpt_offset;
//        const float new_kpt_y = robust_keypoint_img[img_idx][i_1].pt.y + kpt_offset;

//        n.x += new_kpt_x;
//        n.y += new_kpt_y;

//        //     cout << kpt_offset << endl;

//        // At max error rate, the max offset deviation is %of the
//        v_kpts.push_back(Point(new_kpt_x, new_kpt_y));

//        // cout << i_1 << " Keypoint (" <<  robust_keypoint_img[img_idx][i_1].pt.x << "," << robust_keypoint_img[img_idx][i_1].pt.y << ") - Offset " << kpt_offset << " - New Keypoints (" << new_kpt_x << ", " << new_kpt_y << ")" << endl;
//    }

//    n.x /= v_kpts.size() + 0.0000001;
//    n.y /= v_kpts.size() + 0.0000001;

//    v_g.x = n.x;
//    v_g.y = n.y;

//    //for (int i = 0; i < v_kpts.size(); i++)
//    {
//        //   v_g.x += v_kpts[i].x;
//        //   v_g.y += v_kpts[i].y;
//    }

//    //  v_g.x /= v_kpts.size();
//    //  v_g.y /= v_kpts.size();

//    float ret = 0.0;

//    if (v_kpts.size() > 0)
//    {
//        ret = v_kpts.size();
//    }
//         return ret;
//}

//float GeometryMetrics::stitch_geometry_fidelity(const float gain, const float reach_to_prec, const float nbr_robust_keypts) //const Mat rgb_img_0, const Mat rgb_img_1, Mat rgb_img_pano)
//{
//    //---------------------------------------------
//    // Begin Test Graph cut
//    //---------------------------------------------

//    //    CreateSuperPixels(0, nbr_spix, compactness);
//    //    CreateSuperPixels(1, nbr_spix, compactness);
//    //    CreateSuperPixels(2, nbr_spix, compactness);

//    //    // Dumps the generated SPix
//    //    SuperImage *p_simg_ref = &(m_vct_simg[0]);
//    //    SPixTools spt(m_path);

//    //    // Dumps the generated SPix images
//    //    spt.dumpSpixImage(*this, 0, m_path + "dump/", INFO_ID);
//    //    spt.dumpSpixImage(*this, 1, m_path + "dump/", INFO_ID);
//    //    spt.dumpSpixImage(*this, 2, m_path + "dump/", INFO_ID);

//    //    spt.dumpSpixImage(*this, 0, m_path + "dump/", INFO_GRAVITY);
//    //    spt.dumpSpixImage(*this, 1, m_path + "dump/", INFO_GRAVITY);
//    //    spt.dumpSpixImage(*this, 2, m_path + "dump/", INFO_GRAVITY);

//    // Creates the vector of inter-frame sPix distances
//    // vector<vector<vector<SPixPair>>> vct_spix_pairs(m_vct_simg.size(), vector<vector<SPixPair>>(256));
//    // m_max_pairing_dist = 32;

//    // Loops the entire list of sPix between the images 0 and 1
//    // for (int spix_idx = 0; spix_idx < m_vct_simg[0].m_vct_spix.size(); spix_idx++)
//    {
//        //  CreateSPixPairingSet(0, 1, spix_idx, vct_spix_pairs);
//    }

//    //  for (int spix_idx = 0; spix_idx < p_simg_ref->m_vct_spix.size(); spix_idx++)
//    {
//        // Creates and stores each sPix for visual matching
//        // SuperPixel *p_spix = &(p_simg_ref->m_vct_spix[spix_idx]);
//        //   spt.dumpSpix(img_idx, spix_idx, m_vct_rgb_src[img_idx], p_spix);
//    }

//    //--------------------------------------------------------------------------------------------

//    if (!m_vct_rgb_src[0].data || !m_vct_rgb_src[1].data || !m_vct_rgb_src[2].data)
//    {
//        cout << " --(!) Error reading images " << endl;
//        return -1;
//    }

//    Mat img[NBR_IMG];
//    img[0] = m_vct_rgb_src[0];
//    img[1] = m_vct_rgb_src[1];

//    // Extracts the canvas of the nominal images from the mosaic
//    img[2] = m_vct_rgb_src[2](Rect(0, 0, m_vct_rgb_src[0].cols, m_vct_rgb_src[0].rows));
//    img[3] = m_vct_rgb_src[2](Rect(m_vct_src_shift[0].x, m_vct_src_shift[0].y, m_vct_rgb_src[1].cols, m_vct_rgb_src[1].rows));

//    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/geometry_metrics/dump_surf_response/img0.png", img[0]);
//    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/geometry_metrics/dump_surf_response/img1.png", img[1]);
//    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/geometry_metrics/dump_surf_response/img2.png", img[2]);
//    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/geometry_metrics/dump_surf_response/img3.png", img[3]);

//    // Debug
//    //        imshow("img0", img[0] );
//    //        imshow("img1", img[1] );
//    //        imshow("img2", img[2] );
//    //        imshow("img3", img[3] );
//    //        imshow("mosa_dist", m_vct_rgb_src[2] );
//    //        waitKey(0);

//    // Detect the keypoints using SURF Detector
//    int minHessian = 00;

//    SurfFeatureDetector detector(minHessian);
//    vector<KeyPoint> keypoint_img[NBR_IMG];

//    vector<vector<KeyPoint> > robust_keypoint_img(NBR_IMG);
//    float max_keypt_resp[NBR_IMG];
//    float min_keypt_resp[NBR_IMG];

//    float max_keypt_size[NBR_IMG];
//    float min_keypt_size[NBR_IMG];

//    // Vector of keypoint rank
//    // The rank is computed an a weigthed sum of the normalized values of response and size of the keyPoint
//    vector<pair<int, float>> keypt_rank[NBR_IMG];

//    const float size_weight = reach_to_prec;
//    const float resp_weight = 1.0 - size_weight;

//    float nbr_keypts[NBR_IMG];                                            // Number of keypoints to retain
//    // Mat img_keypt[NBR_IMG];                                            // Image of keypoints vector for dumping purposes
//    Mat img_rob_keyp[NBR_IMG];                                            // Image of robust keypoints vector for dumping purposes
//    vector<Point> g(NBR_IMG);                                                     // Gravity point vector

//    vector<Point> v_g(NBR_IMG);

//    for (int i = 0; i < NBR_IMG; i++)
//    {
//        g[0].x = 0;
//        g[0].y = 0;
//    }

//    for (int c = 0; c < NBR_IMG; c++)
//    {
//        detector.detect(img[c], keypoint_img[c]);

//        // Sort the keyPoints upon SURF response
//        // The Keypoint response that specifies strength of keypoints
//        sort(keypoint_img[c].begin(), keypoint_img[c].end(), keyPointCompareResponse);
//        max_keypt_resp[c] = keypoint_img[c][0].response;
//        min_keypt_resp[c] = keypoint_img[c][keypoint_img[c].size() - 1].response;

//        // Sort the robust keyPoints upon SURF size
//        sort(keypoint_img[c].begin(), keypoint_img[c].end(), keyPointCompareSize);
//        max_keypt_size[c] = keypoint_img[c][0].size;
//        min_keypt_size[c] = keypoint_img[c][keypoint_img[c].size() - 1].size;

//        for (int i = 0; i < keypoint_img[c].size(); i++)
//        {
//            keypt_rank[c].push_back(pair<int, float>(i, compute_keypoint_weight(keypoint_img[c][i], resp_weight, size_weight, min_keypt_resp[c], max_keypt_resp[c], min_keypt_size[c], max_keypt_size[c])));
//        }

//        // Sorts the vector in ascending order and choose the last entry as the biggest
//        sort(keypt_rank[c].begin(), keypt_rank[c].end(), pairCompareFloatDiscend);

//        // Dumps the values on the screen
//        // for (int i = 1400; i < 1420; i++)
//        {
//            //    int i2 = keypt_rank[0][i].first;
//            //    cout << i << "  I2: " << i2 << " Response: " << keypoint_img[0][i2].response << " Size: " << keypoint_img[0][i2].size << " Rank: " << keypt_rank[0][i].second << endl;
//        }

//        // Percentage of the total number of keyPoints
//        nbr_keypts[c] = ((float)keypoint_img[c].size() / 100.0) * nbr_robust_keypts;

//        // At this level all keypoints referencred by keypt_rank are sorted by weight of distance and response
//        for (int i = 0; i < nbr_keypts[c]; i++)
//        {
//            //  cout << keypt_rank[c][i].second << endl;
//            robust_keypoint_img[c].push_back(keypoint_img[c][keypt_rank[c][i].first]);
//        }

//        // Draw keypoints
//        const bool do_dump = true;

//        if (do_dump == true)
//        {
//            // drawKeypoints(img[c], keypoint_img[c], img_keypt[c], Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

//            // Draw robust keypoints
//            drawKeypoints(img[c], robust_keypoint_img[c], img_rob_keyp[c], Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

//            // imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/geometry_metrics/dump_sift_response/surf_result_" + to_string(c) + ".jpg", keypoint_img[c]);
//            imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/geometry_metrics/dump_surf_response/robust_surf_result_" + to_string(c) + ".jpg", img_rob_keyp[c]);
//        }

//        // Computing gravity
//        for (int i = 0; i < robust_keypoint_img[c].size(); i++)
//        {
//            g[c].x += pow(robust_keypoint_img[c][i].pt.x, gain);
//            g[c].y += pow(robust_keypoint_img[c][i].pt.y, gain);
//        }

//        if (robust_keypoint_img[c].size() > 0)
//        {
//            g[c].x /= robust_keypoint_img[c].size();
//            g[c].y /= robust_keypoint_img[c].size();
//        }
//    }

//    //---------------------------
//    // Begin RGI 17/04/2017
//    //---------------------------
//    vector<vector<KeyPoint> > shared_robust_keypoint_img(NBR_IMG);

//    for(int img_idx = 0; img_idx < NBR_IMG; img_idx++)
//    {
//        for (int kpt_idx = 0; kpt_idx < robust_keypoint_img[img_idx].size(); kpt_idx++)
//        {
//            //            int akx = robust_keypoint_img[img_idx][kpt_idx].pt.x;
//            //            int aky = robust_keypoint_img[img_idx][kpt_idx].pt.y;

//            //            int ashx = m_vct_src_shift[0].x;
//            //            int ashy = m_vct_src_shift[0].y;

//            //            int aszx =  m_vct_rgb_src[0].cols;
//            //            int aszy =  m_vct_rgb_src[0].rows;

//            if (img_idx == 0 || img_idx == 2)
//            {
//                if (robust_keypoint_img[img_idx][kpt_idx].pt.x > m_vct_src_shift[0].x &&
//                        robust_keypoint_img[img_idx][kpt_idx].pt.y > m_vct_src_shift[0].y &&
//                        robust_keypoint_img[img_idx][kpt_idx].pt.x < m_vct_rgb_src[0].cols &&
//                        robust_keypoint_img[img_idx][kpt_idx].pt.y < m_vct_rgb_src[0].rows)
//                {
//                    shared_robust_keypoint_img[img_idx].push_back(robust_keypoint_img[img_idx][kpt_idx]);
//                }
//            }
//            else if (img_idx == 1 || img_idx == 3)
//            {
//                int adx = m_vct_rgb_src[0].cols - m_vct_src_shift[0].x;
//                int ady = m_vct_rgb_src[0].rows - m_vct_src_shift[0].y;

//                if (robust_keypoint_img[img_idx][kpt_idx].pt.x > 0 &&
//                        robust_keypoint_img[img_idx][kpt_idx].pt.y > 0 &&
//                        robust_keypoint_img[img_idx][kpt_idx].pt.x < m_vct_rgb_src[0].cols - m_vct_src_shift[0].x &&
//                        robust_keypoint_img[img_idx][kpt_idx].pt.y < m_vct_rgb_src[0].rows - m_vct_src_shift[0].y)
//                {
//                    shared_robust_keypoint_img[img_idx].push_back(robust_keypoint_img[img_idx][kpt_idx]);
//                }
//            }

//        }
//    }


//    // Computes the gravity in the shared region
//    for (int img_idx = 0; img_idx < NBR_IMG; img_idx++)
//    {
//        Point2f n;

//        for (int i = 0; i < shared_robust_keypoint_img[img_idx].size(); i++)
//        {
//            n.x += shared_robust_keypoint_img[img_idx][i].pt.x;
//            n.y += shared_robust_keypoint_img[img_idx][i].pt.y;
//        }

//        n.x /= shared_robust_keypoint_img[img_idx].size() + 0.000000001;
//        n.y /= shared_robust_keypoint_img[img_idx].size() + 0.000000001;

//        //       if (n.x < 0.0000001)
//        //       {
//        //           n.x = 0.0;
//        //       }

//        //       if (n.y < 0.0000001)
//        //       {
//        //           n.y = 0.0;
//        //       }

//        v_g[img_idx] = n;
//    }

//    float ret = ghosting_fidelity(shared_robust_keypoint_img, 0, v_g[0]);
//    ret = ghosting_fidelity(shared_robust_keypoint_img, 1, v_g[1]);

//    // Distance of the left nominal and distorted images
//    const float d_1_3 = sqrt(pow(v_g[3].x - v_g[1].x, 2) + pow(v_g[3].y - v_g[1].y, 2));

//    // Distance of the right nominal and distorted images
//    const float d_0_2 = sqrt(pow(v_g[2].x - v_g[0].x, 2) + pow(v_g[2].y - v_g[0].y, 2));
//    const float d_tot = d_0_2 + d_1_3;

//    cout << "-------------------------------------" << endl;
//    if (ret > 0)
//    {
//        cout << " - Distance Totale: " << d_tot << endl;
//    }
//    else
//    {
//        cout << "No matching points found in the shared region !!" << endl;
//    }
//    cout << "-------------------------------------" << endl;

//    //---------------------------
//    // End RGI 17/04/2017
//    //---------------------------


//    // Distance of the left nominal and distorted images
//    const float dist_1_3 = sqrt(pow(g[3].x - g[1].x, 2) + pow(g[3].y - g[1].y, 2));

//    // Distance of the right nominal and distorted images
//    const float dist_0_2 = sqrt(pow(g[2].x - g[0].x, 2) + pow(g[2].y - g[0].y, 2));

//    //  const float dist_tot = dist_1_3 + dist_0_2;

//    float max_euclid_dist[NBR_IMG/2];
//    max_euclid_dist[0] = sqrt(pow(img[0].cols, 2) + pow(img[0].rows, 2));
//    max_euclid_dist[1] = sqrt(pow(img[0].cols, 2) + pow(img[1].rows, 2));

//    //const float n_dist_0_2 = dist_0_2 / max_euclid_dist[0];
//    // const float n_dist_1_3 = dist_1_3 / max_euclid_dist[1];

//    const float dist_tot = dist_0_2 + dist_1_3;

//    // cout << "Total distance error: " << dist_tot << endl;

//    //    //-----------------------------------------------
//    //    // Normalized distances
//    //    //-----------------------------------------------
//    //    Point2f n_g[NBR_IMG];
//    //    for (int i = 0; i < NBR_IMG; i++)
//    //    {
//    //        n_g[i].x = (float)(g[i].x) / img[i].cols;
//    //        n_g[i].y = (float)(g[i].y) / img[i].rows;
//    //    }
//    //    // Distance of the left nominal and distorted images
//    //    const float n_dist_1_3 = sqrt(pow(n_g[3].x - n_g[1].x, 2) + pow(n_g[3].y - n_g[1].y, 2));

//    //    // Distance of the right nominal and distorted images
//    //    const float n_dist_0_2 = sqrt(pow(n_g[2].x - n_g[0].x, 2) + pow(n_g[2].y - n_g[0].y, 2));

//    //    const float dist_tot = n_dist_1_3 + n_dist_0_2;

//    // cout << "Total distance error: " << dist_tot << endl;
//    return dist_tot;

//    //    cout <<  "1. Good matches:  " << m_good_match_count << endl;
//    //    cout <<  "2. Bad matches:  " << m_bad_match_count << endl;
//    //    cout <<  "3. Match failure:  " << m_match_failure_count << endl;
//}

////-----------------------------------------------------------------------------

//// Computes set of cross-frame minimal cartesian distances among sPix and stores them into a distance table
////---------------------------------------------------------------------------
//// CreateSPixPairingSet
////---------------------------------------------------------------------------
//bool GeometryMetrics::CreateSPixPairingSet(const int simg_idx_0, const int simg_idx_1, const int spix_idx, SPixPairSet &vct_spix_pairs)
//{

//    // Registration shift vector
//    // vector<Point> m_vct_src_shift;

//    const int sz = m_vct_simg[simg_idx_0].m_vct_spix.size();
//    vector<int> vct_dist(sz);

//    // Candidate pairs
//    vector<pair<int, int>> pairs;

//    const int min_src_size = min(m_vct_rgb_src[simg_idx_0].rows,  m_vct_rgb_src[simg_idx_0].cols);
//    const int max_dist_ray = 100; //min_src_size/4.0;

//    // if (m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gx + img_shift.x < m_vct_rgb_src[simg_idx_0].rows &
//    //         m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gy + img_shift.y < m_vct_rgb_src[simg_idx_0].cols)
//    {
//        for (int k = 0; k < sz; k++)
//        {
//            //-----------------------------------------------------------------------------------------
//            // Computes the Euclidean distances
//            // Distances are computed at once
//            // Only the k shortest distances will be considered for photometric sPix Template Matching
//            //-----------------------------------------------------------------------------------------
//            // x -> rows, y -> columns

//            // int ax0 = (int)m_vct_simg[simg_idx_1].m_vct_spix[k].m_gx;
//            // int ay0 = (int)m_vct_simg[simg_idx_1].m_vct_spix[k].m_gy;

//            int ax = (int)m_vct_simg[simg_idx_1].m_vct_spix[k].m_gx; // - img_shift.x;
//            int ay = (int)m_vct_simg[simg_idx_1].m_vct_spix[k].m_gy; // img_shift.y;

//            int bx0 = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx].m_gx;
//            int by0 = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx].m_gy;

//            int bx = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx].m_gx - m_vct_src_shift[0].x;
//            int by = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx].m_gy - m_vct_src_shift[0].y;

//            // ax and ay are >= 0 only is shared region
//            if (bx >= 0 && by >= 0)
//            {
//                const int d_x = abs(bx - ax);
//                const int d_y = abs(by - ay);

//                const float d = sqrt(pow(d_x, 2) + pow(d_y, 2));

//                if (d <= max_dist_ray)
//                {
//                    vct_dist[k] = d;
//                    pair <int, int> pr(k, d);
//                    pairs.push_back(pr);
//                }
//            }
//        }

//        // Sorts the dist vector to get the k shortest distances
//        sort(pairs.begin(), pairs.end(), pairCompareFloatAscend);
//        int dist_size = pairs.size();

//        for (int k = 0; k < dist_size; k++)
//        {
//            cout << k << " DIST SPix: " <<  spix_idx << ":" << pairs[k].first << "  " << pairs[k].second << endl;
//            cout << "-------------------------------------" << endl;
//        }

//        int dist_cnt = 0;

//        for (int k = 0; k < pairs.size(); k++)
//        {
//            //-----------------------------------------------------------------------
//            // Computes the SPix Template Matching
//            //-----------------------------------------------------------------------
//            //        // 1. rgbSAD matching
//            //        // Interframe Photometric signature check
//            //        bool photom_match = RgbSAD(simg_idx_0, spix_idx, simg_idx_1, k);

//            //        // 2. Hue Gaussian Histogram Matching
//            //        float hue_similarity = 0.0;
//            //  SPixHueMatching(simg_idx_0, spix_idx, simg_idx_0, pairs[k].first, hue_similarity);

//            // 3. Hue SAD matching
//            // The Template Matching is computed on the k shortest distances
//            bool hue_match = false;
//            float hue_sad = 0.0;
//            int paired_spix_id = pairs[k].first;
//            hue_match = SPixHueSAD(simg_idx_0, spix_idx, simg_idx_1, paired_spix_id, hue_sad);

//            if (hue_match == true)
//            {
//                dist_cnt++;
//                hue_sad = pairs[k].second;

//                vector <SPixPair>vct1;
//                vct_spix_pairs[simg_idx_0].push_back(vct1);
//                vct_spix_pairs[simg_idx_0][spix_idx].push_back(SPixPair(spix_idx, simg_idx_0, paired_spix_id, simg_idx_1, hue_sad));
//            }

//            //-----------------------------------------------------------------------------------
//            // 1. Matching SPix Images 0 - 1
//            //-----------------------------------------------------------------------------------
//            // Extracts the SPix from the selected pairs of images and converts them into Mat image format
//            //   SuperImage *p_simg_ref = &(m_vct_simg[0]);
//            SPixTools spt(m_path);

//            Mat spix_img_0;
//            spt.getImageFromSpix(&m_vct_simg[0].m_vct_spix[spix_idx], m_vct_rgb_src[0], spix_img_0);

//            //imshow("spix_img 0", spix_img_0);
//            //  waitKey(0);

//            Mat spix_img_1;

//            // Extracts the SPix from the selected pairs of images and converts them into Mat image format
//            spt.getImageFromSpix(&m_vct_simg[1].m_vct_spix[paired_spix_id], m_vct_rgb_src[1], spix_img_1);

//            //imshow("spix_img 1",spix_img_1);
//            // waitKey(0);

//            vector<KeyPoint> keypoint_img[4];

//            if (spix_img_0.rows != 0 && spix_img_1.cols != 0)
//            {
//                RobustMatcher *robust_matcher = new RobustMatcher();
//                Mat H = robust_matcher->match(spix_img_0, spix_img_1, &keypoint_img[0], &keypoint_img[1]);
//                robust_matcher->show(spix_img_0, spix_img_1, keypoint_img[0], keypoint_img[1], m_path + "dump/image_matches_" + to_string(spix_idx) + "_" + to_string(paired_spix_id)+ ".png");

//                std::cout << "Number of good matching " << (int)robust_matcher->matchArray.size() << "\n" << endl;

//                if ((int)robust_matcher->matchArray.size() == 0)
//                {
//                    m_match_failure_count++;
//                    // H = 0;
//                    cout << "No matches found (Registration failure) !!" << endl;
//                    // return false;
//                }

//                // Minimum number of good features to find to accept the Feature Matching
//                const int min_match_array_size = 2;

//                vector<Point2f> obj_corners(4);
//                obj_corners[0] = Point2f(0, 0);
//                obj_corners[1] = Point2f(m_vct_rgb_src[simg_idx_1].cols, 0);
//                obj_corners[2] = Point2f(m_vct_rgb_src[simg_idx_1].cols, m_vct_rgb_src[simg_idx_1].rows);
//                obj_corners[3] = Point2f(0, m_vct_rgb_src[simg_idx_1].rows);
//                //  vector<Point2f> scene_corners(4);

//                if ((int)robust_matcher->matchArray.size() >= min_match_array_size)
//                {
//                    m_good_match_count++;
//                    cout << min_match_array_size << "matches found !!" << endl;
//                }
//                else
//                {
//                    m_bad_match_count++;
//                    cout << "Bad matching, skipping entry !!" << endl;
//                }
//            }
//        }
//    }
//    return true;
//}

////-----------------------------------------------------------------------------------------
//// Computes the Cartesian distances
//// Distances are computed at once
////-----------------------------------------------------------------------------------------
//float GeometryMetrics::compute_euclidean_distance(const int spix_idx_0, const int spix_idx_1)
//{
//    //    int simg_idx_1 = simg_idx_0 + 1;
//    //    const int sz = m_vct_simg[simg_idx_1].m_vct_spix.size();
//    //    vector<int> vct_dist(sz);
//    //    vector<pair<int, int>> pairs;
//    //    const int min_src_size = min(m_vct_rgb_src[simg_idx_1].rows,  m_vct_rgb_src[simg_idx_1].cols);
//    //    const int max_dist_ray = min_src_size/4.0;

//    //    for (int spix_idx = 0; spix_idx < m_vct_simg[spix_idx_0].size(); spix_idx++)
//    //    {
//    //        if (m_vct_simg[spix_idx_0].m_vct_spix[spix_idx].m_gx + m_vct_src_shift[simg_idx_1].x < m_vct_rgb_src[simg_idx_1].rows &
//    //                m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gy + m_vct_src_shift[simg_idx_1].y < m_vct_rgb_src[simg_idx_1].cols)
//    //        {
//    //            for (int k = 0; k < sz; k++)
//    //            {
//    //                const int d_x = abs((int)m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gx - (int)m_vct_simg[simg_idx_0].m_vct_spix[k].m_gx + m_vct_src_shift[simg_idx_1].x);
//    //                const int d_y = abs((int)m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gy - (int)m_vct_simg[simg_idx_0].m_vct_spix[k].m_gy + m_vct_src_shift[simg_idx_1].y);

//    //                // After registring shift vector applied, if no registration error exists, the d_x and d_y should be 0
//    //                const float d = sqrt(pow(d_x, 2) + pow(d_y, 2));

//    //                if (d <= max_dist_ray)
//    //                {
//    //                    vct_dist[k] = d;
//    //                    pair <int, int> pr(k, d);
//    //                    pairs.push_back(pr);
//    //                }
//    //            }
//    //        }

//    //        // Sorts the dist vector to get the k shortest distancess
//    //        sort(pairs.begin(), pairs.end(), pairCompareFloatAscend);
//    //        int dist_size = pairs.size();
//    //    }
//    return 0.0;
//}

////-----------------------------------------------------------------------------

//void GeometryMetrics::sort_spix_size(const int img_idx, vector<pair<int, int>> &sorted_spix_size)
//{
//    // Selects the biggest SPix
//    int sz = m_vct_simg[img_idx].m_vct_spix.size();

//    for (int spix_idx = 0; spix_idx < sz; spix_idx++)
//    {
//        SuperPixel p_spix = m_vct_simg[img_idx].m_vct_spix[spix_idx];

//        if (p_spix.m_vct_boundary.size() != 0)
//        {
//            cout << spix_idx << "Boundary: " << p_spix.m_vct_boundary.size() << endl;
//            pair<int, int> pr(0, p_spix.m_vct_boundary.size());
//            sorted_spix_size[spix_idx] = pr;
//        }
//    }

//    // Sorts the vector in ascending order and retains the k largest boundary sizes
//    sort(sorted_spix_size.begin(), sorted_spix_size.end(), pairCompareFloatAscend);
//}

////-----------------------------------------------------------------------------

//void GeometryMetrics::convertInternalToRGB(const Mat rgb_int, Mat rgb_dst)
//{
//    unsigned int *p_int_base2 = (unsigned int *)rgb_int.data;
//    unsigned char *p_dst_base2 = rgb_dst.data;
//    unsigned char *p_dst_end2 = rgb_dst.data + (rgb_dst.rows * rgb_dst.cols) * 3;
//    unsigned int *p_int2 = p_int_base2;
//    unsigned char *p_dst = p_dst_base2;

//    while (p_dst < p_dst_end2)
//    {
//        // Red
//        *p_dst++ = *p_int2 >> 16 & 0xff;

//        // Green
//        *p_dst++ = *p_int2 >> 8 & 0xff;

//        // Blue
//        *p_dst++ = *p_int2 & 0xff;

//        p_int2++;
//    }
//}

////-----------------------------------------------------------------------------
//// CreateSuperPixels
////-----------------------------------------------------------------------------
//bool GeometryMetrics::CreateSuperPixels(const int src_idx, const int spcount, const double compactness)
//{
//    Mat *p_img = &m_vct_rgb_src[src_idx];
//    Mat rgb_int = Mat::zeros(p_img->rows, p_img->cols, CV_32FC3);

//    imwrite(m_path + string("dst/img_src_") + to_string(src_idx) + ".jpg", *p_img);
//    convertRGBToInternal(*p_img, rgb_int);

//    int sz = p_img->cols * p_img->rows;

//    //---------------------------------------------------------
//    if (spcount < 20 || spcount > sz/4)
//    {
//        cout << "sPix size mst be < 200 !!" << endl;
//        // i.e the default size of the superpixel is 200 pixels
//        //      return false;
//    }

//    if (compactness < 1.0 || compactness > 80.0)
//    {
//        cout << "Compactness range [1.0..80.0]" << endl;
//        // return false;
//    }
//    //---------------------------------------------------------

//    int numlabels(0);

//    //  Adds the image to the source image vector
//    // m_vct_rgb_src.push_back(rgb_src);

//    // Adds an empty labels array to the vector of label arrays
//    // const int sz = rgb_src.rows * rgb_src.cols;
//    int* labels = new int[sz];
//    m_vct_labels.push_back(labels);

//    // m_vct_overlap.push_back(vector<int>(sz));

//    GeoImage geo_img(*p_img);
//    m_vct_simg.push_back(SuperImage(geo_img, src_idx));

//    SLIC slic;
//    unsigned int *img = (unsigned int *)rgb_int.data;
//    slic.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels((unsigned int *)rgb_int.data, p_img->cols, p_img->rows, m_vct_labels[src_idx], numlabels, spcount, compactness);

//    // Overlays the boundary over the src image
//    slic.DrawContoursAroundSegments(img, m_vct_labels[src_idx], p_img->cols, p_img->rows, 0);
//    Mat rgb_dst = Mat::zeros(p_img->rows, p_img->cols, CV_8UC3);
//    convertInternalToRGB(rgb_int, rgb_dst);
//    imwrite(m_path + "dst/over_" + to_string(src_idx) + ".jpg", rgb_dst);

//    m_vct_rgb_dst.push_back(rgb_dst);

//    vector<int> clusters(p_img->rows * p_img->cols, -1);

//    SuperPixel *p_spix = 0;

//    for (int r = 0; r < p_img->rows; r++)
//    {
//        for (int c = 0; c < p_img->cols; c++)
//        {
//            const int i = (p_img->cols * r) + c;

//            // At each new label, a new superpixel is created
//            if (clusters[m_vct_labels[src_idx][i]] == -1)
//            {
//                // cout << "Creating SuperPixel: " << vct_super_pix.size() << " Label: " << **p_labels[i] << endl;
//                m_vct_simg[src_idx].m_vct_spix.push_back(SuperPixel( m_vct_labels[src_idx][i], PixPos(r, c)));
//                clusters[ m_vct_labels[src_idx][i]]++;
//            }

//            p_spix = &(m_vct_simg[src_idx].m_vct_spix[ m_vct_labels[src_idx][i]]);

//            if (!p_spix)
//            {
//                return false;
//            }

//            if (p_spix->m_min_x > r)
//            {
//                p_spix->m_min_x = r;
//            }

//            if (p_spix->m_min_y > c)
//            {
//                p_spix->m_min_y = c;
//            }

//            if (p_spix->m_max_x <= r)
//            {
//                p_spix->m_max_x = r;
//            }

//            if (p_spix->m_max_y <= c)
//            {
//                p_spix->m_max_y = c;
//            }

//            // Sums up the rgb channels into grayscale
//            p_spix->m_mean_luma += p_img->at<Vec3b>(r, c)[0];
//            p_spix->m_mean_luma += p_img->at<Vec3b>(r, c)[1];
//            p_spix->m_mean_luma += p_img->at<Vec3b>(r, c)[2];

//            p_spix->m_spix_pos.push_back(PixPos(r, c));
//            clusters[ m_vct_labels[src_idx][i]]++;
//            //  cout << "Adding pixel to SuperPixel" << endl;
//        }
//    }
//    //--------------------------------------------------------------------------
//    // Computes the gravity points and stats of the sPix
//    //--------------------------------------------------------------------------
//    //  Mat drawing = Mat::zeros(this->m_vct_rgb_src[0].size(), CV_8UC3 );

//    for (int i = 0; i < m_vct_simg[src_idx].m_vct_spix.size(); i++)
//    {
//        p_spix = &(m_vct_simg[src_idx].m_vct_spix[i]);

//        if (!p_spix)
//        {
//            return false;
//        }

//        // Calculates the gravity of each sPix
//        p_spix->m_gx = (p_spix->m_min_x + p_spix->m_max_x) / 2;
//        p_spix->m_gy = (p_spix->m_min_y + p_spix->m_max_y) / 2;

//        // Calculates the luminance mean
//        p_spix->m_mean_luma /= (p_spix->m_spix_pos.size() * 3);

//        // Calculates the elongation
//        const int32_t height = abs(p_spix->m_max_x - p_spix->m_min_x);
//        const int32_t width = abs(p_spix->m_max_y - p_spix->m_min_y);
//        p_spix->m_elongation = (float)height / width;

//        // Calculates the cartesian distance between the centroid and the origin
//        p_spix->m_distance = sqrt(pow(p_spix->m_gx, 2) + pow(p_spix->m_gy, 2));

//        // Calculates the signature as polynomial model
//        m_vct_simg[src_idx].set_wights(0.2, 1.0, 1.0, 1.0);
//        m_vct_simg[src_idx].calc_signature(p_spix);

//        // Calculates the Stats
//        // Sums up the statistical values from each different sPix
//        m_vct_simg[src_idx].m_mean_size += p_spix->m_spix_pos.size();
//        m_vct_simg[src_idx].m_mean_gx += p_spix->m_gx;
//        m_vct_simg[src_idx].m_mean_gy += p_spix->m_gy;
//        m_vct_simg[src_idx].m_mean_luma += p_spix->m_mean_luma;
//        m_vct_simg[src_idx].m_elongation += p_spix->m_elongation;
//        m_vct_simg[src_idx].m_distance += p_spix->m_distance;
//    }

//    int nbr_spix = m_vct_simg[src_idx].m_vct_spix.size();
//    m_vct_simg[src_idx].m_mean_size /= nbr_spix;
//    m_vct_simg[src_idx].m_mean_gx /= nbr_spix;
//    m_vct_simg[src_idx].m_mean_gy /= nbr_spix;
//    m_vct_simg[src_idx].m_mean_luma /= nbr_spix;
//    m_vct_simg[src_idx].m_elongation /= nbr_spix;
//    m_vct_simg[src_idx].m_distance /= nbr_spix;

//    // Computes the set of boundary px and stores it into the sPix
//    // ComputeSpixBoundaries(src_idx);

//    // Computes the Morphological Oriented Regularity Estimator
//    // ComputeMORE(src_idx);


//    //    const bool enable_dump = false;

//    //        if (enable_dump == true)
//    //        {
//    //            for (int i = 0; i < m_vct_simg[src_idx].m_vct_spix.size(); i++)
//    //            {
//    //                p_spix = &(m_vct_simg[src_idx].m_vct_spix[i]);

//    //                cout << "-----------------------------------------------------" << endl;
//    //                cout << "- SPix: " << i << " Label: " << p_spix->m_label << endl;
//    //                cout << "- Size: " <<  p_spix->m_spix_pos.size() << endl;
//    //                cout << "- Gravity (" <<  p_spix->m_gx  << ", " <<   p_spix->m_gy << ")" << endl;
//    //                cout << "- mean Luma: " <<  p_spix->m_mean_luma << endl;
//    //                cout << "- Elongation: " <<  p_spix->m_elongation << endl;
//    //                cout << "- Distance from Origin: " <<  p_spix->m_distance << endl;
//    //                cout << "- Signature: " <<  p_spix->m_signature << endl;
//    //                cout << "-----------------------------------------------------" << endl;
//    //            }
//    //        }

//    //        // Print the list of sPixs, their size and their contents
//    //        cout << "---------------------------------------------------" << endl;
//    //        cout << "Printing statistics" << endl;
//    //        cout << "---------------------------------------------------" << endl;

//    //        cout << "Mean Size: " << m_vct_simg[src_idx].m_mean_size << endl;
//    //        cout << "Mean GX: " << m_vct_simg[src_idx].m_mean_gx << endl;
//    //        cout << "Mean GY: " << m_vct_simg[src_idx].m_mean_gy << endl;
//    //        cout << "Mean Luma: " << m_vct_simg[src_idx].m_mean_luma << endl;
//    //        cout << "Mean Elongation: " << m_vct_simg[src_idx].m_elongation << endl;
//    //        cout << "Mean Distance: " <<  m_vct_simg[src_idx].m_distance << endl;
//}

////-----------------------------------------------------------------------------
//// ComputeSpixBoundaries
////-----------------------------------------------------------------------------
//bool GeometryMetrics::ComputeSpixBoundaries(const int src_idx)
//{
//    Mat *p_img = &m_vct_rgb_src[src_idx];
//    Mat rgb_dst = Mat::zeros(p_img->rows, p_img->cols, CV_8UC3);
//    Mat drawing = Mat::zeros(p_img->size(), CV_8UC3);
//    SuperPixel *p_spix = 0;

//    //--------------------------------------------------------------------
//    // Computes the boundary on blank image
//    //--------------------------------------------------------------------
//    for (int i = 0; i < m_vct_simg[src_idx].m_vct_spix.size(); i++)
//    {
//        p_spix = &(m_vct_simg[src_idx].m_vct_spix[i]);

//        if (!p_spix)
//        {
//            return false;
//        }
//        Mat rgb_blank = Mat::zeros(p_img->rows, p_img->cols, CV_8UC3);
//        Mat intern_blank = Mat::zeros(p_img->rows, p_img->cols, CV_32FC3);

//        unsigned int *p_intern_blank = (unsigned int *)intern_blank.data;
//        convertRGBToInternal(rgb_blank, intern_blank);
//        SLIC slic;
//        slic.DrawContoursAroundSegments(p_intern_blank, m_vct_labels[src_idx], rgb_blank.cols, rgb_blank.rows, 0);
//        convertInternalToRGB(intern_blank, rgb_dst);

//        // Show in a window
//        // namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//        // imshow( "Contours", drawing );
//        // waitKey(0);
//    }

//    //--------------------------------------------------------------------
//    // Finds the boundary of the super pixel
//    //--------------------------------------------------------------------
//    vector<vector<Point> > contours;
//    vector<Vec4i> hierarchy;

//    Mat gray = Mat(rgb_dst.size(), CV_8UC1);
//    cvtColor(rgb_dst, gray, COLOR_RGB2GRAY);

//    // Draws a rectangle to close the Pix corresponing to the outer boundaries of the image
//    rectangle(gray, Rect(0, 0, gray.cols, gray.rows), Scalar(255, 255, 255), 2.0);

//    // Finds the most salient contours. The complete set of boundary px will be computed by interpolation
//    findContours(gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

//    // Draw contours
//    vector<vector<Point>> vct_contours;

//    for( int k = 1; k < contours.size(); k++ )
//    {
//        vector<Point>vct;
//        vct_contours.push_back(vct);

//        // Computes the missing point by interpolation
//        for( int z = 0; z <= contours[k].size(); z++ )
//        {
//            int z1 = z%contours[k].size();

//            vct_contours[k - 1].push_back(Point(contours[k][z1].y, contours[k][z1].x));

//            // cout << "---------------------------------------------------------------------------" << endl;
//            //  cout << z << "  Native x: " << contours[k][z1].x << "  y: " << contours[k][z1].y << endl;
//            //  cout << "---------------------------------------------------------------------------" << endl;
//            circle(drawing, Point( contours[k][z1].x, contours[k][z1].y), 2, Scalar(0, 255, 255), CV_FILLED, 8,0);

//            if (z > 0)
//            {
//                // int d = (int)sqrt(pow(contours[k][z].x - contours[k][z - 1].x, 2) + pow(contours[k][z].y - contours[k][z - 1].y, 2));
//                int dx = abs(contours[k][z1].x - contours[k][z - 1].x);
//                int dy = abs(contours[k][z1].y - contours[k][z - 1].y);
//                int d = max(dx, dy);

//                float step_x = (contours[k][z1].x - contours[k][z - 1].x)/d;
//                float step_y = (contours[k][z1].y - contours[k][z - 1].y)/d;

//                for (int j = 0; j < d; j++)
//                {
//                    vct_contours[k - 1].push_back(Point(contours[k][z - 1].y + (j * step_y), contours[k][z - 1].x + (j * step_x)));
//                    // cout << z << "  Interpolation x: " << vct_contours[k - 1][vct_contours[k - 1].size() - 1].x << "  y: " << vct_contours[k - 1][vct_contours[k - 1].size() - 1].y << endl;
//                    // circle(drawing, Point(vct_contours[k - 1][vct_contours[k - 1].size() - 1].y, vct_contours[k - 1][vct_contours[k - 1].size() - 1].x), 1, Scalar(255, 0, 0), CV_FILLED, 8,0);
//                }

//                // Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//                // drawContours( drawing, contours, k, color, 2, 8, hierarchy, 0, Point() );
//            }
//        }
//        // Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//        // drawContours( drawing, contours, k, color, 2, 8, hierarchy, 0, Point() );
//    }

//    // Show in a window
//    // namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//    // imshow( "Contours", drawing );
//    // waitKey(0);

//    //--------------------------------------------------------------------
//    // Matches the OpenCv generated boundary with the SLIC generated sPix
//    //--------------------------------------------------------------------
//    const int sz =  m_vct_simg[src_idx].m_vct_spix.size();

//    for (int i = 0; i < sz; i++)
//    {
//        // RGI 31/01/17
//        //  i = 30;
//        // RGI

//        cout << i << endl;

//        p_spix = &(m_vct_simg[src_idx].m_vct_spix[i]);

//        if (!p_spix)
//        {
//            return false;
//        }

//        bool found = false;
//        int cnt_idx = 0;

//        while (found == false && cnt_idx < contours.size())
//        {
//            // Finds the centroid matching with the selected boundary
//            // Makes sure the centroid is properly included in the boundary coordinates

//            // Scans all px belonging to the selected sPix until a px matches with the contours[cnt_idx][0]
//            int px_idx = 0;
//            const int matching_px = 0;

//            while (found == false && px_idx < p_spix->m_spix_pos.size())
//            {
//                cout << "PX IDX:" << px_idx << endl;
//                int cnt_sz = vct_contours.size();

//                if (cnt_idx < vct_contours.size())
//                {
//                    if (vct_contours[cnt_idx][matching_px].x == p_spix->m_spix_pos[px_idx].m_r &&
//                            vct_contours[cnt_idx][matching_px].y == p_spix->m_spix_pos[px_idx].m_c)
//                    {
//                        // circle(drawing, Point(ay, ax), 15, Scalar(0, 0, 255), CV_FILLED, 8,0);
//                        // circle(drawing, Point(p_spix->m_spix_pos[px_idx].m_c, p_spix->m_spix_pos[px_idx].m_r), 10, Scalar(255, 0, 0), CV_FILLED, 8,0);
//                        // circle(drawing, Point(p_spix->m_gy, p_spix->m_gx), 4, Scalar(255, 255, 0), CV_FILLED, 8,0);

//                        // namedWindow("Contours", CV_WINDOW_AUTOSIZE);
//                        // imshow("Contours", drawing);
//                        // waitKey(0);

//                        // Adds the boundary pixels to the sPix
//                        //  for (int cnt_px_idx = 0; cnt_px_idx < vct_contours[cnt_idx].size(); cnt_px_idx++)
//                        {
//                            //circle(drawing, Point(contours[cnt_idx][cnt_px_idx].yx, contours[cnt_idx][cnt_px_idx].y), 4, Scalar(0, 0, 255), CV_FILLED, 8,0);
//                            //   p_spix->m_vct_boundary.push_back(Point(vct_contours[cnt_idx][cnt_px_idx].y, vct_contours[cnt_idx][cnt_px_idx].x));
//                        }
//                        found = true;
//                    }
//                    px_idx++;
//                }
//                else
//                {
//                    cout << "error !!" << endl;
//                }
//            }
//            cnt_idx++;
//        }
//    }
//    return true;
//}

////---------------------------------------------------------------------------------------
//// Converts the  RGB format to OpenCv RGB format to SLIC internal format
////---------------------------------------------------------------------------------------
//void GeometryMetrics::convertRGBToInternal(const Mat src_rgb, Mat &rgb_int)
//{
//    unsigned char *p_src_base = src_rgb.data;
//    unsigned int *p_int_base = (unsigned int *)rgb_int.data;
//    unsigned int *p_int_end = (unsigned int *)rgb_int.data + (rgb_int.rows * rgb_int.cols);

//    unsigned  char *p_src = p_src_base;
//    unsigned int *p_int = p_int_base;

//    while (p_int < p_int_end)
//    {
//        // Red
//        *p_int = *p_src << 16;
//        p_src++;

//        // Green
//        *p_int |= *p_src << 8;
//        p_src++;

//        // Blue
//        *p_int |= *p_src;
//        p_src++;

//        p_int++;
//    }
//}
