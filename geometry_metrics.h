//#ifndef GEOMETRY_METRICS_H
//#define GEOMETRY_METRICS_H

////-------------------------------------------------------------------------------------
//// Project name: COHERENT MOSAICA
////
//// Creation Date: 20/03/2016
//// Created by: Roberto Giudici - OBELIX, UBS, IRISA, COSTEL, Universiy of Rennes 2
//// Description: Super pFxel time coherent based Mosaicing algorithm
////-------------------------------------------------------------------------------------

//#include "global.h"
//#include "geometry_metrics.h"
//#include "mosaicatools.h"
//#include "tools.h"
//#include <queue>
//#include <stack>
//#include <algorithm>

//#include <stdio.h>      /* printf, scanf, puts, NULL */
//#include <stdlib.h>     /* srand, rand */
//#include <time.h>       /* time */
//#include <iostream>
//#include <fstream>
//#include <string>

//#define NBR_IMG 4

//using namespace std;
//using namespace cv;
//using namespace cv::detail;


//class MaxSGF
//{
//public:
//    MaxSGF(float sgf, float sub, float r2b)
//    {
//        m_sgf = sgf;
//        m_sub = sub;
//        m_r2p = r2b;
//    }

//    float m_sgf;
//    float m_sub;
//    float m_r2p;
//};

//class GeometryMetrics
//{
//public:

//    string m_path;

//  //  Point m_img_shift;

//    // User selected ROI
//    vector<Rect> m_vct_roi;

//    // Object Of Interest vector
//    vector<OOI> m_vct_ooi;
//    typedef vector<int> SPixLabels;

//    // Source imageset
//    vector<Mat> m_vct_rgb_src;

//    // Dest Slic imageset
//    vector<Mat> m_vct_rgb_dst;

//    // Registration shift vector
//    vector<Point> m_vct_src_shift;

//    // Vector of label arrays
//    vector<int *> m_vct_labels;

//    // Super imageset
//    vector<SuperImage> m_vct_simg;

//    GeometryMetrics(const string path);

//    // SPix SIFT Stats
//    int m_good_match_count = 0;
//    int m_bad_match_count = 0;
//    int m_match_failure_count = 0;

//    //-----------------------------------------------------------------------------
//    // Begin Block of Hue detection managment
//    //-----------------------------------------------------------------------------
//    bool createHueHistogram(const int simg_idx, const int spix_idx, Mat src, Mat &hue_histogram);
//    bool createHueHistogram(const int simg_idx, const int spix_idx, Mat &hue_histogram);
//    bool SPixHueMatching(const int simg_idx_0, const int simg_idx_1, const int spix_idx_0, const int spix_idx_1, float &hue_similarity);
//    bool RgbToHue(const int simg_idx, Mat &hue_dst);
//    bool SPixHueSAD(const int simg_idx_0, const int spix_idx_0, const int simg_idx_1, const int spix_idx_1, float &hue_sad);
//    //-----------------------------------------------------------------------------
//    // End Block of Hue detection managment
//    //-----------------------------------------------------------------------------

//    float compute_keypoint_weight(const KeyPoint &keyp,
//                                  const float response_weight,
//                                  const float size_weight,
//                                  const float min_response,
//                                  const float max_response,
//                                  const float min_size,
//                                  const float max_size);

//    //---------------------------------------------------------------------------------------
//    // 1. Unit Test
//    //---------------------------------------------------------------------------------------
//    // Computes the unit test between a nominal mosaic image and a distortd mosaic image
//    // This test is statically computed on a fixed set of used defined parameters
//    void unit_test(const Mat rgb_src_0,
//                   const Mat rgb_src_1,
//                   const Mat rgb_src_mosaic_1,
//                   const Mat rgb_src_mosaic_2);

//    //---------------------------------------------------------------------------------------
//    // 2. Optimal Test
//    //---------------------------------------------------------------------------------------
//    // Computes the mean distance error among all sets of relevant Mathing Points found in each Super Pixel
//    void optimal_test(const Mat rgb_src_0, const Mat rgb_src_1, const Mat rgb_src_mosaic_1, const Mat rgb_src_mosaic_2);
//    float ghosting_fidelity(vector<vector<KeyPoint> >robust_keypoint_img_0, const int img_idx, Point &v_g);
//    float stitch_geometry_fidelity(const float gain, const float reach_to_prec, const float nbr_robust_keypts); //const Mat rgb_img_0, const Mat rgb_img_1, const Mat rgb_img_pano);
//    void convertInternalToRGB(const Mat rgb_int, Mat rgb_dst);
//    bool CreateSuperPixels(const int src_idx, const int spcount, const double compactness);
//    bool CreateSPixPairingSet(const int simg_idx_0, const int simg_idx_1, const int spix_idx, SPixPairSet &vct_spix_pairs);
//    float compute_euclidean_distance(const int spix_idx_0, const int spix_idx_1);
//    void sort_spix_size(const int img_idx, vector<pair<int, int> > &sorted_spix_size);
//    bool ComputeSpixBoundaries(const int src_idx);
//    void convertRGBToInternal(const Mat src_rgb, Mat &rgb_int);
//};
//#endif // GEOMETRY_METRICS_H
