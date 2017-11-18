////-------------------------------------------------------------------------------------
//// Project name: COHERENT MOSAICA
////
//// Creation Date: 20/03/2016
//// Created by: Roberto Giudici - OBELIX, UBS, IRISA, COSTEL, Universiy of Rennes 2
//// Description: Super pixel time coherent based Mosaicing algorithm
////-------------------------------------------------------------------------------------

//#ifndef SPIXTOOLS_H
//#define SPIXTOOLS_H

//#include <math.h>
//#include <stdint.h>
//#include <limits.h>
//#include <iostream>
//#include <sstream>

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

//#include "superpixel.h"
//#include "geometry_metrics.h"

//using namespace cv;
//using namespace std;

//enum SPixInfoOverlay
//{
//    INFO_NONE = 0,
//    INFO_ID,
//    INFO_SIZE,
//    INFO_MEAN_LUMA,
//    INFO_GRAVITY,
//    INFO_SIGNATURE,
//    INFO_MATCHING,
//    INFO_COUNT
//};

////-----------------------------------------------------------------------------
//// Class SPixTools
////-----------------------------------------------------------------------------
//class SPixTools
//{
//public:
//    string m_path;

//    SPixTools(string path);

//    bool getImageFromSpix(SuperPixel *p_spix, const Mat rgb_src, Mat &spix_img);

//    bool dumpSpix(const int img_idx,
//                  const int spix_idx,
//                  const Mat &rgb_src,
//                  const SuperPixel *p_spix);

//    bool dumpSpixImage(const GeometryMetrics &msa,
//                       const int spix_idx,
//                       const string path_dst,
//                       const enum SPixInfoOverlay info_type);


//    bool dumpPairwiseOverlap(const GeometryMetrics &msa,
//                             const string path_dst,
//                             const int simg_idx_0,
//                             const int simg_idx_1,
//                             const vector<float> vct_spix_overlap);

//    bool dumpAllPairwiseOverlaps(const GeometryMetrics &msa, const string path_dst);

//    bool dumpSpixImageStack(const GeometryMetrics &msa,
//                            const vector<SuperImage> &vct_simg,
//                            const string path_dst,
//                            const enum SPixInfoOverlay info_type);

//    bool dumpSPixDistanceVectors(Mat rgb_src,
//                                 GeometryMetrics &msa,
//                                 const int simg_idx,
//                                 const int spix_idx,
//                                 vector<float *> vct_label_dist_min);
//};

//#endif // SPIXTOOLS_H
