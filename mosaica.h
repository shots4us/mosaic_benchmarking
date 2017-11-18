////-------------------------------------------------------------------------------------
//// Project name: COHERENT MOSAICA
////
//// Creation Date: 20/03/2016
//// Created by: Roberto Giudici - OBELIX, UBS, IRISA, COSTEL, Universiy of Rennes 2
//// Description: Super pixel time coherent based Mosaicing algorithm
////-------------------------------------------------------------------------------------

//#ifndef MOSAICA_H
//#define MOSAICA_H

//#include <math.h>
//#include <stdint.h>
//#include <limits.h>
//#include <iostream>
//#include <limits>
//#include <sstream>
//#include <vector>
//#include <map>
//#include "global.h"
//#include "superpixel.h"
//#include "exportdictionary.h"
//#include "SLIC.h"

//using namespace cv;
//using namespace std;

//struct PairedSuperPixel
//{
//public :
//    PairedSuperPixel()
//    {
//    }
//    PairedSuperPixel(const int spix_id, const float dist)
//    {
//        m_spix_id = spix_id;
//        m_dist = dist;
//    }

//    int m_spix_id;
//    float m_dist;
//};

//struct SuperPixelPair
//{
//public :
//    SuperPixelPair()
//    {
//    }
//    SuperPixelPair(const int id_0, const int id_1)
//    {
//        m_id_0 = id_0;
//        m_id_1 = id_1;
//    }

//    int m_simg_id_0;
//    int m_id_0;
//    int m_id_1;
//};

//struct SPixPair
//{
//public :
//    SPixPair()
//    {
//    }
//    SPixPair(const int id_0, const int simg_0, const int id_1, const int simg_1, const int dist)
//    {
//        m_id_0 = id_0;
//        m_simg_id_0 = simg_0;
//        m_id_1 = id_1;
//        m_simg_id_1 = simg_1;
//        m_dist = dist;
//    }

//    int m_simg_id_0;
//    int m_simg_id_1;
//    int m_id_0;
//    int m_id_1;
//    int m_dist;
//};

//struct ObjectThread
//{
//public:
//    ObjectThread()
//    {
//    }

//    vector<pair<int, int> > vct_obj_spix;
//};

//struct ObjNode
//{
//public:
//    int simg_id;
//    int spix_id;
//    float weight;
//    float weight_sum;
//};

//// <sPix><PathId><NodeId>
//// typedef vector<vector<ObjNode>> weight_sum;
//typedef vector<vector<ObjNode> > SPixPath;

//struct ObjPath
//{
//public:
//    ObjPath()
//    {
//    }

//    vector<ObjNode> m_vct_pair;

//     float m_weight;
//};

//struct ObjThread
//{
//public :
//    ObjThread()
//    {
//    }

//    vector<ObjPath> m_vct_path;

//    // Overall Length Confidence
//    float m_length_confidence;

//    // Aggregation Confidence
//    float m_aggregation_confidence;

//    // Isomorphism Confidence
//    float m_isomorph;
//};


//typedef vector<vector<vector<SPixPair> > > SPixPairSet;
//typedef vector<vector<vector<SuperPixelPair > > > PairedDistanceType;

//void RGBToGray(const Mat src, Mat &dst);

////-----------------------------------------------------------------------------
//// Class MosaicStack
//// Time coherent submosaic stack
////-----------------------------------------------------------------------------
//class MosaicStack
//{
//public:

//    vector<Mat> m_mosa_stack;

//    MosaicStack();

//};

////-----------------------------------------------------------------------------
//// Class Mosaica
////-----------------------------------------------------------------------------
//class Mosaica
//{
//public:

//    int m_max_pairing_dist = 0;

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

//    vector<vector<float> > sm_vct_ovr_nbr;
//    // vector<vector<float>> m_vct_ovr_ratio;
//    //vector<vector<float>> m_vct_belong_ratio;

//    // SuperImage for overlapping sPix
//    //  vector<SuperImage> m_vct_overlap;
//    vector<vector<int> > m_vct_overlap;

//    vector<vector<float> > m_vct_spix_matches;

//    // Time coherent sumbosaic stack
//    // MosaicStack m_mosa_stack;
//    vector<Mat> m_mosa_stack;

//    Mosaica();
//    bool ComputeSpixBoundaries(const int src_idx);
//    bool ComputeMORE(const int src_idx);
//    bool CreateSuperPixels(const int src_idx, const int spcount, const double compactness);
//    void convertRGBToInternal(const Mat src_rgb, Mat &rgb_int);
//    bool spixMatch();
//    bool detectOOI(const int simg_idx);
//    bool CreateSPixPairingSet(const int simg_idx_0, const int spix_idx, SPixPairSet &vct_spix_pairs);
//    bool ObjThreadDetection(vector<vector<SPixPair> > intfrm_obj_thread);
//    bool MotionAffinityMatching(PairedDistanceType intfrm_paired_dist);
//    bool createHueHistogram(const int simg_idx, const int spix_idx, Mat src, Mat &hue_histogram);
//    bool createHueHistogram(const int simg_idx, const int spix_idx, Mat &hue_histogram);
//    bool SPixHueMatching(const int simg_idx_0, const int simg_idx_1, const int spix_idx_0, const int spix_idx_1, float &hue_similarity);
//    bool RgbToHue(const int simg_idx, Mat &hue_dst);
//    bool SPixHueSAD(const int simg_idx_0, const int spix_idx_0, const int simg_idx_1, const int spix_idx_1, float &hue_sad);
//    bool SPixHueHistogramDiff(const int simg_idx_0, const int simg_idx_1, const int spix_idx_0, const int spix_idx_1, float &hue_diff);
//    bool RgbSAD(const int simg_idx_0, const int spix_idx_0, const int simg_idx_1, const int spix_idx_1);
//    bool spixOverlapMatch();
//    bool spixOverlap();
//    bool crossframOverlapPenetration(const int simg_idx_0, const int simg_idx_1, vector<float> vct_cross_overlap);
//    bool stackMerge(const int t0, const int t1);
//};

//#endif // MOSAICA_H
