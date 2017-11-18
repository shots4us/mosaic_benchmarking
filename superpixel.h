//-------------------------------------------------------------------------------------
// Project name: COHERENT MOSAICA
//
// Creation Date: 20/03/2016
// Created by: Roberto Giudici - OBELIX, UBS, IRISA, COSTEL, Universiy of Rennes 2
// Description: Super pixel time coherent based Mosaicing algorithm
//-------------------------------------------------------------------------------------

#ifndef SUPERPIXEL_H
#define SUPERPIXEL_H

#include <math.h>
#include <stdint.h>
#include <limits.h>
#include <iostream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

//-----------------------------------------------------------------------------
// Class LabelledPixel
//-----------------------------------------------------------------------------

class LabelledPixel
{
public:
    int m_r;
    int m_c;
    int m_label;

    LabelledPixel()
    {
        m_r = 0;
        m_c = 0;
        m_label = -1;
    }
};

//-----------------------------------------------------------------------------
// Class PixPos
// Coordinates expressed in Cartesian metric
//-----------------------------------------------------------------------------
class PixPos
{
public:
    int m_r;
    int m_c;

    PixPos()
    {
        m_r = 0;
        m_c = 0;
    }

    PixPos(const int r, const int c)
    {
        m_r = r;
        m_c = c;
    }
};

//-----------------------------------------------------------------------------
// Class Image
// Extends the OpenCv native Mat class with EXIF and georeferencing features
//-----------------------------------------------------------------------------
class GeoImage
{
public:

    Mat m_rgb_img;
    int64_t m_timestamp = 0;

    GeoImage(const Mat rgb_img);
    int rows();
    int cols();
    uchar *data();
    void copyTo(Mat &dst);
};

//-----------------------------------------------------------------------------
// Class ImageSet
//-----------------------------------------------------------------------------
class ImageSet
{
public:

    int64_t m_timestamp = 0;

    vector<Mat> m_vct_rgb_src;

    ImageSet();
};

//-----------------------------------------------------------------------------
// Class SuperImageStack
// Stack of SuperImages
//-----------------------------------------------------------------------------
class SuperImageStack
{
public:
    SuperImageStack();
};


//-----------------------------------------------------------------------------
// Object Of Interest Class (OOI)
//-----------------------------------------------------------------------------
class OOI
{
public :
    OOI()
    {
    }

    vector<int> m_vct_spix;
};

//-----------------------------------------------------------------------------
// Class SuperPixel
//-----------------------------------------------------------------------------
class SuperPixel
{
public:

    int32_t m_label = 0;

    int32_t m_min_x = INT_MAX;
    int32_t m_min_y = INT_MAX;
    int32_t m_max_x = 0;
    int32_t m_max_y = 0;

    // OverlapGraph *m_ptr_ovr_graph;
    // vector <int> m_map_ptr_spix;

    // Vector of pairwise overlap ratio between two sPix
    vector<float> m_vct_overlap_ratio;

    // Vector of pairwise belong
    // vector<float> m_vct_belong;

    // Gravity
    uint32_t m_gx = 0;
    uint32_t m_gy = 0;

    // mean luminance
    uint32_t m_mean_luma = 0;

    // Elongation
    float m_elongation = 0;

    // Cartesian distance from the origin
    uint32_t m_distance = 0;

    // Signature
    float m_signature = 0.0;

    // Vector of cartesian (x, y) coordinates of the pixels belonging to the sPix
    vector<PixPos>m_spix_pos;

    // Vector of cartesian bounday coordinates
    vector<Point>m_vct_boundary;

    // Vector of linear coordinates of the pixels belonging to the sPix
    // vector<uint64_t> m_spix_lin_pos;

    SuperPixel();
    SuperPixel(const int32_t label, PixPos pos);
};

//-----------------------------------------------------------------------------
// Class SPixSignature
//-----------------------------------------------------------------------------
class SPixSignature
{
public:
    SPixSignature();

    void set_wights(const float weight_size,
                    const float weight_mean_luma,
                    const float weight_elongation,
                    const float weight_distance);

    const float calc_signature(SuperPixel *spix);

protected:

    float m_weight_size = 0.0;
    float m_weight_mean_luma = 0.0;
    float m_weight_elongation = 0.0;
    float m_weight_distance = 0.0;
};


//-----------------------------------------------------------------------------
// Class OverlapImage
//-----------------------------------------------------------------------------
class OverlapImage
{
public:
    OverlapImage();
    OverlapImage(const uint8_t level_0, const uint8_t level_1, const uint32_t id, const uint32_t nbr_spix);

    uint32_t m_level_0;
    uint32_t m_level_1;
    uint32_t m_id = 0;
    vector<SuperPixel> m_vct_spix;
};

//-----------------------------------------------------------------------------
// Class SuperImage
//-----------------------------------------------------------------------------
class SuperImage: public SPixSignature
{
public:
    SuperImage();
    SuperImage(GeoImage &geo_img, const uint32_t id);

    uint32_t m_id = 0;
    uint64_t m_timestamp = 0;

    // Reference to the source image
    GeoImage *m_geo_img;

    vector<SuperPixel>m_vct_spix;

    // Stats

    // Size mean
    float m_mean_size = 0.0;

    // Gravity
    float m_mean_gx = 0.0;
    float m_mean_gy = 0.0;

    // Mean luminance mean
    float m_mean_luma = 0.0;

    // Mean Elongation mean
    float m_elongation = 0.0;

    // Mean Cartesian distance from the origin
    float m_distance = 0.0;
};


#endif // SUPERPIXEL_H
