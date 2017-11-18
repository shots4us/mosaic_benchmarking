//-------------------------------------------------------------------------------------
// Project name: COHERENT MOSAICA
//
// Creation Date: 20/03/2016
// Created by: Roberto Giudici - OBELIX, UBS, IRISA, COSTEL, Universiy of Rennes 2
// Description: Super pixel time coherent based Mosaicing algorithm
//-------------------------------------------------------------------------------------

#include "superpixel.h"

GeoImage::GeoImage(const Mat rgb_img)
{
    m_rgb_img = rgb_img;
}

int GeoImage::rows()
{
    return m_rgb_img.rows;
}

int GeoImage::cols()
{
    return m_rgb_img.cols;
}

uchar *GeoImage::data()
{
    return m_rgb_img.data;
}

void GeoImage::copyTo(Mat &dst)
{
    m_rgb_img.copyTo(dst);
}

ImageSet::ImageSet()
{
}

SuperPixel::SuperPixel()
{
    m_label = -1;
}

SuperPixel::SuperPixel(const int32_t label, PixPos pos)
{
    m_label = label;

    // A superpixel is composed by at least one pixel
    m_spix_pos.push_back(pos);
}


SPixSignature::SPixSignature()
{
}

void SPixSignature::set_wights(const float weight_size,
                               const float weight_mean_luma,
                               const float weight_elongation,
                               const float weight_distance)
{
    m_weight_size = weight_size;
    m_weight_mean_luma = weight_mean_luma;
    m_weight_elongation = weight_elongation;
    m_weight_distance = weight_distance;
}

const float SPixSignature::calc_signature(SuperPixel *spix)
{
    const int height = abs(spix->m_max_x - spix->m_min_x);
    const int width = abs(spix->m_max_y - spix->m_min_y);

    spix->m_signature = (m_weight_size * height * width) +
            (m_weight_mean_luma * spix->m_mean_luma) +
            (m_weight_elongation * spix->m_elongation) +
            (m_weight_distance * spix->m_distance);

    return spix->m_signature;
}

SuperImage::SuperImage()
{
}

SuperImage::SuperImage(GeoImage &geo_img, const uint32_t id): SPixSignature()
{
    m_geo_img = &geo_img;
    m_id = id;
}

OverlapImage::OverlapImage()
{
}

OverlapImage::OverlapImage(const uint8_t level_0, const uint8_t level_1, const uint32_t id, const uint32_t nbr_spix)
{
    m_level_0 = level_0;
    m_level_1 = level_1;
    m_id = id;
    m_vct_spix = vector<SuperPixel>(nbr_spix);
}
