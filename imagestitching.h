#ifndef IMAGESTITCHING_H
#define IMAGESTITCHING_H

#include "global.h"

using namespace std;
using namespace cv;
using namespace cv::detail;

class ImageStitching
{
public:
    ImageStitching();
    ~ImageStitching();

    bool nominalStitching(const Mat rgb_1, Mat &rgb_2);

    bool watershed(const Mat rgb_src_1,
                   const Mat rgb_src_2,
                   const Mat marker,
                   Mat &rgb_stitch,
                   const bool remove_seam);
    
    bool blending(const Mat rgb_src_1,
                  const Mat rgb_src_2,
                  const Point corner_1,
                  const Point corner_2,
                  Mat &rgb_stitch,
                  const int blend_type,
                  const float blend_strength);

    bool poisson(const Mat rgb_src_1,
                 const Mat rgb_src_2,
                 const Point corners_1,
                 const Point corners_2,
                 Mat &rgb_stitch);
    
    bool graphcut(const Mat rgb_src_1,
                  const Mat rgb_src_2,
                  const Point corners_1,
                  const Point corners_2,
                  const string seam_find_type,
                  Mat &rgb_stitch);
};
#endif // IMAGESTITCHING_H
