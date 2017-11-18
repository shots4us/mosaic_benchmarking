#ifndef STITCH_GRAPHCUT_H
#define STITCH_GRAPHCUT_H

//#include <opencv2\opencv.hpp>


#include "watershed_soille.h"
#include "qm.h"
#include "image_compose.h"

#include "robustmatcher.h"
#include "stitching_detailed.h"
#include "blending_test.h"
#include "poisson_blender_new.h"
#include "geometry_metrics.h"
#include "stitch_graphcut.h"
//#include "gcoptimization.h"

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/opencv_modules.hpp"

#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

#define ENABLE_GRAPHCUT_LOG 1
#define HAVE_OPENCV_NONFREE 1

using namespace cv;
using namespace std;
using namespace qm;

class StitchGraphcut
{
public:
    static string m_path;

    static int create_stitch_graphcut( vector<Mat> images,
                                      const vector<Point> shift,
                                      const string seam_find_type,
                                      Mat &img_stitch);

    static int unit_test(const int test_id,
                         const string img_name_1,
                         const string img_name_2,
                         const string img_name_nominal,
                         const Point shift_1,
                         const Point shift_2,
                         float &psnr_val,
                         float &ssim_val);

    static int unit_test_plan(const string path);
    static int stitch_opencv(const string path);
};

#endif // GRAPHCUT_H
