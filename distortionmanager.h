#ifndef DISTORTIONMANAGER_H
#define DISTORTIONMANAGER_H

#include "global.h"
#include "hsvmanager.h"
#include "tools.h"

#ifdef IS_WIN
//#include <io.h>
#endif

#ifdef IS_LINUX
#include <sys/types.h>
#include <sys/stat.h>
#endif

#include <vector>
#include <map>
#include <iostream>
#include <ctime>
#include <sstream>
#include <math.h>
#include <stdint.h>

using namespace cv;
using namespace std;

typedef unsigned int uint32_t;

// #define DEBUG_MODE 1
#define HAS_TIMER 1

enum DirOrder
{
    SET_TO_OPERATION = 0,
    OPERATION_TO_SET
};

enum Distortions
{
    DISTORTION_LUMINANCE = 0,
    DISTORTION_SATURATION,
    DISTORTION_HUE,
    DISTORTION_GAUSSIAN_BLUR,
    DISTORTION_MOTION_BLUR,
    DISTORTION_SALT_PEPPER_NOISE,
    DISTORTION_CAMERA_NOISE,
    DISTORTION_SHIFT,
    DISTORTION_ROTATION,
    DISTORTION_RESCALE,
    //  DISTORTION_PERSPECTIVE,
    DISTORTION_PROJECTION_3D,
    // DISTORTION_MOVING_OBJECT,
    DISTORTION_VIGNETTING,
    DISTORTION_BARREL,
    //  DISTORTION_SHADOW,
    DISTORTION_COUNT
};

enum DistortionGeometry
{
    GEOMETRY_CONSTANT = 0,
    GEOMETRY_LINEAR,
    GEOMETRY_QUADRATIC,
    GEOMETRY_VIGNETTING,
    GEOMETRY_BARREL
};

enum BarrelMode
{
    BARREL_CONVEX,
    BARREL_CONCAVE
};

typedef vector<Mat> StepDim;
typedef vector<StepDim> ImageCube;

class DistortionManager
{
public:
    DistortionManager();
    DistortionManager(const string path, const DirOrder dir_order, const Mat rgb_src, const uint32_t nbr_distortions, const uint32_t nbr_steps);
    ~DistortionManager();

    bool vignetting(const Mat rgb_src, Mat &rgb_dst, const float strength, const float speed, Mat &mask_out, const float quality = 1.0);
    bool testVignettingDistortion(const uint32_t cycles, const uint32_t step);
    bool barrel(const Mat rgb_src, Mat &rgb_distort, const float strength,const BarrelMode mode);
    bool testBarrelDistortion(const uint32_t cycles, const uint32_t step);
    void setDistortionProperties();
    bool applyAllDistortionsToImage(const uint32_t set_id);
    void rotate3D(const Mat &input, Mat &output, float alpha, float beta, float gamma, float dx, const float dy, const float dz,const float f);
    void perspective();
    bool imageResize(const float size_ratio);
    bool selectRandomRegion();
    bool applyDistortionSteps(const int k);
    bool applyMultistepDistortion();
    string setFileName(const string file_name);
    string setFileName2(const string path_name, const string file_name);
    string get_distort_name(const int distort_id);

    // protected:
    uint32_t m_set_id = 0;
    uint32_t m_step_idx = 0;
    uint32_t m_distortion_id = 0;
    uint32_t m_strength_id = 0;

    uint32_t m_nbr_distortions;

    float m_vect_range_min[DISTORTION_COUNT];
    float m_vect_range_max[DISTORTION_COUNT];
    float m_vect_strength[DISTORTION_COUNT];
    float m_vect_step[DISTORTION_COUNT];

    uint32_t m_clip_cnt[3];
    uint32_t m_clip_ratio[3];

    DirOrder m_dir_order = OPERATION_TO_SET;                             // Sets the order of directory creation : Se/Operation or Operation/Set

    Mat m_rgb_src;
    Mat m_rgb_dst;

    ImageCube m_rgb_distort;                                             // Cube of images

    uint32_t m_nbr_distortion_steps = 0;
    uint32_t m_distortion_step = 0;

    string m_path;
    string m_distort_name;

    // Timers
    clock_t m_distort_img_ck_start;
    clock_t m_distort_oper_ck_start;
    clock_t m_distort_step_ck_start;
    clock_t m_distort_set_ck_start;
    clock_t m_distort_set_ck_stop;
    clock_t m_distort_img_ck_stop;
    clock_t m_distort_oper_ck_stop;
    clock_t m_distort_step_ck_stop;

    // Stats
    double m_distort_set_time = 0.0;
    double m_distort_img_time = 0.0;
    double m_distort_oper_time = 0.0;
    double m_distort_single_time = 0.0;

    // private:
};

#endif // DISTORSIONMANAGER_H
