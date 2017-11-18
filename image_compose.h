#ifndef image_compose_H
#define image_compose_H

#include "test.h"
#include "tools.h"

#include "hsvmanager.h"
#include "distortionmanager.h"
#include "exportdictionary.h"
#include "watershed_soille.h"
#include "qm.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef IS_WIN
#include <io.h>
#endif

#include <vector>
#include <map>
#include <iostream>
#include <ctime>
#include <sstream>
#include <math.h>
#include <stdint.h>

#define HAS_TIMER 1                                    // Enables timer stats
// #define EXPORT_UNIT_TEST 1                          // Enables Excel Exporting Unit Test
// #define ENABLE_EXCEL 1                                 // Enables Excel exporting
#define ENABLE_GNU_PLOT 1                              // Enables GNU Plot exporting
// #define UNIT_TEST 1                                 // Enables Unit Testing

#define KERNEL_WIDTH 3
#define KERNEL_HEIGHT 3

using namespace cv;
using namespace std;
using namespace qm;


// typedef vector<string> SetArea;

typedef vector<float> SingleDistortArea;                  // Every entry of the vector is a signle step distortion
typedef vector<SingleDistortArea> DistortSetArea;         // Every entry of the vector is a vector of single step distortions
typedef vector<DistortSetArea> SetArea;                   // ImageSet area aggregate by distortion type. BEWARE before using it !!

typedef vector<float> DistortMean;
typedef vector<DistortMean> SetMean;

//struct DistArea
//{
//    vector<float> m_vct_step;
//    float m_step_area_mean;
//};

//struct ImageArea
//{
//    vector<DistArea> m_vct_dist_area;
//    float m_dist_area_mean;
//};

//struct SetArea
//{
//      vector<ImageArea> m_vct_img_area;
//      float m_img_area_mean;
//};


// Every entry of the vector is a vector of single step distortions
// typedef vector<SetArea> SetArea;              // ImageSet area ggregate by distortion type. BEWARE before using it !!

// typedef vector<float> DistortMean;
// typedef vector<DistortMean> SetMean;

class ImageCompose
{
public:
    // Point image_offset[8];
    Mat *m_ptr_marker;


public:
    ImageCompose();
    ImageCompose(const uint32_t img_set_size, const uint32_t nbr_distortions, const uint32_t nbr_distort_steps, const string path);
    ~ImageCompose();

    void setDirOrder(const DirOrder order);
    bool extractIdFromFileName(const string file_name, int &set_id);
    string setFileName(const string file_name);
    string setFileName2(const string path_name, const string file_name, const int hide_numbers = 0);
    void initCube(const uint32_t img_set_size);
    bool generateDistortedImageset(const string sub_path);
    bool applyMaskToComposite(const Mat rgb_nom, const Mat rgb_dist, Mat &patch, Mat &rgb_composed);
    bool computeInterseam(const int k);
    bool exportToFile(SetArea area_cube, vector<float> mean_images);
    void incDistortionStrength();

    bool distance(const Mat &in, Mat &dist);
    bool compose(vector<Mat> gray_in);
    void projection(const Mat grad_nom, const Mat grad_dist, Mat &mask);
   // void projection2(const Mat grad_nom, const Mat grad_dist, Mat &mask);
    void setDistortionProperties(const Distortions distort_id);
    bool composeBenchmark(const uint32_t k);

    string m_distort_name = "";

    vector<string> m_image_set;

    DirOrder m_dir_order = OPERATION_TO_SET;                             // Sets the order of directory creation : Se/Operation or Operation/Set

    uint32_t m_set_id = 0;                                                   // Identifier of the current image set
    uint32_t m_distort_id = 0;
    uint32_t m_step_id = 0;

    uint32_t m_nbr_distortions;

    // Watershed metrics
    SetArea m_area_cube;

    uint32_t m_file_idx;

    vector<Mat> m_mask_out;
    vector<Mat> m_step_interseam;
    Mat m_step_interseam_overlay;

    // Sheet exporting data structure
    ExportHeader xl_header;
    ExportColumn xl_cols;
    ExportSheet xl_sheet;
    ExportDictionary xl_dict;

    vector<Mat> m_step_interseam_residual;
    DistortionManager m_dist_man;

    string m_path;
    Mat m_rgb_src;

    vector<Mat> m_distort_img;                                            // Stack of distorted images to compose in one composition exection
    vector<Mat> m_gray_vect_in;

    clock_t m_generate_gradient_ck_start;
    clock_t m_compose_ck_start;

    clock_t m_distort_set_ck_start;
    clock_t m_distort_set_ck_stop;
    clock_t m_distort_img_ck_stop;
    clock_t m_distort_type_ck_stop;
    clock_t m_distort_step_ck_stop;
    clock_t m_generate_gradient_ck_stop;
    clock_t m_compose_ck_stop;

    // Stats
    double m_distort_set_time = 0.0;
    //    double m_distort_img_time = 0.0;
    //    double m_distort_type_time = 0.0;
    //    double m_distort_single_time = 0.0;
    double m_generate_gradient_time = 0.0;
    double m_compose_time = 0.0;
};

class TestCompose
{
public:
    ImageCompose m_image_compose;

public:
    void showFiles();

    int recursive(const string path, int max_img_set_size);
    TestCompose();
    TestCompose(const string path, const string sub_path, const uint32_t max_img_set_size, const uint32_t nbr_distortions, const uint32_t nbr_steps, const bool enable_generation = 0);
    ~TestCompose();

protected:

    // vector<string> m_image_set;
};

class ObjTest
{
public:
    ImageCompose m_image_compose;

public:
    void showFiles();

    int recursive(const string path, int max_img_set_size);
    ObjTest();
    ObjTest(const string path, const string sub_path, const uint32_t max_img_set_size, const uint32_t nbr_distortions, const uint32_t nbr_steps, const bool enable_generation = 0);
    ~ObjTest();
    void match_surf(const Mat rgb_src_1, const Mat rgb_src_2, Mat &rgb_registered);
protected:

    // vector<string> m_image_set;
};
#endif // image_compose_H

