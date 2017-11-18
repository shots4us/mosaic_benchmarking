#ifndef OBJECTIVETEST_H
#define OBJECTIVETEST_H

#ifdef IS_WIN
#include <io.h>
#endif

#include "global.h"
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <ctime>
#include <sstream>
#include <math.h>
#include <stdint.h>

#include "distortionmanager.h"
#include "exportdictionary.h"

#ifdef IS_LINUX
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#endif

using namespace std;
using namespace cv;

enum MosaicingMethod
{
    UseInternalAlgorithm = 0,
    UseExternalTool,
    MosaicingMethodNbr
};

enum BenchmarkType
{
    StitchingPipeline,
    MosaicingPipeline,
    GeometryFidelity,
    Nbr
};

enum StitchingType
{
    Nominal = 0,
    Feathering,
    Multiband,
    Poisson,
    Watershed,
    Graphcut,
    Voronoi,
    External,
    None,
    StitchNbr
};

class ObjectiveTest
{
public:
    ObjectiveTest();
    ~ObjectiveTest();

    ObjectiveTest(const string path,
                  const string src_path,
                  const string dst_path,
                  const uint32_t max_img_set_size,
                  const uint32_t shift_nbr_steps,
                  const uint32_t nbr_distortion_types,
                  const uint32_t distortion_range,
                  const uint32_t nbr_distortion_steps,
                  const uint32_t img_size,
                  const BenchmarkType benchmark_pipeline,
                  const bool enable_generation,
                  const bool generate_target_files,
                  const int min_match_array_size,
                  const MosaicingMethod mosaicing_method,
                  const StitchingType stitching_type);

    int recursive(const string path, const unsigned int tid);
    int recursive(const string path);

    bool applyStitchingToVector(const int s_idx, const unsigned int tid);

    bool applyIncrementalDistortions(const Mat rgb_src,
                                     const Rect roi_1,
                                     const Rect roi_2,
                                     const float sub_ratio,
                                     const unsigned int tid);

    bool applyDistortionStack(const Mat rgb_src,
                              const Rect roi_1,
                              const Rect roi_2,
                              Mat &rgb_dist_1,
                              Mat &rgb_dist_2);

    // bool nominalStitching(const Mat rgb_src_1, const Mat rgb_src_2, Mat &rgb_mosaic);

    bool watershedStitching(const Mat rgb_src_1,
                            const Mat rgb_src_2,
                            Mat &rgb_mosaic,
                            Mat &rgb_mosaic_nominal,
                            const bool hide_seam);

    bool comparative_stitching_1(const Mat rgb_src_1,
                                 const Mat rgb_src_2,
                                 const vector<Point2f> scene_corners,
                                 const vector<Point2f> obj_corners,
                                 Mat &rgb_stitch_distort,
                                 Mat &rgb_stitch_nominal_1
                                 , Mat &rgb_stitch_nominal_2,
                                 vector<Point2f> &shared_region,
                                 const bool hide_seam,
                                 const bool generate_outline,
                                 const unsigned int tid);

    bool comparative_stitching_2(const Mat rgb_src_1,
                                 const Mat rgb_src_2,
                                 Mat &rgb_src_distort_2,
                                 Mat &rgb_stitch_distort,
                                 const vector<Point2f> obj_corners,
                                 const vector<Point2f> scene_corners,
                                 const bool hide_seam);

    bool generateGnuPlot(const string export_name,
                         ExportDictionary &xl_dict,
                         const int f_idx,
                         const int d_idx,
                         const bool is_log_scale,
                         const bool skip_row_0);

    bool generateMergedGnuPlot(const string export_name,
                               ExportDictionary &xl_dict,
                               const int d_idx,
                               const bool is_log_scale,
                               const bool skip_row_0);

    string get_stitching_fnct_name(const int fnct_id);

    unsigned int m_nbr_cores;
    unsigned int m_nbr_imgs;
    unsigned int m_idx;                               // Distortion geometry index
    unsigned int m_distortion_id;                     // Distortion type index

    unsigned int m_max_imgset_size;
    unsigned int m_shift_nbr_steps;

    uint32_t m_nbr_distortion_types;
    uint32_t m_distortion_range;
    uint32_t m_nbr_distortion_steps;
    uint32_t m_distortion_step;

    bool m_generate_target_files;

    int m_strength;                          // Distortion strength index

    string m_path;
    string m_src_path;
    string m_dst_path;
    string m_distort_src_path;

    BenchmarkType m_selected_benchmark;
    StitchingType m_stitching_type;
    MosaicingMethod m_mosaicing_method;
    int m_min_match_array_size;

    // Imageset for generation of distort images
    vector<pair<int,string> > m_nominal_image_set;


    //-------------------------------------------------------------
    // Begin Thread parallel variables
    //-------------------------------------------------------------
    vector<unsigned int> m_vct_image_id;
    vector<unsigned int> m_vct_distortion_id;
    vector<pair<int, string> > m_vct_image_set;

    vector<unsigned int> m_image_id;

    // Imageset for benchmark execution
    vector<vector<pair<int,string> > > m_image_set;

    vector<vector<pair<int, string> > > m_vct_roi_1;
    vector<vector<pair<int, string> > > m_vct_roi_2;

    vector<pair<int, string> > vct_roi_undistorted_2;

    //------------------------------------------
    // Incremental Distortion variables
    //------------------------------------------
    vector<int> m_dw_1;
    vector<int> m_dw_2;
    vector<int> m_dh_1;
    vector<int> m_dh_2;
    vector<int> m_dx;
    vector<int> m_dy;
    //-------------------------------------------------------------
    // End Thread parallel variables
    //-------------------------------------------------------------

    vector<vector<vector<double> > > m_psnr_mean;                    // Mean of PSNR for all cominations of Image_1 and Image 2
    vector<vector<vector<double> > > m_ssim_mean;                    // Mean of SSIM for all cominations of Image_1 and Image 2

    vector<vector<vector<double> > > m_psnr_mse;                     // MSE of PSNR for all cominations of Image_1 and Image 2
    vector<vector<vector<double> > > m_ssim_mse;                     // MSE of SSIM for all cominations of Image_1 and Image 2

    int m_geom_mismatch = 0;                               // Counter of geometry mismatches between registered image 1 and 2

    ofstream *m_log_file;

    //------------------------------------------
    // GnuPlot Sheet Exporting Data structures
    //------------------------------------------

    // Cube exporting data structure

    // Mean Multicube
    ExportHeader m_psnr_xl_header;
    ExportTab m_psnr_xl_tabs;
    ExportColumn m_psnr_xl_cols;
    ExportSheet m_psnr_xl_sheet_2;
    ExportDictionary m_psnr_xl_dict;

    ExportHeader m_ssim_xl_header;
    ExportTab m_ssim_xl_tabs;
    ExportColumn m_ssim_xl_cols;
    ExportSheet m_ssim_xl_sheet_2;
    ExportDictionary m_ssim_xl_dict;

    // MSE Mulicube
    ExportHeader m_mse_psnr_xl_header;
    ExportTab m_mse_psnr_xl_tabs;
    ExportColumn m_mse_psnr_xl_cols;
    ExportSheet m_mse_psnr_xl_sheet_2;
    ExportDictionary m_mse_psnr_xl_dict;

    ExportHeader m_mse_ssim_xl_header;
    ExportTab m_mse_ssim_xl_tabs;
    ExportColumn m_mse_ssim_xl_cols;
    ExportSheet m_mse_ssim_xl_sheet_2;
    ExportDictionary m_mse_ssim_xl_dict;
};

#endif // OBJECTIVETEST_H

