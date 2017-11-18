#include "objectivetest.h"
#include "imagestitching.h"
#include "qm.h"
#include "stitching.h"
#include "robustmatcher.h"
#include "gnuplotexport.h"
#include "stitch_graphcut.h"

#include <thread>
#include <algorithm>

#ifdef IS_WIN
#include <direct.h>
#include <windows.h>
#endif

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>
#include <fstream>
#include <string>

#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;
using namespace qm;

#define ENABLE_GNU_PLOT 1

//------------------------------------------------------------------------------------------------------------
// Multithread execution functions
//------------------------------------------------------------------------------------------------------------
struct ThdParams_1
{
public:
    
    ObjectiveTest *m_objective_test;
    Mat m_rgb_src;
    Rect m_roi_1;
    Rect m_roi_2;
    unsigned int m_loops_per_thread;
    unsigned int m_nbr_threads;
    unsigned int m_min_img_size;
    unsigned int m_image_idx;
    unsigned int m_distortion_idx;
    vector<pair<int, string> > vct_roi_undistorted_2;
    unsigned int m_idx;
    unsigned int m_tid;
    
    double m_psnr;
    double m_ssim;
    vector<double> m_mse_psnr;
    vector<double> m_mse_ssim;
};

//------------------------------------------------------------------------------------------------------------

void *IncrementalDistortionParallelize(void *p_thd_params);
void *IncrementalDistortionParallelize(void *p_thd_params)
{
    ThdParams_1 *thd_params = NULL;
    thd_params = ((ThdParams_1 *)p_thd_params);
    const unsigned int tid = thd_params->m_tid;
    
    if (thd_params->m_objective_test->m_nominal_image_set.size() == 0)
    {
        cout << "Image set size = 0 !!" << endl;
        exit(0);
    }
    
    for (int idx = 0; idx < thd_params->m_loops_per_thread; idx++)
    {
        const unsigned int img_idx = idx + (thd_params->m_tid * thd_params->m_loops_per_thread);
        
        if (img_idx < thd_params->m_objective_test->m_nominal_image_set.size())
        {
            thd_params->m_objective_test->m_vct_image_id[tid] = img_idx;
            const string file_ref_in(thd_params->m_objective_test->m_nominal_image_set[img_idx].second);
            const Mat rgb_src = imread(file_ref_in, CV_LOAD_IMAGE_COLOR);
            
            if (rgb_src.rows == 0 || rgb_src.cols == 0)
            {
#ifdef IS_VERBOSE
                cout << "Source file: " << file_ref_in << " is empty !!" << endl;
#endif
                exit(-1);
            }
            const int test_type = 0;
            int i = 0;
            
            //----------------------------------------------------------------------------------
            // Phase 1: Begin of generation of distort ROI images
            //----------------------------------------------------------------------------------
            // Number of shifts
            //     const int  thd_params->m_shift_nbr_steps = 10;
            
            // The shift step is expressed in % of the original image size
            Point shift_step(1, 1);
            
            // Cropping ratio. It should correspond to the maximum deviation caused by rotation and perspective.
            const float sub_ratio = 0.5;
            const int dw_1 = thd_params->m_min_img_size;
            
            // Minimal overlap = 2/3 of roi_1
            const float overlap_min = (int)dw_1 * 0.2;
            
            const int dh_1 = thd_params->m_min_img_size;
            const int dw_2 = thd_params->m_min_img_size;
            const int dh_2 = thd_params->m_min_img_size;
            
            thd_params->m_objective_test->m_dh_1[tid] = dh_1;
            thd_params->m_objective_test->m_dw_1[tid] = dw_1;
            
            thd_params->m_objective_test->m_dh_2[tid] = dh_2;
            thd_params->m_objective_test->m_dw_2[tid] = dw_2;
            
            vector<pair<int, string> > vct_roi_1;
            vector<pair<int, string> > vct_roi_2;
            
            thd_params->m_objective_test->m_vct_roi_1.push_back(vct_roi_1);
            thd_params->m_objective_test->m_vct_roi_2.push_back(vct_roi_2);

            shift_step.x = ((rgb_src.cols - dw_2) / 2) / thd_params->m_objective_test->m_shift_nbr_steps;
            shift_step.y = ((rgb_src.rows - dh_2) / 2) / thd_params->m_objective_test->m_shift_nbr_steps;
            
            const int x0 = (rgb_src.cols - dw_2) / 2;
            const int y0 = (rgb_src.rows - dh_2) / 2;

            int dx = 0;
            for (int x_idx = 0; x_idx < thd_params->m_objective_test->m_shift_nbr_steps; x_idx++)
            {
                if (dx < x0 + overlap_min && rgb_src.cols - x0 + dx < rgb_src.cols)
                {
                    thd_params->m_objective_test->m_dx[tid] = dx;
                    int dy = 0;
                    for (int y_idx = 0; y_idx < thd_params->m_objective_test->m_shift_nbr_steps; y_idx++)
                    {
                        if ((dx != 0 || dy != 0) && dy < y0 + overlap_min && rgb_src.rows - y0 + dy < rgb_src.rows)
                        {
                            thd_params->m_objective_test->m_dy[tid]= dy;

                            // Virtual Camera Application
                            const Rect roi_1 = Rect((rgb_src.cols - dw_1) / 2, (rgb_src.rows - dh_1) / 2, dw_1, dh_1);
                            const Rect roi_2 = Rect(((rgb_src.cols - dw_2) / 2) + dx, ((rgb_src.rows - dh_2) / 2) + dy, dw_2, dh_2);

                            // Checks the Roi 1 image doesn't overflow the Roi 2
                            if (roi_1.x + roi_1.width > rgb_src.cols ||
                                    roi_1.y + roi_1.height > rgb_src.rows ||
                                    roi_2.x + roi_2.width > rgb_src.cols ||
                                    roi_2.y + roi_2.height > rgb_src.rows)
                            {
#ifdef IS_VERBOSE
                                cout << "The window exceedes the size of the source image !!" << endl;
#endif
                                return false;
                            }

                            Mat rgb_dist_1;
                            Mat rgb_dist_2;

                            switch (test_type)
                            {
                            case 0:  // Incremental Distortion
                            {
                                //----------------------------------------------------------------------------------------
                                // For every single geometrical configuration of image 1 and image 2,
                                // the entire set of photometric and geometrical distortions are generated for the image 2
                                //----------------------------------------------------------------------------------------
                                for (int distortion_idx = 0; distortion_idx < thd_params->m_objective_test->m_nbr_distortion_types; distortion_idx++)
                                {
                                    thd_params->m_objective_test->m_vct_distortion_id[tid] = distortion_idx;
                                    const DirOrder dir_order = SET_TO_OPERATION;
                                    DistortionManager dist_man_2 = DistortionManager(thd_params->m_objective_test->m_path, dir_order, rgb_src(Rect(roi_2)), DISTORTION_COUNT, thd_params->m_objective_test->m_nbr_distortion_steps);
                                    dist_man_2.m_distortion_id = (Distortions)(distortion_idx);
                                    dist_man_2.setDistortionProperties();
#ifdef IS_VERBOSE
                                    cout << "Computing distortion: " << dist_man_2.m_distort_name << endl;
#endif
                                    thd_params->m_objective_test->applyIncrementalDistortions(rgb_src, roi_1, roi_2, sub_ratio, tid);
                                }
                                break;
                            }
                            case 1:  // Serial Distortion
                            {
                                // Applies a stack of different distortions to the original image to simulate the optical chain
                                // and camera real life lightening and posing distortions

                                thd_params->m_objective_test->applyDistortionStack(rgb_src, roi_1, roi_2, rgb_dist_1, rgb_dist_2);
                            }
                                i++;
                            }
                        }
                        dy += shift_step.y;
                    }
                }
                dx += shift_step.x;
            }
        }
    }
#ifdef IS_LINUX
    //   pthread_exit(NULL);
#endif
}

//------------------------------------------------------------------------------------------------------------

void *ParallelStitching(void *p_thd_params);
void *ParallelStitching(void *p_thd_params)
{
    //-------------------------------------------------------------------------------------------------
    // Read the input dataset of icon images
    //-------------------------------------------------------------------------------------------------
    ThdParams_1 *thd_params = NULL;
    thd_params = ((ThdParams_1 *)p_thd_params);

    const unsigned int tid = thd_params->m_tid;
    const unsigned int d_idx = thd_params->m_objective_test->m_distortion_id;
    const unsigned int s_idx = thd_params->m_objective_test->m_distortion_step;
    unsigned int idx_valid = 0;

    // #ifdef IS_VERBOSE
    //    cout << "---------------------------------------------------------" << endl;
    //    cout << "BEGIN ParallelStitching" << endl;
    //    cout << "TID: " << thd_params->m_tid << endl;
    //    cout << "IMAGESET SIZE: " << thd_params->m_objective_test->m_image_set[tid].size() << endl;
    //    cout << "LOOPS PER THREAD: " << thd_params->m_loops_per_thread << endl;
    //    cout << "NBR OF THREADS: " <<  thd_params->m_nbr_threads << endl;
    //    cout << "MOSAIC FUNCTION: " << thd_params->m_objective_test->m_mosaicing_method << endl;
    //    cout << "DISTORTION TYPE: " << thd_params->m_objective_test->m_distortion_id << endl;
    //    cout << "DISTORTION STEP: " << thd_params->m_objective_test->m_distortion_step << endl;
    //    cout << "---------------------------------------------------------" << endl;
    // #endif

    for (int idx = 0; idx < thd_params->m_loops_per_thread; idx++)
    {
        const unsigned int img_idx = idx + (thd_params->m_tid * thd_params->m_loops_per_thread);

        // Retrieves the distortion name
        const string path = "";

        Mat rgb_src;
        DistortionManager dist_man = DistortionManager(path, SET_TO_OPERATION, rgb_src, 1, 1);
        dist_man.m_distortion_id = (Distortions)d_idx;
        dist_man.setDistortionProperties();

        if (img_idx < thd_params->m_objective_test->m_nbr_imgs)
        {
            idx_valid++;

            *thd_params->m_objective_test->m_log_file << "-----------------------------------------------------------" << endl;
            *thd_params->m_objective_test->m_log_file << "Begin Parallel " << dist_man.m_distort_name << " - Image: " << img_idx << " - Distortion: " << d_idx << " - Step: " << s_idx << endl;
            *thd_params->m_objective_test->m_log_file << "-----------------------------------------------------------" << endl;
            thd_params->m_objective_test->m_image_id[tid] = img_idx;

            char val_img_id[256];
            memset(val_img_id, 0, 128);
            sprintf(val_img_id, "%04d", thd_params->m_objective_test->m_image_id[tid]);

            thd_params->m_objective_test->m_vct_roi_1[tid].clear();
            thd_params->m_objective_test->m_vct_roi_2[tid].clear();

            //--------------------------------------------------------------------------
            // Loads recursively the entire set of nominal images
            //--------------------------------------------------------------------------
            string f_name = thd_params->m_objective_test->m_src_path + "Image_" + string(val_img_id) + "/roi_1/";
            const unsigned int nbr_loaded_images_1 = thd_params->m_objective_test->recursive(f_name, tid);

            if (nbr_loaded_images_1 > 0)
            {
#ifdef IS_VERBOSE
                cout << "Loading: " << nbr_loaded_images_1 << " images from roi_1" << endl;
#endif
            }
            else
            {
#ifdef IS_VERBOSE
                cout << "No images loaded from roi_1 !!" << endl;
#endif
                return NULL;
            }
            thd_params->m_objective_test->m_vct_roi_1[tid] = thd_params->m_objective_test->m_image_set[tid];
            thd_params->m_objective_test->m_image_set[tid].clear();

            //--------------------------------------------------------------------------
            // Loads recursively the entire set of distorted images
            // The imges are copied into the m_image_set vector and then manually
            // transferred to m_vct_roi_1 and m_vct_roi_2
            //--------------------------------------------------------------------------
            char val_s[256];
            memset(val_s, 0, 128);
            sprintf(val_s, "%02d", s_idx);

            f_name = thd_params->m_objective_test->m_src_path + "Image_" + string(val_img_id) + "/roi_2/";
            f_name += dist_man.m_distort_name;
            f_name += "/" + string(val_s) + "/";
            const unsigned int nbr_loaded_images_2 = thd_params->m_objective_test->recursive(f_name, tid);

            if (nbr_loaded_images_2 > 0)
            {
#ifdef IS_VERBOSE
                cout << "Loading: " << nbr_loaded_images_2 << " images from roi_2" << endl;
#endif
            }
            else
            {
#ifdef IS_VERBOSE
                cout << "No images loaded from roi_2 !!" << endl;
#endif
                return NULL;
            }
            thd_params->m_objective_test->m_vct_roi_2[tid] = thd_params->m_objective_test->m_image_set[tid];
            thd_params->m_objective_test->m_image_set[tid].clear();

            // #ifdef IS_VERBOSE
            //            cout << "---------------------------------------------------" << endl;
            //            cout << "TID: " << tid << endl;
            //            cout << "Number of Images: " << thd_params->m_objective_test->m_nbr_imgs << endl;
            //            cout << "Nbr of Loaded Images: " << nbr_loaded_images_2 << endl;
            //            cout << "Image Id: " <<  img_idx << "  " <<  thd_params->m_objective_test->m_image_id[tid]  << endl;
            //            cout << "File Name: " << f_name << endl;
            //            cout << "VCT ROI 2: "  <<    thd_params->m_objective_test->m_vct_roi_2[tid].size() << endl;
            //            cout << "---------------------------------------------------" << endl;
            // #endif

            //-------------------------------------------------------------------------
            // Loads recursively the entire set of distorted images
            //--------------------------------------------------------------------------
            f_name =   thd_params->m_objective_test->m_src_path + "Image_" + string(val_img_id) + "/roi_2/";
            f_name += dist_man.m_distort_name;
            f_name += "/00/";
            const unsigned int nbr_loaded_undistorted_images_2 = thd_params->m_objective_test->recursive(f_name, tid);

            if (nbr_loaded_undistorted_images_2 > 0)
            {

#ifdef IS_VERBOSE
                cout << "Loading: " << nbr_loaded_undistorted_images_2 << " undistorted images from roi_2" << endl;
#endif
            }
            else
            {
#ifdef IS_VERBOSE
                cout << "No images loaded from roi_2 !!" << endl;
#endif
                return NULL;
            }

            // vct_roi_undistorted_2 = thd_params->m_objective_test->m_image_set;
            thd_params->m_objective_test->m_image_set[tid].clear();

            //-------------------------------------------------------------------------------------
            // Stitching single degradatation, single strength, all stitching functions
            //-------------------------------------------------------------------------------------
            if (thd_params->m_objective_test->applyStitchingToVector(s_idx, tid) == false)
            {
#ifdef IS_VERBOSE
                cout << "applyStitchingToVector failed !!" << endl;
#endif
                return NULL;
            }

            thd_params->m_psnr += thd_params->m_objective_test->m_psnr_mean[d_idx][s_idx][tid];
            thd_params->m_ssim += thd_params->m_objective_test->m_ssim_mean[d_idx][s_idx][tid];

            thd_params->m_mse_psnr.push_back(thd_params->m_objective_test->m_psnr_mse[d_idx][s_idx][tid]);
            thd_params->m_mse_ssim.push_back(thd_params->m_objective_test->m_ssim_mse[d_idx][s_idx][tid]);

            *thd_params->m_objective_test->m_log_file << "PSNR strength mean: "
                                                      << thd_params->m_objective_test->m_psnr_mean[d_idx][s_idx][tid]
                                                         << " - SSIM strength mean: "
                                                         << thd_params->m_objective_test->m_ssim_mean[d_idx][s_idx][tid]
                                                            << endl;
            *thd_params->m_objective_test->m_log_file << "-----------------------------------------------------------" << endl;
            *thd_params->m_objective_test->m_log_file << "End Parallel " << dist_man.m_distort_name << " - Image: " << img_idx << " - Distortion: " << d_idx << " - Step: " << s_idx << endl;
            *thd_params->m_objective_test->m_log_file << "-----------------------------------------------------------" << endl;
            thd_params->m_objective_test->m_log_file->flush();
        }
        else
        {
#ifdef IS_VERBOSE
            cout << "---------------------------------------------------------" << endl;
            cout << "IMG IDX DISCARDED: " << img_idx << endl;
            cout << "---------------------------------------------------------" << endl;
#endif
        }
    }   // End img_idx par thread cycle
    // Computes the mean for thread image subset
    thd_params->m_psnr /= idx_valid;
    thd_params->m_ssim /= idx_valid;
    // cout << "Distortion Stack Parallelize.. Thread ID, " << thd_params->m_tid << endl;

#ifdef IS_LINUX
    //pthread_exit(NULL);
#endif
}

//------------------------------------------------------------------------------------------------------------

void *DistortionStackParallelize(void *p_thd_params);
void *DistortionStackParallelize(void *p_thd_params)
{
    return NULL;
}

//------------------------------------------------------------------------------------------------------------

ObjectiveTest::ObjectiveTest()
{
    m_nbr_cores = std::thread::hardware_concurrency();
}

//------------------------------------------------------------------------------------------------------------

ObjectiveTest::~ObjectiveTest()
{

}

//------------------------------------------------------------------------------------------------------------

ObjectiveTest::ObjectiveTest(const string path,
                             const string src_path,
                             const string dst_path,
                             const uint32_t max_img_set_size,
                             const uint32_t shift_nbr_steps,
                             const uint32_t nbr_distortion_types,
                             const uint32_t distortion_range,
                             const uint32_t nbr_distortion_steps,
                             const uint32_t min_img_size,
                             const BenchmarkType benchmark_pipeline,
                             const bool enable_generation,
                             const bool generate_target_files,
                             const int min_match_array_size,
                             const MosaicingMethod mosaicing_method,
                             const StitchingType stitching_type)
{
    m_path = path;
    m_src_path = src_path;
    m_dst_path = dst_path;
    m_selected_benchmark = benchmark_pipeline;
    m_mosaicing_method = mosaicing_method;
    m_min_match_array_size = min_match_array_size;
    m_stitching_type = stitching_type;
    m_shift_nbr_steps = shift_nbr_steps;

    m_nbr_distortion_types = nbr_distortion_types;
    m_distortion_range = distortion_range;
    m_nbr_distortion_steps = nbr_distortion_steps;

    m_max_imgset_size = max_img_set_size;
    m_generate_target_files = generate_target_files;

    m_nbr_cores = std::thread::hardware_concurrency();

    // Set to minimize the first result
    const bool is_log_scale = false;

    // Set to hide the first result
    const bool skip_image_0 = true;

#ifdef IS_WIN
    if (mkdir(string(path + "log").c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create log folder [" << string(path + "log") << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(string(path + "log").c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {

#ifdef IS_VERBOSE
        cout << "cannot create folder [" << string(path + "log") << "] : " << strerror(errno) << endl;
#endif
    }
#endif

    long timer_roi_gener_start = clock();

    m_log_file = new ofstream();
    m_log_file->open(path + "/log/log_" + currentDateTime() + ".txt");
    *m_log_file << "------------------------------------------------------------"  << endl;
    *m_log_file << "Begin of Objective Benchmark" << endl;
    *m_log_file << "------------------------------------------------------------"  << endl;


    //----------------------------------------------------------------------------------
    // Begin of generation of distort ROI images
    //----------------------------------------------------------------------------------
    if (enable_generation == true)
    {
#ifdef IS_VERBOSE
        cout << "//----------------------------------------------------------------------------------" << endl;
        cout << "// Start of Distort ROI image generation" << endl;
        cout << "//----------------------------------------------------------------------------------" << endl;
#endif

        const int nbr_loaded_imgs = recursive(src_path);

        if (nbr_loaded_imgs == 0)
        {
#ifdef IS_VERBOSE
            cout << "No images found in NominalSub directory !!" << endl;
#endif
            exit(-1);
        }

        sort(m_nominal_image_set.begin(), m_nominal_image_set.end());

        const unsigned int nbr_thds = ceil((float)m_nominal_image_set.size() / m_nbr_cores);

        // Multithreading execution
        void *status;

        for (int thd_idx = 0; thd_idx < nbr_thds; thd_idx++)
        {
            ThdParams_1 thd_params;
            void *p_thd_params = &thd_params;
            thd_params.m_objective_test = this;

            // Excess approximation
            thd_params.m_loops_per_thread = ceil((float)m_nominal_image_set.size() / nbr_thds);
            thd_params.m_nbr_threads = nbr_thds;
            thd_params.m_distortion_idx = m_distortion_id;
            thd_params.m_tid = thd_idx;
            thd_params.m_min_img_size = min_img_size;

            // Thread vector initialization
            thd_params.m_objective_test->m_vct_image_id.push_back(0);
            thd_params.m_objective_test->m_vct_distortion_id.push_back(0);
            thd_params.m_objective_test->m_dw_1.push_back(0);
            thd_params.m_objective_test->m_dw_2.push_back(0);
            thd_params.m_objective_test->m_dh_1.push_back(0);
            thd_params.m_objective_test->m_dh_2.push_back(0);
            thd_params.m_objective_test->m_dx.push_back(0);
            thd_params.m_objective_test->m_dy.push_back(0);

            thread thd(IncrementalDistortionParallelize, (void *)p_thd_params);
            thd.join();

#ifdef IS_VERBOSE
            cout << "Main: completed thread id :" << thd_idx << " with status: " << status << endl;
#endif
        }

        long timer_roi_gener_stop = clock();
#ifdef IS_VERBOSE
        cout << "ROI Generation done in: " << ((double)(timer_roi_gener_stop - timer_roi_gener_start)) / CLOCKS_PER_SEC << " sec." << endl;
#endif

        exit(0);
    }
    //----------------------------------------------------------------------------------
    // End of generation of distort ROI images
    //----------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------
    // Phase 2: Begin of stitching process
    //----------------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    // Computes the entire set of photometric and geometric distortions to the
    // selected image and stores the results into destination image files
    //-------------------------------------------------------------------------

    long timer_benchmark_start = clock();
    //---------------------------------------------------------------------
    // Stitching Function Loop
    //
    // Applies the comparative stitching performing the sntire set of
    // stitching algorithms to benchmark
    //---------------------------------------------------------------------

    vector<StitchingType> stitch_types;
    stitch_types.push_back(Feathering);
    stitch_types.push_back(Multiband);
    // stitch_types.push_back(Poisson);

    stitch_types.push_back(Watershed);
    stitch_types.push_back(Graphcut);
    stitch_types.push_back(Voronoi);

    for (int stp_idx = 0; stp_idx < nbr_distortion_steps; stp_idx++)
    {
        m_psnr_xl_tabs.push_back(0.0);
        m_ssim_xl_tabs.push_back(0.0);
    }

    for (int d_idx = 0; d_idx < nbr_distortion_types; d_idx++)
    {
        m_psnr_xl_cols.push_back(m_psnr_xl_tabs);
        m_ssim_xl_cols.push_back(m_ssim_xl_tabs);
    }

    for (int fnct_idx = 0; fnct_idx < stitch_types.size(); fnct_idx++)
    {
        m_psnr_xl_sheet_2.push_back(m_psnr_xl_cols);
        m_ssim_xl_sheet_2.push_back(m_psnr_xl_cols);

        m_psnr_xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]));
        m_ssim_xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]));

        m_mse_psnr_xl_sheet_2.push_back(m_psnr_xl_cols);
        m_mse_ssim_xl_sheet_2.push_back(m_psnr_xl_cols);

        //m_psnr_xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]));
        // m_ssim_xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]));
    }

    // Stitching function merged plot data structures
    ExportDictionary xl_dict_merge;
    ExportHeader xl_header_merge;

    vector<pair<int,string> > v1;
    m_image_set.push_back(v1);
    m_nbr_imgs = recursive(src_path, 0);
    m_image_set.clear();

    if (m_nbr_imgs == 0)
    {
#ifdef IS_VERBOSE
        cout << "No images found in: " << src_path << endl;
#endif
        exit(-1);
    }

    for (int fnct_idx = 0; fnct_idx < stitch_types.size(); fnct_idx++)
    {
        m_stitching_type = stitch_types[fnct_idx];

        *m_log_file << "-----------------------------------------------------------" << endl;
        *m_log_file << "- Begin " << get_stitching_fnct_name(stitch_types[fnct_idx]) << endl;
        *m_log_file << "-----------------------------------------------------------" << endl;

        //---------------------------------------------
        // GnuPlotExport
        // Appends the Header column
        //---------------------------------------------
        m_psnr_xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]));
        m_psnr_xl_dict.m_header = m_psnr_xl_header;

        //-------------------------------------------------------------------------------------------------
        // 1. Loop: Image Distortion Step Loop
        // Each step applies a different image distortion transform
        //-------------------------------------------------------------------------------------------------
        const long timer_distortion_loop_start = clock();

        for (int d_idx = 0; d_idx < nbr_distortion_types; d_idx++)
        {
            const DirOrder dir_order = SET_TO_OPERATION;
            Mat rgb_src;
            DistortionManager dist_man = DistortionManager(m_path, dir_order, rgb_src, 1, 1);
            dist_man.m_distortion_id = (Distortions)d_idx;
            dist_man.setDistortionProperties();

            *m_log_file << "-----------------------------------------------------------" << endl;
            *m_log_file << "Begin Distortion: " << "DIST: " << d_idx << "/" << nbr_distortion_types << "  " << dist_man.m_distort_name << endl;
            *m_log_file << "-----------------------------------------------------------" << endl;

            vector<vector<double> > psnr_tmp;
            m_psnr_mean.push_back(psnr_tmp);

            vector<vector<double> > ssim_tmp;
            m_ssim_mean.push_back(ssim_tmp);

            vector<vector<double> > psnr_mse_tmp;
            m_psnr_mse.push_back(psnr_mse_tmp);

            vector<vector<double> > ssim_mse_tmp;
            m_ssim_mse.push_back(ssim_mse_tmp);

            m_distortion_id = d_idx;

            //-----------------------------------------------------------------------------------------------------
            // 2. Loop: Image Distortion Step Stitching Loop
            // Each step is a different strength of a degraded image the selected image degradation transform
            //-----------------------------------------------------------------------------------------------------
            const long timer_step_loop_start = clock();

            vector<vector<double> > psnr_mean;                    // Mean of PSNR for all cominations of Image_1 and Image 2
            vector<vector<double> > ssim_mean;                    // Mean of SSIM for all cominations of Image_1 and Image 2

            m_psnr_mean.push_back(psnr_mean);
            m_ssim_mean.push_back(ssim_mean);

            m_psnr_mse.push_back(psnr_mean);
            m_ssim_mse.push_back(ssim_mean);

            for (int s_idx = 0; s_idx < nbr_distortion_steps; s_idx++)
            {
                *m_log_file << "-----------------------------------------------------------" << endl;
                *m_log_file << "Begin " << " STEP: " << s_idx << "  " << dist_man.m_distort_name << " Step: " << s_idx << endl;
                *m_log_file << "-----------------------------------------------------------" << endl;

                vector<double> v1;
                m_psnr_mean[d_idx].push_back(v1);

                vector<double> v2;
                m_ssim_mean[d_idx].push_back(v2);

                //vector<double> v1;
                m_psnr_mse[d_idx].push_back(v1);

                // vector<double> v2;
                m_ssim_mse[d_idx].push_back(v2);

                m_strength = s_idx; // * incremental_step;
                m_distortion_step = s_idx;

                const unsigned int nbr_thds = ceil((float)m_nbr_imgs / m_nbr_cores);

                //-------------------------------------------
                // Begin of Multithreading execution section
                //-------------------------------------------

                vector<float> vct_mse_psnr;
                vector<float> vct_mse_ssim;

                void *status;
                vector<int> thd_ret(nbr_thds);

                for (int thd_idx = 0; thd_idx < nbr_thds; thd_idx++)
                {
                    ThdParams_1 thd_params;
                    void *p_thd_params = &thd_params;

                    thd_params.m_tid = thd_idx;
                    thd_params.m_objective_test->m_nbr_imgs;
                    thd_params.m_objective_test = this;
                    thd_params.m_loops_per_thread = ceil((float)m_nbr_imgs / nbr_thds);
                    thd_params.m_nbr_threads = nbr_thds;

                    m_psnr_mean[d_idx][s_idx].push_back(0.0);
                    m_ssim_mean[d_idx][s_idx].push_back(0.0);

                    m_psnr_mse[d_idx][s_idx].push_back(0.0);
                    m_ssim_mse[d_idx][s_idx].push_back(0.0);

                    vector<pair<int, string> > vct_roi_1;
                    vector<pair<int, string> > vct_roi_2;

                    m_vct_roi_1.push_back(vct_roi_1);
                    m_vct_roi_2.push_back(vct_roi_2);
                    m_image_id.push_back(0);

                    vector<pair<int, string> > v1;
                    m_image_set.push_back(v1);

                    thread thd(ParallelStitching, (void *)p_thd_params);

                    thd_params.m_psnr = 0.0;
                    thd_params.m_ssim = 0.0;

                    thd_params.m_mse_psnr.clear();
                    thd_params.m_mse_ssim.clear();

                    thd.join();

#ifdef IS_VERBOSE
                    cout << "Main: completed thread id: " << thd_idx << " with status: " << status << endl;
#endif
                    thd_ret[thd_idx] = 1;

                    // Adds all images per thread
                    for (int i = 0; i < thd_params.m_mse_psnr.size(); i++)
                    {
                        vct_mse_psnr.push_back(thd_params.m_mse_psnr[i]);
                        vct_mse_ssim.push_back(thd_params.m_mse_ssim[i]);
                    }
                    m_psnr_xl_sheet_2[fnct_idx][d_idx][s_idx] += thd_params.m_psnr;
                    m_ssim_xl_sheet_2[fnct_idx][d_idx][s_idx] += thd_params.m_ssim;
                }  // End of thd_idx loop

                //--------------------------------------------------------------------------------
                // All threads must be ended successfully at this point
                //--------------------------------------------------------------------------------
                unsigned int thd_cnt = 0;

                for (int thd_idx = 0; thd_idx < nbr_thds; thd_idx++)
                {
                    if (thd_ret[thd_idx] == 1)
                    {
                        thd_cnt++;
                    }
                }

                if (thd_cnt < nbr_thds)
                {
                    cout << "Threads not ended: " << nbr_thds - thd_cnt << " Program ends !" << endl;
                    //   exit(-1);
                }

                const long timer_step_loop_stop = clock();
                *m_log_file << "-----------------------------------------------------------" << endl;
                *m_log_file << "End Distortion " << dist_man.m_distort_name << " " << timer_step_loop_stop - timer_step_loop_start / CLOCKS_PER_SEC << " sec." << endl;
                *m_log_file << "-----------------------------------------------------------" << endl;
                m_log_file->flush();

                // 1. Computes the mean of the PSNR and SSIM
                m_psnr_xl_sheet_2[fnct_idx][d_idx][s_idx] /= nbr_thds;
                m_ssim_xl_sheet_2[fnct_idx][d_idx][s_idx] /= nbr_thds;

                const float mean = m_psnr_xl_sheet_2[fnct_idx][d_idx][s_idx];
                const float siz = vct_mse_psnr.size();

                // 2. Computes the MSE of the PSNR and SSIM
                float mse_psnr = 0.0;

                for (int i = 0; i < siz; i++)
                {
                    mse_psnr += pow(vct_mse_psnr[i] - mean, 2);
                }
                mse_psnr /= siz;

                m_mse_psnr_xl_sheet_2[fnct_idx][d_idx][s_idx] = mse_psnr;

                float mse_ssim = 0.0;

                for (int i = 0; i < siz; i++)
                {
                    mse_ssim += pow(vct_mse_ssim[i] - mean, 2);
                }
                mse_ssim /= siz;
                m_mse_ssim_xl_sheet_2[fnct_idx][d_idx][s_idx] = mse_ssim;

                //                cout << "-----------------------------------" << endl;
                //                cout << "step: " <<  s_idx << endl;
                //                cout << "MEAN: " <<  mean << endl;
                //                cout << "MSE: " <<  mse_psnr << endl;
                //                cout << "-----------------------------------" << endl;

            }  // End of step_idx cycle

            ExportDictionary xl_dict_2;
            ExportHeader xl_header;
            xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]) + " " + dist_man.m_distort_name);
            xl_dict_2.m_header = xl_header;
            xl_dict_2.m_data.push_back(m_psnr_xl_sheet_2);

            // 1. Plotting PSNR mean
            generateGnuPlot("psnr mean ",
                            xl_dict_2,
                            fnct_idx,
                            d_idx,
                            is_log_scale,
                            skip_image_0);

            xl_header.clear();
            xl_dict_2.m_data.clear();

            xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]) + " " + dist_man.m_distort_name);
            xl_dict_2.m_header = xl_header;
            xl_dict_2.m_data.push_back(m_ssim_xl_sheet_2);

            // 2. Plotting SSIM mean
            generateGnuPlot("ssim mean ",
                            xl_dict_2,
                            fnct_idx,
                            d_idx,
                            is_log_scale,
                            skip_image_0);

            xl_header.clear();
            xl_dict_2.m_data.clear();

            xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]) + " " + dist_man.m_distort_name);
            xl_dict_2.m_header = xl_header;
            xl_dict_2.m_data.push_back(m_mse_psnr_xl_sheet_2);

            //            // 3. Plotting PSNR MSE
            //            generateGnuPlot("psnr mse ",
            //                            xl_dict_2,
            //                            fnct_idx,
            //                            d_idx,
            //                            is_log_scale,
            //                            skip_image_0);

            //            xl_header.clear();
            //            xl_dict_2.m_data.clear();

            //            xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]) + " " + dist_man.m_distort_name);
            //            xl_dict_2.m_header = xl_header;
            //            xl_dict_2.m_data.push_back(m_mse_ssim_xl_sheet_2);

            //            // 4. Plotting SSIM MSE
            //            generateGnuPlot("ssim mean ",
            //                            xl_dict_2,
            //                            fnct_idx,
            //                            d_idx,
            //                            is_log_scale,
            //                            skip_image_0);

            xl_header.clear();
            xl_dict_2.m_data.clear();
        }  // End of dist_idx cycle

        //        for (int d_idx = 0; d_idx < stitch_types.size(); d_idx++)
        //        {
        //            for (int s_idx = 0; s_idx < nbr_distortion_steps; s_idx ++)
        //            {
        //                m_psnr_xl_sheet[fnct_idx][s_idx] /= (nbr_img * nbr_distortion_types);
        //                m_ssim_xl_sheet[fnct_idx][s_idx] /= (nbr_img * nbr_distortion_types);
        //            } // End of s_idx cycle
        //        } // End of d_idx cycle

        //        ExportDictionary xl_dict_2;
        //        ExportHeader xl_header;
        //        xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]));
        //        xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]));
        //        xl_dict_2.m_header = xl_header;
        //        xl_dict_2.m_data.push_back(m_psnr_xl_sheet);
        //        generateGnuPlot("psnr", xl_dict_2);
        //        xl_header.clear();
        //        xl_dict_2.m_data.clear();

        //        xl_header.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]));

        //        xl_dict_2.m_header = xl_header;
        //        xl_dict_2.m_data.push_back(m_ssim_xl_sheet);
        //        generateGnuPlot("ssim", xl_dict_2);

        //        xl_header.clear();
        //        xl_dict_2.m_data.clear();

        const long timer_distortion_loop_stop = clock();
        const double timer_duration_distortion_loop = ((double)(timer_distortion_loop_stop - timer_distortion_loop_start)) / CLOCKS_PER_SEC;

        xl_header_merge.push_back(get_stitching_fnct_name(stitch_types[fnct_idx]));

        *m_log_file << "-----------------------------------------------------------" << endl;
        *m_log_file << "End " << get_stitching_fnct_name(stitch_types[fnct_idx]) << "    (" << timer_duration_distortion_loop << " sec.)" << endl;
        *m_log_file << "-----------------------------------------------------------" << endl;
    }  // End of Stitching Function cycle

    //----------------------------------------------------------
    // Stitching function merged plot generation
    //----------------------------------------------------------

    //----------------------------------------------------------
    // Start Debug section
    //----------------------------------------------------------
    // cout << "BEGIN PRINT" << endl;

    //    for (int f=0; f<m_ssim_xl_sheet_2.size();f++)
    //    {
    //        cout << "BEGIN D" << endl;
    //        for (int d=0; d < m_ssim_xl_sheet_2[f].size();d++)
    //        {
    //            cout << "BEGIN S" << endl;
    //            for (int s=0; s<m_ssim_xl_sheet_2[f][d].size();s++)
    //            {
    //                cout << "F:" << f << " - D:" << d << " - S: " << s << " SSIM: " << m_ssim_xl_sheet_2[f][d][s] << endl;
    //            }
    //        }
    //    }
    //exit(0);
    //----------------------------------------------------------
    // End Debug section
    //----------------------------------------------------------

    xl_dict_merge.m_header = xl_header_merge;
    xl_dict_merge.m_data.push_back(m_psnr_xl_sheet_2);

    for (int d_idx = 0; d_idx < nbr_distortion_types; d_idx++)
    {
        DistortionManager dist_man = DistortionManager();
        dist_man.m_distortion_id = (Distortions)d_idx;
        dist_man.setDistortionProperties();

        xl_dict_merge.m_data.push_back(m_psnr_xl_sheet_2);
        generateMergedGnuPlot("merge psnr mean " + dist_man.m_distort_name,
                              xl_dict_merge,
                              d_idx,
                              is_log_scale,
                              skip_image_0);

        xl_dict_merge.m_data.clear();

        xl_dict_merge.m_data.push_back(m_ssim_xl_sheet_2);
        generateMergedGnuPlot("merge ssim mean " + dist_man.m_distort_name,
                              xl_dict_merge,
                              d_idx,
                              is_log_scale,
                              skip_image_0);

        xl_dict_merge.m_data.clear();

        xl_dict_merge.m_data.push_back(m_mse_psnr_xl_sheet_2);
        generateMergedGnuPlot("merge psnr mse " + dist_man.m_distort_name,
                              xl_dict_merge,
                              d_idx,
                              is_log_scale,
                              skip_image_0);
        xl_dict_merge.m_data.clear();

        xl_dict_merge.m_data.push_back(m_mse_ssim_xl_sheet_2);
        generateMergedGnuPlot("merge ssim mse " + dist_man.m_distort_name,
                              xl_dict_merge,
                              d_idx,
                              is_log_scale,
                              skip_image_0);
        xl_dict_merge.m_data.clear();
    }
    xl_dict_merge.m_header.clear();
    xl_dict_merge.m_data.clear();

    const long timer_benchmark_stop = clock();
    const double timer_diff_benchmark = ((double)(timer_benchmark_stop - timer_benchmark_start)) / CLOCKS_PER_SEC;
    *m_log_file << "------------------------------------------------------------"  << endl;
    *m_log_file << "End of Benchmark    (" << timer_diff_benchmark << " sec.)" << endl;
    *m_log_file << "------------------------------------------------------------"  << endl;
    m_log_file->close();
}

//----------------------------------------------------------------------------------------------------
// applyStitchingToVector
//
// This function computes the manualSIFT image pairing and computes the whole set of stitching
// functions on each pair of <nominal, distort> images.
// At the end of each cycle, aggregated mean similarity metrics like PSNR and SSIM are computed.
//----------------------------------------------------------------------------------------------------
bool ObjectiveTest::applyStitchingToVector(const int s_idx, const unsigned int tid)
{
    // Registration counter
    int cnt = 0;

    // Registration failure counter
    // int fail_cnt = 0;

    const bool hide_seam = false;
    const bool generate_outline = false;

    Point shift;

    Mat rgb_stitch_distort;
    Mat rgb_stitch_nominal_1;
    Mat rgb_stitch_nominal_2;

    m_psnr_mean[m_distortion_id][s_idx][tid] = 0.0;
    m_ssim_mean[m_distortion_id][s_idx][tid] = 0.0;

    m_psnr_mse[m_distortion_id][s_idx][tid] = 0.0;
    m_ssim_mse[m_distortion_id][s_idx][tid] = 0.0;

    if (m_mosaicing_method == MosaicingMethod::UseInternalAlgorithm)
    {
        for (int vct2_idx = 0; vct2_idx < m_vct_roi_2[tid].size(); vct2_idx++)
        {
            //----------------------------------------------------
            // Load the source images from the input vectors
            //----------------------------------------------------
            for (int vct1_idx = 0; vct1_idx < m_vct_roi_1[tid].size(); vct1_idx++)
            {
                string file_in_1 = m_vct_roi_1[tid][vct1_idx].second;
                const Mat rgb_src_1 = imread(file_in_1);

                if (rgb_src_1.rows == 0 || rgb_src_1.cols == 0)
                {
#ifdef IS_VERBOSE
                    cout << "Source file: " << file_in_1 << " is empty !!" << endl;
#endif
                    exit(-1);
                }
                string file_in_2 = m_vct_roi_2[tid][vct2_idx].second;
                const Mat rgb_src_2 = imread(file_in_2);

                if (rgb_src_2.rows == 0 || rgb_src_2.cols == 0)
                {
#ifdef IS_VERBOSE
                    cout << "Source file: " << file_in_2 << " is empty !!" << endl;
#endif
                    exit(-1);

                }
                //-------------------------------------------------------------------------
                // Extracts the Shift from the file name
                //-------------------------------------------------------------------------
                shift.x = atoi(file_in_2.substr(file_in_2.size() - 25, 4).c_str());
                shift.y = atoi(file_in_2.substr(file_in_2.size() - 20, 4).c_str());

                //-------------------------------------------------------------------------
                // 2. Comparative image stitching: The images are stitched together using the
                // one of the stitching algorithms defined by StitchingType
                //-------------------------------------------------------------------------

#ifdef IS_VERBOSE
                cout << "--------------------------------------------------------------------" << endl;
                cout << " 2. Quality Metrics Mosaicing measurement:" << endl;
                cout << "--------------------------------------------------------------------" << endl;
#endif

                m_idx = cnt++;

                vector<Point2f> corners_1(4);
                vector<Point2f> corners_2(4);
                corners_2[0] = shift;
                corners_2[1] = cvPoint(rgb_src_2.cols, 0);
                corners_2[2] = cvPoint(rgb_src_2.cols, rgb_src_2.rows);
                corners_2[3] = cvPoint(0, rgb_src_2.rows);

                if (m_selected_benchmark == StitchingPipeline)
                {
                    //--------------------------------------------------------------------------
                    // 1. Test BenchmarkStitch: The image shifting vector is manually provided.
                    // The test is aimed to measure the behaviour of the stitcing algorithm to
                    // the raw input images prior to any photometric of geometrical correction.
                    //--------------------------------------------------------------------------
                    corners_1[0] = cvPoint(0, 0);
                    corners_1[1] = cvPoint(rgb_src_1.cols, 0);
                    corners_1[2] = cvPoint(rgb_src_1.cols, rgb_src_1.rows);
                    corners_1[3] = cvPoint(0, rgb_src_1.rows);

                    vector<Point2f> shared_region(2);

                    //--------------------------------------------------------------------------
                    // Begin No SIFT SIngle Image Benchmarking
                    // Mat img1 = imread("D:/PRJ/Benchmark/OBJECTIVE_TEST/TEST_COMPARE_SEAM_INVARIANCE/img_1_128_8.png");
                    // Mat img2 = imread("D:/PRJ/Benchmark/OBJECTIVE_TEST/TEST_COMPARE_SEAM_INVARIANCE/img_2_128_8.png");
                    //  corners_2[0].x = 8;
                    //  corners_2[0].y = 8;
                    //--------------------------------------------------------------------------
                    if (comparative_stitching_1(rgb_src_1,
                                                rgb_src_2,
                                                corners_1,
                                                corners_2,
                                                rgb_stitch_distort,
                                                rgb_stitch_nominal_1,
                                                rgb_stitch_nominal_2,
                                                shared_region,
                                                hide_seam,
                                                generate_outline,
                                                tid) == false)
                    {
                        exit(-1);
                    }
                    double psnr_val_1 = 0.0;
                    double ssim_val_1 = 0.0;

                    double psnr_val_2 = 0.0;
                    double ssim_val_2 = 0.0;

                    Mat shared_img_nominal_1(rgb_stitch_nominal_1(Rect(shared_region[0].x, shared_region[0].y, shared_region[1].x, shared_region[1].y)));
                    Mat shared_img_nominal_2(rgb_stitch_nominal_2(Rect(shared_region[0].x, shared_region[0].y, shared_region[1].x, shared_region[1].y)));
                    Mat shared_img_distort(rgb_stitch_distort(Rect(shared_region[0].x, shared_region[0].y, shared_region[1].x, shared_region[1].y)));

                    //--------------------------------------------------------------------------
                    // Image stitches are compared and similarity metrics are updated
                    // If stitch_nominal and stitch_distort canvases are different, the quality
                    // metric measurement will fail and return:
                    // PSNR = 0.0, SSIM = 0.0
                    //-------------------------------------------------------------------------
                    // Artificial unit distortion added to px(0, 0), to avoid the value to drop to -infinite when contents ae completely te same
                    shared_img_distort.at<Vec3b>(0, 0)[0]++;
                    shared_img_distort.at<Vec3b>(0, 0)[1]++;
                    shared_img_distort.at<Vec3b>(0, 0)[2]++;

                    // Fidelity metrics are computed in shared areas only
                    compute_quality_metrics_2(shared_img_nominal_1, shared_img_distort, 16, psnr_val_1, ssim_val_1);
                    compute_quality_metrics_2(shared_img_nominal_2, shared_img_distort, 16, psnr_val_2, ssim_val_2);

                    if (psnr_val_1 - 0.0001 <= 0.0 && ssim_val_1 - 0.0001 <= 0.0)
                    {
                        m_geom_mismatch++;
                    }

                    // Computes the combined PSNR and SSIM
                    const float psnr_combi = (psnr_val_1 + psnr_val_2) / 2;
                    const float ssim_combi = (ssim_val_1 + ssim_val_2) / 2;

                    m_psnr_mean[m_distortion_id][s_idx][tid] += psnr_combi;
                    m_ssim_mean[m_distortion_id][s_idx][tid] += ssim_combi;

                    //                    cout << "-------------------------------------------------" << endl;
                    //                    cout << "- m_distortion_id: " << m_distortion_id << endl;
                    //                    cout << "- s_idx: " << s_idx << endl;
                    //                    cout << "- TID: " << tid << endl;
                    //                    cout <<  "- PSNR MSE: " <<  m_psnr_mse[m_distortion_id][s_idx][tid] << endl;
                    //                    cout << "-------------------------------------------------" << endl;

                    m_psnr_mse[m_distortion_id][s_idx][tid] += psnr_combi;
                    m_ssim_mse[m_distortion_id][s_idx][tid] += ssim_combi;

                    *m_log_file << "-----------------------------------------------------------" << endl;
                    *m_log_file << " Id: " << m_idx << " - Image 1: " << "(" << rgb_src_1.cols << "," << rgb_src_1.rows << ")"
                                << " - Image 2: " << "(" << rgb_src_2.cols << "," << rgb_src_2.rows << ")" << endl;
                    DistortionManager dist_man;

                    *m_log_file << " Function: " << get_stitching_fnct_name(m_stitching_type) << " - Distortion: " << dist_man.get_distort_name(m_distortion_id) << " step: " << s_idx << " - PSNR: " << psnr_combi << " - SSIM: " << ssim_combi << endl;
                    // *m_log_file << " Geometry mismatch: " << m_geom_mismatch << " - Shift error: (" << shift_error.x << "," << shift_error.y << ")" << endl;
                    // *m_log_file << "-----------------------------------------------------------" << endl;
#ifdef IS_VERBOSE
                    cout << endl << "--------------------------------------------------------------------" << endl;
#endif
                }
                else if (m_selected_benchmark == GeometryFidelity)
                {
                }
                else // if (m_selected_benchmark == BenchmarkingPipeline)
                {
                }
            }  // End m_vct_roi_1.size() Loop
        }  // End m_vct_roi_2.size() Loop

        const int vct_siz = m_vct_roi_1[tid].size() * m_vct_roi_2[tid].size();

        // Means-up the values generated from all input images having the same distortion and strength
        // const float good_res = siz - fail_cnt;
        m_psnr_mean[m_distortion_id][s_idx][tid] /= vct_siz;
        m_ssim_mean[m_distortion_id][s_idx][tid] /= vct_siz;

        m_psnr_mse[m_distortion_id][s_idx][tid] /= vct_siz;
        m_ssim_mse[m_distortion_id][s_idx][tid] /= vct_siz;

        //   *m_log_file << "-----------------------------------------------------------------------------" << endl;
        //   *m_log_file << "N. of failing matchings: " << fail_cnt << "/" << siz << endl;

        //  const float mathing_failure_ratio = ((float)fail_cnt / siz) * 100;
        //  *m_log_file << "Matching failure ratio: " << mathing_failure_ratio << endl;
        *m_log_file << "-----------------------------------------------------------------------------" << endl;
        *m_log_file << "TID: " << tid << " - PSNR mean per thread: " << m_psnr_mean[m_distortion_id][s_idx][tid] << endl;
        *m_log_file << "TID: " << tid << " - SSIM mean per thread: " << m_ssim_mean[m_distortion_id][s_idx][tid] << endl;
        // *m_log_file << "TID: " << tid << " - MSE PSNR mean per thread: " << m_psnr_mse[m_distortion_id][s_idx][tid] << endl;
        // *m_log_file << "TID: " << tid << " - MSE SSIM mean per thread: " << m_ssim_mse[m_distortion_id][s_idx][tid] << endl;
        m_log_file->flush();

        //----------------------------------------------------------------------
        // End Computing SIFT Feature Matching statisctics
        //----------------------------------------------------------------------

        //-- Get the corners from the rgb_src_1 ( the object to be "detected" )
        //                    vector<Point2f> corners_2(4);
        //                    corners_2[0] = cvPoint(0, 0);
        //                    corners_2[1] = cvPoint(rgb_src_2.cols, 0);
        //                    corners_2[2] = cvPoint(rgb_src_2.cols, rgb_src_2.rows);
        //                    corners_2[3] = cvPoint(0, rgb_src_2.rows);
        //                    vector<Point2f> corners_1(4);

    }
    else if (m_mosaicing_method == MosaicingMethod::UseExternalTool)
    {
        //-------------------------------------------------------------------------
        // 3. Manual testing
        //-------------------------------------------------------------------------
        // Note: dist_sub_img_1.png and dist_sub_img_1.png must be merged trough
        // external third party tool and saved as mosaic_in_0
        // before executinng this operation !!
        //-------------------------------------------------------------------------

#ifdef IS_VERBOSE
        cout << "--------------------------------------------------------------------" << endl;
        cout << " 3. Manual Metrics External Mosaicing:" << endl;
        cout << "--------------------------------------------------------------------" << endl;
#endif
        int strip_margin = 0;

        string file_ext("D:/PRJ/Benchmark/OBJECTIVE_TEST/photoshop_mosaic/mosaic_in_0.png");
        Mat rgb_ext = imread(file_ext, CV_LOAD_IMAGE_COLOR);

        if (rgb_ext.rows == 0 || rgb_ext.cols == 0)
        {
#ifdef IS_VERBOSE
            cout << "Source file: " << file_ext << " is empty !!" << endl;
#endif
            exit(-1);
        }

        // If the third party generated mosaic is smaller than the nominal mosaic image
        int diff_x = rgb_ext.rows - rgb_stitch_nominal_1.rows + strip_margin;
        int diff_y = rgb_ext.cols - rgb_stitch_nominal_1.cols + strip_margin;

        int size_x = 0;
        int size_y = 0;

        if (diff_x < strip_margin || diff_y < strip_margin < strip_margin)
        {
            size_x = rgb_ext.rows - diff_x;
            size_y = rgb_ext.cols - diff_y;
        }
        else
        {
            size_x = rgb_stitch_nominal_1.rows;
            size_y = rgb_stitch_nominal_1.cols;
        }
    }
    return true;
}

//----------------------------------------------------------------------------
// Recursive Directory contents scanning
//----------------------------------------------------------------------------
#ifdef IS_WIN

int ObjectiveTest::recursive(const string path, const unsigned int tid)
{
    intptr_t hFile;
    struct _finddata_t fd;
    string filespec = "*.*";
    string findPath = path + filespec;
    hFile = _findfirst(findPath.c_str(), &fd);

    if (hFile == -1L)
    {
#ifdef IS_VERBOSE
        cout << "No  " << filespec << " files in the current dir\n";
#endif
    }
    else
    {
#ifdef IS_VERBOSE
        cout << "Scanning recursively in " << path << "\n" << endl;
#endif
        do
        {
            if ((fd.name != string(".")) && (fd.name != string("..")))
            {
                if (fd.attrib & _A_SUBDIR)
                {
#ifdef IS_VERBOSE
                    cout << "Entering directory " << fd.name << endl;
#endif
                    string str = path + fd.name + "/";

                    // Enables the multi-level depth research
                    // recursive(str, tid);
                }
                int set_id = 0;
                char id_str[4];
                id_str[0] = fd.name[6];
                id_str[1] = fd.name[7];
                id_str[2] = fd.name[8];
                id_str[3] = fd.name[9];

                const int id = atoi(id_str);
                pair<int, string> itm;
                itm.first = id;
                itm.second = path + fd.name;
                m_image_set[tid].push_back(itm);

#ifdef IS_VERBOSE
                cout << "Loading set " << set_id << " - " << fd.name << endl;
#endif
            }
        }
        while (_findnext(hFile, &fd) == 0 && m_image_set[tid].size() < m_max_imgset_size);

#ifdef IS_VERBOSE
        cout << "-------------------------------------------------"  << endl;
        cout << m_image_set[tid].size() << " Images loaded" << endl;
        cout << "-------------------------------------------------"  << endl;
#endif
    }
    _findclose(hFile);  // close the file handle
    return m_image_set[tid].size();
}
#endif  // IS_WIN

#ifdef IS_LINUX
int ObjectiveTest::recursive(const string path, const unsigned int tid)
{
    if (m_image_set.size() == 0)
    {
        cout << "No thread defined !! "<< endl;
        exit(-1);
    }

    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(path.c_str())) == NULL && m_image_set[tid].size() < m_max_imgset_size)
    {
#ifdef IS_VERBOSE
        cout << "Error(" << errno << ") opening " << path << endl;
        cout << path << endl;
        exit(-1);
#endif
        return -1;
    }

    int id = 0;

    while ((dirp = readdir(dp)) != NULL)
    {
        if ((dirp->d_name != string(".")) && (dirp->d_name != string("..")))
        {
#ifdef IS_VERBOSE
            cout << "File name: " << dirp->d_name << endl;
#endif
            pair<int, string> itm;
            itm.first = id;
            itm.second = path + dirp->d_name;
            m_image_set[tid].push_back(itm);
        }

#ifdef IS_VERBOSE
        cout << "Loading set " << id << " - " << string(dirp->d_name) << endl;
#endif
    }
    closedir(dp);
    return m_image_set[tid].size();
}
#endif  // IS_LINUX

//----------------------------------------------------------------------------
// Recursive Directory contents scanning
//----------------------------------------------------------------------------
#ifdef IS_WIN

int ObjectiveTest::recursive(const string path)
{
    intptr_t hFile;
    struct _finddata_t fd;
    string filespec = "*.*";
    string findPath = path + filespec;
    hFile = _findfirst(findPath.c_str(), &fd);

    if (hFile == -1L)
    {
#ifdef IS_VERBOSE
        cout << "No  " << filespec << " files in the current dir\n";
#endif
    }
    else
    {
#ifdef IS_VERBOSE
        cout << "Scanning recursively in " << path << "\n" << endl;
#endif
        do
        {
            if ((fd.name != string(".")) && (fd.name != string("..")))
            {
                if (fd.attrib & _A_SUBDIR)
                {
#ifdef IS_VERBOSE
                    cout << "Entering directory " << fd.name << endl;
#endif
                    string str = path + fd.name + "/";
                    recursive(str);
                }
                int set_id = 0;
                char id_str[4];
                id_str[0] = fd.name[6];
                id_str[1] = fd.name[7];
                id_str[2] = fd.name[8];
                id_str[3] = fd.name[9];

                const int id = atoi(id_str);
                pair<int, string> itm;
                itm.first = id;
                itm.second = path + fd.name;
                m_nominal_image_set.push_back(itm);

#ifdef IS_VERBOSE
                cout << "Loading set " << set_id << " - " << fd.name << endl;
#endif
            }
        }
        while (_findnext(hFile, &fd) == 0 && m_nominal_image_set.size() < m_max_imgset_size);

#ifdef IS_VERBOSE
        cout << "-------------------------------------------------"  << endl;
        cout << m_nominal_image_set.size() << " Images loaded" << endl;
        cout << "-------------------------------------------------"  << endl;
#endif
    }
    _findclose(hFile);  // close the file handle
    return m_nominal_image_set.size();
}
#endif  // IS_WIN

#ifdef IS_LINUX
int ObjectiveTest::recursive(const string path)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(path.c_str())) == NULL && m_nominal_image_set.size() < m_max_imgset_size)
    {
#ifdef IS_VERBOSE
        cout << "Error(" << errno << ") opening " << path << endl;
        cout << path << endl;
        exit(-1);
#endif
        return -1;
    }

    int id = 0;

    while ((dirp = readdir(dp)) != NULL)
    {
        if ((dirp->d_name != string(".")) && (dirp->d_name != string("..")))
        {
#ifdef IS_VERBOSE
            cout << "File name: " << dirp->d_name << endl;
#endif
            pair<int, string> itm;
            itm.first = id;
            itm.second = path + dirp->d_name;

            m_nominal_image_set.push_back(itm);
        }

#ifdef IS_VERBOSE
        cout << "Loading set " << id << " - " << string(dirp->d_name) << endl;
#endif
    }
    closedir(dp);
    return m_nominal_image_set.size();
}
#endif  // IS_LINUX

//----------------------------------------------------------------------------
// Apply Incremental Distortions
// The distortion is applied only to the image 2
//----------------------------------------------------------------------------
bool ObjectiveTest::applyIncrementalDistortions(const Mat rgb_src, const Rect roi_1, const Rect roi_2, const float sub_ratio, const unsigned int tid)
{
    // #ifdef IS_VERBOSE
    //    cout << "--------------------------------------------------------" << endl;
    //    cout << "Tid: " << tid << endl;
    //    cout << "Image Id: " << m_vct_image_id[tid] << endl;
    //    cout << "Rgb Src Size: (ros" << rgb_src.cols << "," << rgb_src.cols << ")" << endl;
    //    cout << "Roi 1: " << roi_1.x << "  " << roi_1.x << endl;
    //    cout << "Roi 2: " << roi_2.x << "  " << roi_2.x << endl;
    //    cout << "Dx: " << m_dx[tid] << endl;
    //    cout << "Dy " << m_dy[tid] << endl;
    //    cout << "--------------------------------------------------------" << endl;
    // #endif

    // Resizes the images to eliminate Rotation and Perspective black borders
    const Rect sub_roi_1(roi_1.width * sub_ratio / 2, roi_1.height * sub_ratio / 2, roi_1.width * sub_ratio, roi_1.height * sub_ratio);
    const Rect sub_roi_2(roi_2.width * sub_ratio / 2, roi_2.height  * sub_ratio / 2, roi_2.width * sub_ratio, roi_2.height * sub_ratio);

    char val_width_1[256];
    char val_height_1[256];

    char val_width_2[256];
    char val_height_2[256];

    char val_dx[256];
    char val_dy[256];

    char val_img_id[128];
    memset(val_img_id, 0, 128);
    sprintf(val_img_id, "%04d", m_vct_image_id[tid]);
    string f_name = m_dst_path + "Image_" + string(val_img_id);

#ifdef IS_WIN
    if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif
    f_name += "/roi_1/";

#ifdef IS_WIN
    if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

    f_name += "roi_1_";
    // Creates the distortion subpattern for the image 1
    memset(val_width_1, 0, 128);

    sprintf(val_width_1, "%05d", sub_roi_1.width);
    f_name += string(val_width_1);
    f_name += "_";

    memset(val_height_1, 0, 128);
    sprintf(val_height_1, "%05d", sub_roi_1.height);
    f_name += string(val_height_1);
    f_name += ".png";

    pair<int, string> itm;
    itm.first = m_idx;
    itm.second = f_name;

    m_vct_roi_1[tid].push_back(itm);
    Mat rgb_roi_1(rgb_src(roi_1));
    imwrite(f_name, rgb_roi_1(Rect(sub_roi_1)));

    // Sets the order of directory creation : Set/Operation or Operation/Set
    const DirOrder dir_order = SET_TO_OPERATION;

    DistortionManager dist_man_2 = DistortionManager(m_path, dir_order, rgb_src(Rect(roi_2)), DISTORTION_COUNT, m_nbr_distortion_steps);
    dist_man_2.m_distortion_id = m_vct_distortion_id[tid];
    dist_man_2.setDistortionProperties();

    for (int s_idx = 0; s_idx < m_nbr_distortion_steps; s_idx++)
    {
        int i_bias = s_idx;

        // Converts unipolar index to bipolar required by Luminance bipolar range [-100..100]
        if (m_vct_distortion_id[tid] == DISTORTION_LUMINANCE ||
                m_vct_distortion_id[tid] == DISTORTION_SATURATION ||
                m_vct_distortion_id[tid] == DISTORTION_HUE) // ||
            //  m_distortion_id == DISTORTION_RESCALE) // ||
            //    m_distortion_id == DISTORTION_ROTATION)
        {
            i_bias += (m_nbr_distortion_steps / 2);
        }

        //----------------------------------------------------------------
        // Computes the Distortion Transform to the selected image crop
        //----------------------------------------------------------------
        dist_man_2.applyDistortionSteps(i_bias);

        char val_img_id[128];
        memset(val_img_id, 0, 128);
        sprintf(val_img_id, "%04d", m_vct_image_id[tid]);
        f_name = m_dst_path + "Image_" + string(val_img_id);

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif
        f_name += "/roi_2/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        f_name += dist_man_2.m_distort_name;
        f_name += "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif
        char val_i[128];
        memset(val_i, 0, 128);
        sprintf(val_i, "%02d", s_idx);

        f_name += string(val_i);
        f_name += "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif
        // Creates the distortion subpattern for the image 2
        f_name += "roi_2_";

        memset(val_dx, 0, 128);
        sprintf(val_dx, "%04d", m_dx[tid]);
        f_name += string(val_dx);
        f_name += "_";

        memset(val_dy, 0, 128);
        sprintf(val_dy, "%04d", m_dy[tid]);
        f_name += string(val_dy);
        f_name += "_";

        memset(val_width_2, 0, 128);
        sprintf(val_width_2, "%05d", sub_roi_2.width);
        f_name += string(val_width_2);
        f_name += "_";

        memset(val_height_2, 0, 128);
        sprintf(val_height_2, "%05d", sub_roi_2.height);
        f_name += string(val_height_2);

        f_name += ".png";

        const int idx = (float)s_idx/dist_man_2.m_nbr_distortion_steps;
        itm.first = idx;
        itm.second = f_name;
        m_vct_roi_2[tid].push_back(itm);
        imwrite(f_name, dist_man_2.m_rgb_dst(sub_roi_2));
    }
    return true;
}

//----------------------------------------------------------------------------
// applyDistortionStack
// Applies a stack of distortions to simulate camera real-life lightenins and
// posing distortions
//----------------------------------------------------------------------------
bool ObjectiveTest::applyDistortionStack(const Mat rgb_src, const Rect roi_1, const Rect roi_2, Mat &rgb_dist_1, Mat &rgb_dist_2)
{
    // imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/sub_nominal_ref.png", rgb_src);
    const DirOrder dir_order = SET_TO_OPERATION;
    // Sets the order of directory creation : Set/Operation or Operation/Set
    const string path("");
    DistortionManager dist_man_1 = DistortionManager(path, dir_order, rgb_src(Rect(roi_1)), 1, 1);

    dist_man_1.m_nbr_distortion_steps = 20;

    dist_man_1.m_distortion_id = DISTORTION_ROTATION;
    dist_man_1.setDistortionProperties();
    dist_man_1.applyDistortionSteps(0);

    const int edge = 0;
    Mat rot_roi = dist_man_1.m_rgb_dst(Rect(edge, edge, dist_man_1.m_rgb_dst.cols - (edge * 2), dist_man_1.m_rgb_dst.rows - (edge * 2)));
    rot_roi.copyTo(dist_man_1.m_rgb_dst);

    dist_man_1.m_distortion_id = DISTORTION_LUMINANCE;
    dist_man_1.m_rgb_src = dist_man_1.m_rgb_dst;
    dist_man_1.setDistortionProperties();
    dist_man_1.applyDistortionSteps(10);

    dist_man_1.m_distortion_id = DISTORTION_GAUSSIAN_BLUR;
    dist_man_1.m_rgb_src = dist_man_1.m_rgb_dst;
    dist_man_1.setDistortionProperties();
    dist_man_1.applyDistortionSteps(0);

    dist_man_1.m_distortion_id = DISTORTION_SALT_PEPPER_NOISE;
    dist_man_1.m_rgb_src = dist_man_1.m_rgb_dst;
    dist_man_1.setDistortionProperties();
    dist_man_1.applyDistortionSteps(0);

    dist_man_1.m_distortion_id = DISTORTION_CAMERA_NOISE;
    dist_man_1.m_rgb_src = dist_man_1.m_rgb_dst;
    dist_man_1.setDistortionProperties();
    dist_man_1.applyDistortionSteps(0);

    dist_man_1.m_distortion_id = DISTORTION_MOTION_BLUR;
    dist_man_1.m_rgb_src = dist_man_1.m_rgb_dst;
    dist_man_1.setDistortionProperties();
    dist_man_1.applyDistortionSteps(0);

    dist_man_1.m_distortion_id = DISTORTION_BARREL;
    dist_man_1.m_rgb_src = dist_man_1.m_rgb_dst;
    dist_man_1.setDistortionProperties();
    dist_man_1.applyDistortionSteps(20);

    // Applies a geometric transform to the sub image 1
    rgb_dist_1 = dist_man_1.m_rgb_dst.clone();

    // Sets the order of directory creation : Set/Operation or Operation/Setz
    DistortionManager dist_man_2= DistortionManager(path, dir_order, rgb_src(Rect(roi_2)), 1, 1);

    dist_man_2.m_nbr_distortion_steps = 20;

    dist_man_2.m_distortion_id = DISTORTION_ROTATION;
    dist_man_2.setDistortionProperties();
    dist_man_2.applyDistortionSteps(8);

    rot_roi = dist_man_2.m_rgb_dst(Rect(edge, edge, dist_man_2.m_rgb_dst.cols - (edge * 2), dist_man_2.m_rgb_dst.rows - (edge * 2)));
    rot_roi.copyTo(dist_man_2.m_rgb_dst);

    dist_man_2.m_distortion_id = DISTORTION_LUMINANCE;
    dist_man_2.m_rgb_src = dist_man_2.m_rgb_dst;
    dist_man_2.setDistortionProperties();
    dist_man_2.applyDistortionSteps(10);

    dist_man_2.m_distortion_id = DISTORTION_GAUSSIAN_BLUR;
    dist_man_2.m_rgb_src = dist_man_2.m_rgb_dst;
    dist_man_2.setDistortionProperties();
    dist_man_2.applyDistortionSteps(0);

    dist_man_2.m_distortion_id = DISTORTION_SALT_PEPPER_NOISE;
    dist_man_2.m_rgb_src = dist_man_2.m_rgb_dst;
    dist_man_2.setDistortionProperties();
    dist_man_2.applyDistortionSteps(0);

    dist_man_2.m_distortion_id = DISTORTION_CAMERA_NOISE;
    dist_man_2.m_rgb_src = dist_man_2.m_rgb_dst;
    dist_man_2.setDistortionProperties();
    dist_man_2.applyDistortionSteps(0);

    dist_man_2.m_distortion_id = DISTORTION_MOTION_BLUR;
    dist_man_2.m_rgb_src = dist_man_2.m_rgb_dst;
    dist_man_2.setDistortionProperties();
    dist_man_2.applyDistortionSteps(0);

    dist_man_2.m_distortion_id = DISTORTION_BARREL;
    dist_man_2.m_rgb_src = dist_man_2.m_rgb_dst;
    dist_man_2.setDistortionProperties();
    dist_man_2.applyDistortionSteps(0);

    rgb_dist_2 = dist_man_2.m_rgb_dst.clone();

    return true;
}

//-------------------------------------------------------------------------------------------
// 1. Comparative Stitching
// This function compares the nominal stitch obtained by regional merge-up
// of the undistorted image 1 with the distorted image 2, with no blending or seam applied.
//-------------------------------------------------------------------------------------------
bool ObjectiveTest::comparative_stitching_1(const Mat rgb_src_1,
                                            const Mat rgb_src_2,
                                            const vector<Point2f> corners_1,
                                            const vector<Point2f> corners_2,
                                            Mat &rgb_stitch_distort,
                                            Mat &rgb_stitch_nominal_1,
                                            Mat &rgb_stitch_nominal_2,
                                            vector<Point2f> &shared_region,
                                            const bool hide_seam,
                                            const bool generate_outline,
                                            const unsigned int tid)
{
    ImageStitching img_stitch;

    // string f_result_name = "stitch_1/";
    string f_name = m_dst_path; // + f_result_name;

#ifdef IS_WIN
    if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

    char val_d[128];
    memset(val_d, 0, 128);
    sprintf(val_d, "%02d", m_image_id[tid]);
    f_name += "Image_" + string(val_d) + "/";

#ifdef IS_WIN
    if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif
    string f_funct_name = "";

    shared_region[0].x = corners_2[0].x;
    shared_region[0].y = corners_2[0].y;
    shared_region[1].x = rgb_src_1.cols - corners_2[0].x;
    shared_region[1].y = rgb_src_1.rows - corners_2[0].y;

    // Size of the target canvas
    const int canvas_rows_max = rgb_src_2.rows + (corners_2[0].y);
    const int canvas_cols_max = rgb_src_2.cols + (corners_2[0].x);

    Mat canvas_1 = Mat::zeros(canvas_rows_max, canvas_cols_max, CV_8UC3);
    Mat canvas_2 = Mat::zeros(canvas_rows_max, canvas_cols_max, CV_8UC3);
    Mat canvas_3 = Mat::zeros(canvas_rows_max, canvas_cols_max, CV_8UC3);

    rgb_src_2.copyTo(canvas_3(Rect(corners_2[0].x, corners_2[0].y, rgb_src_2.cols, rgb_src_2.rows)));
    rgb_src_2.copyTo(canvas_2(Rect(corners_2[0].x, corners_2[0].y, rgb_src_2.cols, rgb_src_2.rows)));

    // Embeds the image 1 into the target canvas
    // The image 1 is shifted to fit the warped image 2
    rgb_src_1.copyTo(canvas_3(Rect(0, 0, rgb_src_1.cols, rgb_src_1.rows)));
    rgb_src_1.copyTo(canvas_1(Rect(0, 0, rgb_src_1.cols, rgb_src_1.rows)));

    //-------------------------------------------------------------------------------
    // Draws matching points
    //-------------------------------------------------------------------------------
    // Mat img_outline;
    // drawMatches(rgb_src_1, keypoint_img_1,rgb_src_2, keypoint_img_1,
    // robust_matcher->matchArray, img_outline, Scalar::all(-1), Scalar::all(-1),
    // vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-------------------------------------------------------------------------------
    // Creates the target images for rgb_src_1 and rgb_src_2
    //-------------------------------------------------------------------------------
    rgb_stitch_nominal_1 = Mat::zeros(canvas_1.rows, canvas_1.cols, CV_8UC3);
    rgb_stitch_nominal_2 = Mat::zeros(canvas_1.rows, canvas_1.cols, CV_8UC3);

    //-------------------------------------------------------------------------------
    // Reprojection
    //-------------------------------------------------------------------------------
    canvas_3(Rect(0, 0, canvas_cols_max, canvas_rows_max)).copyTo(rgb_stitch_nominal_1);
    canvas_3(Rect(0, 0, canvas_cols_max, canvas_rows_max)).copyTo(rgb_stitch_nominal_2);

    for (int r = 0; r < shared_region[1].y; r++)
    {
        for (int c = 0; c < shared_region[1].x; c++)
        {
            rgb_stitch_nominal_1.at<Vec3b>(r + shared_region[0].y, c + shared_region[0].x)[0] = rgb_src_2.at<Vec3b>(r, c)[0];
            rgb_stitch_nominal_1.at<Vec3b>(r + shared_region[0].y, c + shared_region[0].x)[1] = rgb_src_2.at<Vec3b>(r, c)[1];
            rgb_stitch_nominal_1.at<Vec3b>(r + shared_region[0].y, c + shared_region[0].x)[2] = rgb_src_2.at<Vec3b>(r, c)[2];
        }
    }

    //    const float step = (float)shared_region[1].x / shared_region[1].y;
    //    float cnt = 0.0;

    //     for (int r = 0; r < shared_region[1].y; r++)
    //    {
    //        for (int c = (int)cnt; c < shared_region[1].x; c++)
    //        {
    //            rgb_stitch_nominal_1.at<Vec3b>(r + shared_region[0].y, c + shared_region[0].x)[0] = rgb_src_2.at<Vec3b>(r, c)[0];
    //            rgb_stitch_nominal_1.at<Vec3b>(r + shared_region[0].y, c + shared_region[0].x)[1] = rgb_src_2.at<Vec3b>(r, c)[1];
    //            rgb_stitch_nominal_1.at<Vec3b>(r + shared_region[0].y, c + shared_region[0].x)[2] = rgb_src_2.at<Vec3b>(r, c)[2];
    //        }
    //        cnt += step;
    //    }

    vector<Point2f>shift;
    shift.push_back(corners_1[0]);
    shift.push_back(corners_2[0]);

    if (m_stitching_type == StitchingType::Watershed)  //  1. Watershed Stitching
    {
        //--------------------------------------------------------------------------
        // Begin of Marker creation
        //--------------------------------------------------------------------------
        Mat marker = Mat::zeros(canvas_1.rows, canvas_1.cols, CV_32SC1);
        marker.setTo(-1);

        // Embeds the image 1
        // The image 1 is shifted to fit the warped image 1
        marker(Rect(0, 0, rgb_src_1.cols - corners_1[0].x, rgb_src_1.rows - corners_1[0].y)) = 1;

        // Embeds the image 2
        marker(Rect(corners_2[0].x, corners_2[0].y, rgb_src_2.cols, rgb_src_2.rows)) = 2;

        // Embeds the shared region
        marker(Rect(shared_region[0].x, shared_region[0].y, shared_region[1].x, shared_region[1].y)) = -1;
        marker(Rect(shared_region[0].x, shared_region[0].y, shared_region[1].x, 1)) = 1;
        marker(Rect(shared_region[0].x, shared_region[0].y + shared_region[1].y - 1, shared_region[1].x, 1)) = 2;
        marker(Rect(shared_region[0].x, shared_region[0].y, 1, shared_region[1].y)) = 1;
        marker(Rect(shared_region[0].x + shared_region[1].x - 1, shared_region[0].y, 1, shared_region[1].y)) = 2;
        //--------------------------------------------------------------------------
        // End of Marker creation
        //--------------------------------------------------------------------------

        img_stitch.watershed(canvas_1, canvas_2, marker, rgb_stitch_distort, hide_seam);
        f_funct_name = "watershed";
    }

    else if (m_stitching_type == StitchingType::Feathering)  // 2. Feathering Stitching
    {
        img_stitch.blending(rgb_src_1,
                            rgb_src_2,
                            shift[0],
                shift[1],
                rgb_stitch_distort,
                Blender::FEATHER,
                20);

        f_funct_name = "feathering_blending";
    }
    else if (m_stitching_type == StitchingType::Multiband)   // 3. Multiband Feathering Stitching
    {
        img_stitch.blending(rgb_src_1,
                            rgb_src_2,
                            shift[0],
                shift[1],
                rgb_stitch_distort,
                Blender::MULTI_BAND,
                20);

        f_funct_name = "multiband_blending";
    }
    else if (m_stitching_type == StitchingType::Poisson)     // 4. Poisson Stitching
    {
        img_stitch.poisson(rgb_src_1, rgb_src_2, shift[0], shift[1], rgb_stitch_distort);
        f_funct_name = "poisson";
    }
    else if (m_stitching_type == StitchingType::Graphcut)    // 5. Graph-cut Stitching
    {
        img_stitch.graphcut(rgb_src_1, rgb_src_2, shift[0], shift[1], "gc_color", rgb_stitch_distort);
        f_funct_name = "graphcut";
    }

    else if (m_stitching_type == StitchingType::Voronoi)     // 6. Voronoi Stitching
    {
        img_stitch.graphcut(rgb_src_1, rgb_src_2, corners_1[0], corners_2[0], "voronoi", rgb_stitch_distort);
        f_name += "voronoi";
    }

    const DirOrder dir_order = SET_TO_OPERATION;
    DistortionManager dist_man = DistortionManager(m_path, dir_order, rgb_src_1, 1, 1);
    dist_man.m_distortion_id = (Distortions)m_distortion_id;
    dist_man.setDistortionProperties();

    if (m_generate_target_files == true)
    {
        f_name += f_funct_name + "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        f_name += dist_man.m_distort_name + "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        memset(val_d, 0, 128);
        sprintf(val_d, "%02d", m_strength);
        f_name += string(val_d) + "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        char val_shift_x[128];
        memset(val_shift_x, 0, 128);
        sprintf(val_shift_x, "%04d", shift[1].x);

        char val_shift_y[128];
        memset(val_shift_y, 0, 128);
        sprintf(val_shift_y, "%04d", shift[1].y);

        f_name += f_funct_name + "_";
        f_name +=  to_string(m_idx) + "_" + to_string(rgb_src_1.cols) + "_" + to_string(rgb_src_1.rows) + "_" + string(val_shift_x) + "_" + string(val_shift_y) + "_" + to_string(rgb_src_2.cols) + "_" + to_string(rgb_src_2.rows);
        f_name += ".png";
        imwrite(f_name, rgb_stitch_distort);

        if (generate_outline == true)
        {
            vector<Point2f> corners_22(4);
            corners_22[0] = cvPoint(round(corners_1[0].x), round(corners_1[0].y));
            corners_22[1] = cvPoint(round(corners_1[0].x) - 1 + rgb_src_1.cols, round(corners_1[0].y));
            corners_22[2] = cvPoint(round(corners_1[0].x) + rgb_src_1.cols - 1, round(corners_1[0].y) + rgb_src_1.rows - 1);
            corners_22[3] = cvPoint(round(corners_1[0].x), round(corners_1[0].y) + rgb_src_1.rows - 1);

            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line(rgb_stitch_distort, corners_22[0], corners_22[1], Scalar(0, 0, 255), 1);
            line(rgb_stitch_distort, corners_22[1], corners_22[2], Scalar(0, 0, 255), 1);
            line(rgb_stitch_distort, corners_22[2], corners_22[3], Scalar(0, 0, 255), 1);
            line(rgb_stitch_distort, corners_22[3], corners_22[0], Scalar(0, 0, 255), 1);

            line(rgb_stitch_distort, corners_2[0], corners_2[1], Scalar(0, 255, 0), 1);
            line(rgb_stitch_distort, corners_2[1], corners_2[2], Scalar( 0, 255, 0), 1);
            line(rgb_stitch_distort, corners_2[2], corners_2[3], Scalar( 0, 255, 0), 1);
            line(rgb_stitch_distort, corners_2[3], corners_2[0], Scalar( 0, 255, 0), 1);

            //----------------------------------------------------------------------
            // Dumps the Stitching outline
            //----------------------------------------------------------------------
            f_name = m_dst_path + "watershed_outline/";

#ifdef IS_WIN
            if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
            {
#ifdef IS_VERBOSE
                cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
            }
#endif

#ifdef IS_LINUX
            if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
            {
#ifdef IS_VERBOSE
                cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
            }
#endif

            f_name += dist_man.m_distort_name + "/";

#ifdef IS_WIN
            if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
            {
#ifdef IS_VERBOSE
                cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
            }
#endif

#ifdef IS_LINUX
            if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
            {
#ifdef IS_VERBOSE
                cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
            }
#endif

            char val_d[128];
            memset(val_d, 0, 128);
            sprintf(val_d, "%02d", m_strength);
            f_name += string(val_d) + "/";

#ifdef IS_WIN
            if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
            {
#ifdef IS_VERBOSE
                cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
            }
#endif

#ifdef IS_LINUX
            if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
            {
#ifdef IS_VERBOSE
                cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
            }
#endif

            f_name += "watershed_outline_";
            f_name += to_string(m_idx);
            f_name += ".png";
            imwrite(f_name, rgb_stitch_distort);
        }

        //--------------------------------------------------------------------
        // Dumps the nominal mosaicing
        //--------------------------------------------------------------------
        f_name = m_dst_path + "nominal/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        f_name += dist_man.m_distort_name + "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        memset(val_d, 0, 128);
        sprintf(val_d, "%02d", m_strength);
        f_name += string(val_d) + "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        f_name += "nominal_";
        f_name +=  to_string(m_idx) + "_" + to_string(rgb_src_1.cols) + "_" + to_string(rgb_src_1.rows) + "_" + to_string(rgb_src_2.cols) + "_" + to_string(rgb_src_2.rows);
        f_name += ".png";
        imwrite(f_name, rgb_stitch_nominal_1);
    }
    return true;
}

//-------------------------------------------------------------------------------------------
// 2. Comparative Stitching
// This function compares the selected stitch of the undistorted image 1 with the distorted
// image 2. The same stitching function is used at both sides.
//-------------------------------------------------------------------------------------------
bool ObjectiveTest::comparative_stitching_2(const Mat rgb_src_1,
                                            const Mat rgb_src_2,
                                            Mat &rgb_stitch_nominal,
                                            Mat &rgb_stitch_distort,
                                            const vector<Point2f> corners_2,
                                            const vector<Point2f> corners_1,
                                            const bool hide_seam
                                            )
{

    string f_result_name = "stitch_2/";
    string f_name = m_dst_path + f_result_name;

#ifdef IS_WIN
    if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

    char val_d[128];
    memset(val_d, 0, 128);
    sprintf(val_d, "%02d", m_image_id);
    ImageStitching img_stitch;
    f_name += "Image_" + string(val_d) + "/";

#ifdef IS_WIN
    if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif
    string f_funct_name = "";

    vector<Point> shared_region(2);
    shared_region[0].x = corners_2[0].x;
    shared_region[0].y = corners_2[0].y;
    shared_region[1].x = rgb_src_1.cols - corners_2[0].x;
    shared_region[1].y = rgb_src_1.rows - corners_2[0].y;

    // Size of the target canvas
    const int canvas_rows_max = rgb_src_2.rows + (corners_2[0].y);
    const int canvas_cols_max = rgb_src_2.cols + (corners_2[0].x);

    Mat canvas_1 = Mat::zeros(canvas_rows_max, canvas_cols_max, CV_8UC3);
    Mat canvas_2 = Mat::zeros(canvas_rows_max, canvas_cols_max, CV_8UC3);

    rgb_stitch_nominal = Mat::zeros(canvas_rows_max, canvas_cols_max, CV_8UC3);

    // Embeds the image 1 into the target canvas
    // The image 1 is shifted to fit the warped image 2
    rgb_src_1.copyTo(rgb_stitch_nominal(Rect(0, 0, rgb_src_1.cols, rgb_src_1.rows)));
    rgb_src_1.copyTo(canvas_1(Rect(0, 0, rgb_src_1.cols, rgb_src_1.rows)));

    rgb_src_2.copyTo(rgb_stitch_nominal(Rect(corners_2[0].x, corners_2[0].y, rgb_src_2.cols, rgb_src_2.rows)));
    rgb_src_2.copyTo(canvas_2(Rect(corners_2[0].x, corners_2[0].y, rgb_src_2.cols, rgb_src_2.rows)));

    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/nominal1.png.png", rgb_stitch_nominal);

    vector<Point2f>shift;
    shift.push_back(corners_1[0]);
    shift.push_back(corners_2[0]);

    if (m_stitching_type == StitchingType::Watershed)  //  1. Watershed Stitching
    {
        //--------------------------------------------------------------------------
        // Begin of Marker creation
        //--------------------------------------------------------------------------

        Mat marker = Mat::zeros(canvas_1.rows, canvas_1.cols, CV_32SC1);
        marker.setTo(-1);

        // Embeds the image 1
        // The image 1 is shifted to fit the warped image 1
        marker(Rect(0, 0, rgb_src_1.cols - corners_1[0].x, rgb_src_1.rows - corners_1[0].y)) = 1;
        //imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/test_watershed_marker_1.png", marker * 64);

        // Embeds the image 2
        marker(Rect(corners_2[0].x, corners_2[0].y, rgb_src_2.cols, rgb_src_2.rows)) = 2;
        // imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/test_watershed_marker_2.png", marker * 64);

        // Embeds the shared region
        marker(Rect(shared_region[0].x, shared_region[0].y, shared_region[1].x, shared_region[1].y)) = -1;
        marker(Rect(shared_region[0].x, shared_region[0].y, shared_region[1].x, 1)) = 1;
        marker(Rect(shared_region[0].x, shared_region[0].y + shared_region[1].y - 1, shared_region[1].x, 1)) = 2;
        marker(Rect(shared_region[0].x, shared_region[0].y, 1, shared_region[1].y)) = 1;
        marker(Rect(shared_region[0].x + shared_region[1].x - 1, shared_region[0].y, 1, shared_region[1].y)) = 2;
        //--------------------------------------------------------------------------
        // End of Marker creation
        //--------------------------------------------------------------------------

        // imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/test_canvas_crop_1.png", canvas_crop_1);
        // imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/test_canvas_crop_2.png", canvas_crop_2);
        img_stitch.watershed(canvas_1, canvas_2, marker, rgb_stitch_distort, hide_seam);
        f_funct_name = "watershed";
    }

    else if (m_stitching_type == StitchingType::Feathering)  // 2. Feathering Stitching
    {
        img_stitch.blending(rgb_src_1,
                            rgb_src_2,
                            shift[0],
                shift[1],
                rgb_stitch_distort,
                Blender::FEATHER,
                20);

        f_funct_name = "feathering_blending";
    }
    else if (m_stitching_type == StitchingType::Multiband)   // 3. Multiband Feathering Stitching
    {
        img_stitch.blending(rgb_src_1,
                            rgb_src_2,
                            shift[0],
                shift[1],
                rgb_stitch_distort,
                Blender::MULTI_BAND,
                20);

        f_funct_name = "multiband_blending";
    }
    else if (m_stitching_type == StitchingType::Poisson)     // 4. Poisson Stitching
    {
        img_stitch.poisson(rgb_src_1, rgb_src_2, shift[0], shift[1], rgb_stitch_distort);
        f_funct_name = "poisson";
    }
    else if (m_stitching_type == StitchingType::Graphcut)    // 5. Graph-cut Stitching
    {
        img_stitch.graphcut(rgb_src_1, rgb_src_2, shift[0], shift[1], "gc_color", rgb_stitch_distort);
        f_funct_name = "graphcut";
    }

    else if (m_stitching_type == StitchingType::Voronoi)     // 6. Voronoi Stitching
    {
        img_stitch.graphcut(rgb_src_1, rgb_src_2, corners_1[0], corners_2[0], "voronoi", rgb_stitch_distort);
        f_name += "voronoi";
    }

    const DirOrder dir_order = SET_TO_OPERATION;
    DistortionManager dist_man = DistortionManager(m_path, dir_order, rgb_src_1, 1, 1);
    dist_man.m_distortion_id = (Distortions)m_distortion_id;
    dist_man.setDistortionProperties();

    if (m_generate_target_files == true)
    {
        f_name += f_funct_name + "/";

        //string path_dst("D:/PRJ/Benchmark/OBJECTIVE_TEST/");

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        f_name += dist_man.m_distort_name + "/";
        //  string f_name = m_path + "result/watershed/" + dist_man.m_distort_name + "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        memset(val_d, 0, 128);
        sprintf(val_d, "%02d", m_strength);
        f_name += string(val_d) + "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        char val_shift_x[128];
        memset(val_shift_x, 0, 128);
        sprintf(val_shift_x, "%04d", shift[1].x);

        char val_shift_y[128];
        memset(val_shift_y, 0, 128);
        sprintf(val_shift_y, "%04d", shift[1].y);

        f_name += f_funct_name + "_";
        f_name +=  to_string(m_idx) + "_" + to_string(rgb_src_1.cols) + "_" + to_string(rgb_src_1.rows) + "_" + string(val_shift_x) + "_" + string(val_shift_y) + "_" + to_string(rgb_src_2.cols) + "_" + to_string(rgb_src_2.rows);
        f_name += ".png";
        imwrite(f_name, rgb_stitch_distort);
        // imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/stitch_2.png", rgb_stitch_distort);

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        f_name += dist_man.m_distort_name + "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        char val_d[128];
        memset(val_d, 0, 128);
        sprintf(val_d, "%02d", m_strength);
        f_name += string(val_d) + "/";

#ifdef IS_WIN
        if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

#ifdef IS_LINUX
        if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
        }
#endif

        f_name += "watershed_outline_";
        f_name += to_string(m_idx);
        f_name += ".png";
        imwrite(f_name, rgb_stitch_distort);
    }

    //--------------------------------------------------------------------
    // Dumps the nominal mosaicing
    //--------------------------------------------------------------------
    f_name = m_dst_path + f_result_name + "nominal/";

#ifdef IS_WIN
    if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

    f_name += dist_man.m_distort_name + "/";

#ifdef IS_WIN
    if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

    memset(val_d, 0, 128);
    sprintf(val_d, "%02d", m_strength);
    f_name += string(val_d) + "/";

#ifdef IS_WIN
    if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

    f_name += "nominal_";
    f_name +=  to_string(m_idx) + "_" + to_string(rgb_src_1.cols) + "_" + to_string(rgb_src_1.rows) + "_" + to_string(rgb_src_2.cols) + "_" + to_string(rgb_src_2.rows);
    f_name += ".png";
    imwrite(f_name, rgb_stitch_nominal);

    return true;
}

//------------------------------------------------------------------------------------------------------------

bool ObjectiveTest::generateGnuPlot(const string export_name,
                                    ExportDictionary &xl_dict,
                                    const int f_idx,
                                    const int d_idx,
                                    const bool is_log_scale,
                                    const bool skip_row_0)
{
#ifdef ENABLE_GNU_PLOT
    GnuPlotExport gnu;

#ifdef IS_WIN
    if (mkdir(string(m_path + "Export").c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create log folder [" << string(m_path + "Export") << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(string(m_path + "Export").c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << string(m_path + "Export") << "] : " << strerror(errno) << endl;
#endif
    }
#endif

    const string plot_ext = ".png";
    const string file_name = m_path + "Export/export_" + export_name + "_" + currentDateTime();
    const string CSV_file = file_name + "_" + xl_dict.m_header[0] + ".csv";

    if (gnu.exportColumnToCSV(CSV_file,
                              xl_dict,
                              f_idx,
                              d_idx,
                              skip_row_0) == false)
    {
#ifdef IS_VERBOSE
        cout << "Export to CSV failed !!" << endl;
#endif
        return false;
    }

    const string x_label = "step";

    if (gnu.generateGnuPlot(m_path,
                            file_name,
                            plot_ext,
                            CSV_file,
                            xl_dict,
                            x_label,
                            export_name,
                            is_log_scale) == false)
    {
#ifdef IS_VERBOSE
        cout << "GNU File plot failed  !" << endl;
#endif
        return false;
    }
#endif

    return true;
}

//------------------------------------------------------------------------------------------------------------

bool ObjectiveTest::generateMergedGnuPlot(const string export_name,
                                          ExportDictionary &xl_dict,
                                          const int d_idx,
                                          const bool is_log_scale,
                                          const bool skip_row_0)
{
#ifdef ENABLE_GNU_PLOT
    GnuPlotExport gnu;

    const string f_name(m_path + "Export/Merged");

#ifdef IS_WIN
    if (mkdir(f_name.c_str()) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << f_name << "] : " << strerror(errno) << endl;
#endif
    }
#endif

#ifdef IS_LINUX
    if (mkdir(f_name.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << string(f_name.c_str()) << "] : " << strerror(errno) << endl;
#endif
    }
#endif

    const string plot_ext = ".png";
    const string file_name = m_path + "Export/Merged/export_"+ export_name + "_" + currentDateTime();
    const string CSV_file = file_name + ".csv";

    if (gnu.exportMergedColumnsToCSV(CSV_file,
                                     xl_dict,
                                     d_idx,
                                     skip_row_0) == false)
    {
#ifdef IS_VERBOSE
        cout << "Export to CSV failed !!" << endl;
#endif
        return false;
    }
    const string x_label = "step";

    if (gnu.generateMergedGnuPlot(m_path,
                                  file_name,
                                  plot_ext,
                                  CSV_file,
                                  xl_dict,
                                  x_label,
                                  export_name,
                                  is_log_scale) == false)
    {
#ifdef IS_VERBOSE
        cout << "Merge GNU File plot failed  !" << endl;
#endif
        return false;
    }

#endif

    return true;
}
//------------------------------------------------------------------------------------------------------------

string ObjectiveTest::get_stitching_fnct_name(const int fnct_id)
{
    switch (fnct_id)
    {
    case Nominal:
        return "Nominal";

    case Feathering:
        return "Feathering";

    case Multiband:
        return "Multiband";

    case Poisson:
        return "Poisson";

    case Watershed:
        return "Watershed";

    case Graphcut:
        return "Graphcut";

    case Voronoi:
        return "Voronoi";

    case External:
        return "External";

    case None:
        return "None";
    }
}
