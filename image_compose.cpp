#include "image_compose.h"
#include "excelexport.h"
#include "gnuplotexport.h"
#include "distortionmanager.h"
#include "imagestitching.h"
#include "lib_math.h"
#include "luc/WatershedMarkerSoille.hpp"

#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include <string>
#include <chrono>
#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/flann/flann.hpp>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#ifdef IS_LINUX
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#endif

#define KERNEL_SIZE 21

using namespace std;


ImageCompose::ImageCompose()
{

}

//------------------------------------------------------------------------------------------------------------

ImageCompose::ImageCompose(const uint32_t img_set_size, const uint32_t nbr_distortions, const uint32_t nbr_steps, const string path)
{
    if (nbr_distortions > DISTORTION_COUNT)
    {
        cout << "The number of distortions: " << nbr_distortions << " should be <= " << DISTORTION_COUNT << endl;
        exit(-1);
    }

    m_path = path;
    m_nbr_distortions = nbr_distortions;
    m_ptr_marker = 0;
    const DirOrder dir_order = SET_TO_OPERATION;            // Sets the order of directory creation : Set/Operation or Operation/Set
    m_dist_man = DistortionManager(path, dir_order, m_rgb_src, m_nbr_distortions, nbr_steps);

    // Initializes the struct multicube
    initCube(img_set_size);
}

//------------------------------------------------------------------------------------------------------------

ImageCompose::~ImageCompose()
{

}

//------------------------------------------------------------------------------------------------------------

void ImageCompose::setDirOrder(const DirOrder order)
{
    m_dir_order = order;
}

//------------------------------------------------------------------------------------------------------------

bool ImageCompose::extractIdFromFileName(string file_name, int &set_id)
{
    if (file_name.substr(0, 4).compare("Img_") != 0)
    {
        set_id = -1;
        return false;
    }

    // Detects a new image set
    if (file_name.size() > 8)
    {
        string s_set = file_name.substr(file_name.size() - 8, 4);
        set_id = atoi(s_set.c_str());
    }
    return true;
}

//------------------------------------------------------------------------------------------------------------

void ImageCompose::initCube(const uint32_t img_set_size)
{
    // Image set dimension
    for (uint32_t i = 0; i < img_set_size; i++)
    {
        DistortSetArea dis_set;

        m_area_cube.push_back(dis_set);

        // Distortion type dimension
        for (uint32_t j = 0; j < m_nbr_distortions; j++)
        {
            SingleDistortArea dis;
            m_area_cube[i].push_back(dis);

            // Distortion single step dimension
            for (uint32_t k = 0; k < m_dist_man.m_distortion_step; k++)
            {
                m_area_cube[i][j].push_back(0.0);
            }
        }
    }
}

//------------------------------------------------------------------------------------------------------------

bool ImageCompose::generateDistortedImageset(const string sub_path)
{
    // Scans the entire benchmark image set
    for (uint32_t i = 0; i < m_image_set.size(); i++)
    {
        m_set_id = i;

        // Scans the entire stack of the image i of the benchmark set
        cout << "Reading image : " << i << endl;
        cout << "--------------------------------------------------------------------" << endl;

        string file_in = m_path + sub_path + m_image_set[i];
        m_rgb_src = imread(file_in, CV_LOAD_IMAGE_COLOR);

        if (m_rgb_src.rows == 0 || m_rgb_src.cols == 0)
        {
            cout << "Source file: " << file_in << " is empty !!" << endl;
            return false;
        }

        cout << "Image " << i << "  " << m_image_set[i] << " - (" << m_rgb_src.rows << ", " << m_rgb_src.cols << ")" << " loaded" << endl;
        cout << "--------------------------------------------------------------------" << endl;

#ifdef HAS_TIMER
        m_distort_set_ck_start = clock();
#endif

#ifdef UNIT_TEST
        const uint32_t nbr_test_cycles = 20;
        HSVManager hsv;

        //    cout << "------------------------------------------------------------------------------" << endl;
        //    cout << " Begin Test Luminance" << endl;
        //    cout << "------------------------------------------------------------------------------" << endl;
        hsv.unitTest(TestLuminance, nbr_test_cycles);
        //    cout << "------------------------------------------------------------------------------" << endl;
        //    cout << " End Test Luminance" << endl;
        //    cout << "------------------------------------------------------------------------------" << endl;

        //    cout << "------------------------------------------------------------------------------" << endl;
        //    cout << " Begin Test Saturation" << endl;
        //    cout << "------------------------------------------------------------------------------" << endl;
        hsv.unitTest(TestSaturation, nbr_test_cycles);
        //    cout << "------------------------------------------------------------------------------" << endl;
        //    cout << " End Test Saturation" << endl;
        //    cout << "------------------------------------------------------------------------------" << endl;

        //    cout << "------------------------------------------------------------------------------" << endl;
        //    cout << " Begin Test Hue" << endl;
        //    cout << "------------------------------------------------------------------------------" << endl;
        hsv.unitTest(TestHue, nbr_test_cycles);
        //    cout << "------------------------------------------------------------------------------" << endl;
        //    cout << " End Test Hue" << endl;
        //    cout << "------------------------------------------------------------------------------" << endl;

        // Creates a distorted image set
        // Applies the entire set of distortions to the input image
        dist_man = DistortionManager(m_rgb_src, nbr_test_cycles);
        dist_man.m_path = m_path;

        //    cout << "------------------------------------------------------------------------------" << endl;
        //    cout << " Begin Test Barrel" << endl;
        //    cout << "------------------------------------------------------------------------------" << endl;
        dist_man.testBarrelDistortion(nbr_test_cycles, 1);
        //    cout << "------------------------------------------------------------------------------" << endl;
        //    cout << " End Test Barrel" << endl;
        //    cout << "------------------------------------------------------------------------------" << endl;

        //    cout << "------------------------------------------------------------------------------" << endl;
        //    cout << " Begin Test Vignetting" << endl;
        //    cout << "------------------------------------------------------------------------------" << endl;
        dist_man.testVignettingDistortion(nbr_test_cycles, 1);
        //    cout << "------------------------------------------------------------------------------" << endl;
        //    cout << " End Test Vignetting" << endl;
        //    cout << "------------------------------------------------------------------------------" << endl;
#endif

        // m_dist_man.m_rgb_src = m_rgb_src;
        m_rgb_src.copyTo(m_dist_man.m_rgb_src);

        //-------------------------------------------------------------------------
        // Computes the entire set of photometric and geometric distortions to the
        // selected image and stores the results into destination image files
        //-------------------------------------------------------------------------
        m_dist_man.applyAllDistortionsToImage(m_set_id);

#ifdef HAS_TIMER
        m_distort_set_ck_stop = clock();
        m_distort_set_time = diffclock(m_distort_set_ck_stop, m_distort_set_ck_start);
        cout << "Elapsed time: " << m_distort_set_time << endl;
#endif
    }
    return true;
}

//----------------------------------------------------------------------------------------------

string ImageCompose::setFileName(const string file_name)
{
    char s_strength[4];
    sprintf(s_strength, "%.3f", m_dist_man.m_vect_strength[m_dist_man.m_distortion_id]);
    int len_1 = strlen(s_strength);
    s_strength[len_1 - 4] = '_';

    char s_set[255];
    sprintf(s_set, "%04d", m_set_id);

    string new_path = "";

    if (m_dir_order == OPERATION_TO_SET)
    {
        new_path = m_path + file_name;

#ifdef IS_WIN
        if (mkdir(new_path.c_str()) != 0 && errno != EEXIST)
        {
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
        }
#endif

#ifdef IS_LINUX
        if (mkdir(new_path.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
        }
#endif
        new_path = new_path + "/" + string("Set_") + s_set;

#ifdef IS_WIN
        if (mkdir(new_path.c_str()) != 0 && errno != EEXIST)
        {
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
        }
#endif

#ifdef IS_LINUX
        if (mkdir(new_path.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
        }
#endif

    }
    else // SET_TO_OPERATION
    {
        new_path = m_path + string("Set_") + s_set;

#ifdef IS_WIN
        if (mkdir(new_path.c_str()) != 0 && errno != EEXIST)
        {
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
        }
#endif

#ifdef IS_LINUX
        if (mkdir(new_path.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
        }
#endif

        new_path = new_path + "/" + file_name;

#ifdef IS_WIN
        if (mkdir(new_path.c_str()) != 0 && errno != EEXIST)
        {
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
        }
#endif

#ifdef IS_LINUX
        if (mkdir(new_path.c_str(), S_IRWXU) != 0 && errno != EEXIST)
        {
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
        }
#endif
    }

    //    new_path = new_path + "/" + m_dist_man.m_path_operation; // + "/";

#ifdef IS_WIN
    if (mkdir(new_path.c_str()) != 0 && errno != EEXIST)
    {
        cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
    }
#endif

#ifdef IS_LINUX
    if (mkdir(new_path.c_str(), S_IRWXU) != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
    }
#endif
    return new_path + "/" + "Img_" + s_set + "_" + s_strength + ".png";
}

//------------------------------------------------------------------------------------------------------------

string ImageCompose::setFileName2(const string path_name, const string file_name, const int hide_numbers)
{
    int ret = -1;
    char s_set[255];
    sprintf(s_set, "%04d", m_set_id);

    char s_step[255];
    sprintf(s_step, "%04d", m_step_id);

    string new_path = "";

    if (m_dir_order == OPERATION_TO_SET)
    {
        new_path = m_path + path_name;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }

        new_path = new_path + "/" + string("Set_") + s_set;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }
    }
    else // SET_TO_OPERATION
    {
        new_path = m_path + string("Set_") + s_set;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }

        new_path = new_path + "/" + path_name;

#ifdef IS_WIN
        ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
        ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
        if (ret != 0 && errno != EEXIST)
        {
#ifdef IS_VERBOSE
            cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
        }
    }

#ifdef IS_WIN
    ret = mkdir(new_path.c_str());
#endif
#ifdef IS_LINUX
    ret = mkdir(new_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
    if (ret != 0 && errno != EEXIST)
    {
#ifdef IS_VERBOSE
        cout << "cannot create folder [" << new_path << "] : " << strerror(errno) << endl;
#endif
    }

    if (hide_numbers == 1)
        return new_path + "/" + file_name + ".png";
    else
        return new_path + "/" + file_name + s_set + "_" + s_step + ".png";
}

//------------------------------------------------------------------------------------------------------------

bool ImageCompose::applyMaskToComposite(const Mat rgb_nom, const Mat rgb_dist, Mat &patch, Mat &rgb_composed)
{
    if (m_ptr_marker->rows == 0 ||
            m_ptr_marker->cols == 0 ||
            m_ptr_marker->rows != rgb_composed.rows ||
            m_ptr_marker->cols != rgb_composed.cols)
    {
#ifdef IS_VERBOSE
        cout << "Input image mismatch !!" << endl;
#endif
        return false;
    }

    // imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/rgb_nom.png", rgb_nom);
    //  imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/rgb_dist.png", rgb_dist);

    // Applies the masks to compose the input images into a final mosaic
    for (uint32_t r = 0; r < m_ptr_marker->rows; r++)
    {
        for (uint32_t c = 0; c < m_ptr_marker->cols; c++)
        {
            uint8_t mrk = m_ptr_marker->at<uint8_t>(r, c);
            if (mrk == 1)
            {
                patch.at<uint8_t>(r, c) = 64;
                rgb_composed.at<Vec3b>(r, c) = rgb_nom.at<Vec3b>(r, c);
            }
            else if (mrk == 2)
            {
                patch.at<uint8_t>(r, c) = 128;
                rgb_composed.at<Vec3b>(r, c) = rgb_dist.at<Vec3b>(r, c);
            }
            else if (mrk == 0)
            {
                patch.at<uint8_t>(r, c) = 255;
            }
        }
    }
    return true;
}

//------------------------------------------------------------------------------------------------------------

bool ImageCompose::computeInterseam(const int k)
{
    // Calculates the interseam area
    if (k < 1)
    {
        return false;
    }

    if (m_mask_out.size() < 2)
    {
        return false;
    }

    Size siz = m_mask_out[0].size();
    m_step_interseam_overlay.setTo(0);
    m_step_interseam.push_back(Mat(siz, CV_8U));

    imwrite(setFileName("testInterseam0").data(), m_mask_out[0]);
    if (m_mask_out.size() > 1)
    {
        imwrite(setFileName("testInterseam1").data(), m_mask_out[1]);
    }

    uint32_t rows = m_mask_out[k].rows;
    uint32_t cols =  m_mask_out[k].cols;

    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            if (m_mask_out[0].at<int8_t>(r, c) == m_mask_out[k].at<int8_t>(r, c))
            {
                m_step_interseam[k - 1].at<uint8_t>(r, c) = 0;
            }
            else
            {
                m_step_interseam[k - 1].at<uint8_t>(r, c) = 128;
            }
        }
    }
    imwrite(setFileName("/interseam_").data(), m_step_interseam[k - 1]);

    return true;
}

//------------------------------------------------------------------------------------------------------------

bool ImageCompose::exportToFile(SetArea area_cube, vector<float> mean_images)
{
    //    for (int i = 0; i < m_image_set.size(); i++)
    //    {
    //        for (int j = 0; j < m_nbr_distortions; j++)
    //        {
    //            setDistortionProperties((Distortions)j);

    //            for (int k = 0; k < m_dist_man.m_distortion_step; k++)
    //            {
    //                // The values originated from different images are summed up
    //                // with the the values computed at each step
    //                xl_sheet[j][k] += area_cube[i][j][k];
    //            }
    //        }
    //    }
    //    xl_dict.m_data.push_back(xl_sheet);

#ifdef ENABLE_EXCEL
    ExcelExport exc;

    if (exc.createExcelFile(m_path + "Excel/excel_" + currentDateTime() + ".xls", xl_dict, true) == false)
    {
        return false;
    }

#endif

#ifdef ENABLE_GNU_PLOT
    GnuPlotExport gnu;

    const string CSV_file = m_path + "Export/export_" + currentDateTime() + ".csv";

    if (gnu.exportColumnToCSV(CSV_file,
                              xl_dict,
                              0,
                              0,
                              false) == false)
    {
#ifdef IS_VERBOSE
        cout << "Eport to CSV failed !!" << endl;
#endif
        return false;
    }

    const string plot_file = m_path + "Export/export_" + currentDateTime();
    const string plot_ext = ".png";

    if (gnu.generateGnuPlot(m_path,
                            plot_file,
                            plot_ext,
                            CSV_file,
                            xl_dict,
                            "Step",
                            "Normalized Interseam",
                            true) == false)
    {
#ifdef IS_VERBOSE
        cout << "GNU File plot failed  !" << endl;
#endif
        return false;
    }

    if (gnu.generateMergedGnuPlot(m_path,
                                  plot_file,
                                  plot_ext,
                                  CSV_file,
                                  xl_dict,
                                  "Step",
                                  "Normalized Interseam", true) == false)
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

bool ImageCompose::distance(const Mat &in, Mat &dist)
{
    const uint32_t distort_max = 256;
    const float step = 2;

    for (int r = 0;  r < in.rows; r++)
    {
        for (int c = 0;  c < in.cols; c++)
        {
            if (in.at<uint8_t>(r, c) <= 16)
            {
                float wr = 0.0001;
                float wc = 0.0001;

                // Up
                for (int dr = 0; dr < distort_max; dr++)
                {
                    if ((r + dr < in.rows) && (in.at<uint8_t>(r + dr, c) > 64))
                    {
                        wr = dr * step; //((float)dr/distort_max) * 64; // / dr;
                        break;
                    }
                }

                // Down
                for (int dr = 0; dr < distort_max; dr++)
                {
                    if ((r - dr > 0) && (in.at<uint8_t>(r - dr, c) > 64))
                    {
                        wr -= dr * step; //((float)distort_max/dr) * 64; // / dr;
                        break;
                    }
                }

                // Right
                for (int dc = 0; dc < distort_max; dc++)
                {
                    if ((c + dc < in.cols) && (in.at<uint8_t>(r, c + dc) > 64))
                    {
                        wc = dc * step; //distort_max / dc;
                        break;
                    }
                }

                // Left
                for (int dc = 0; dc < distort_max; dc++)
                {
                    if ((c - dc > 0) && (in.at<uint8_t>(r, c - dc) > 64))
                    {
                        wc -= dc * step;
                        break;
                    }
                }
                wr = abs(1/wr) * 255;
                wc = abs(1/wc) * 255;

                float val = sqrt((wr * wr) + (wc * wc));
                if (val > 255)
                {
                    dist.at<uint8_t>(r, c) = 255;
                }
                else
                {
                    dist.at<uint8_t>(r, c) = val;
                }
            }
        }
    }
    return true;
}

//------------------------------------------------------------------------------------------------------------
// Shared region pixel writing on the target mask
void ImageCompose::projection(const Mat grad_nom, const Mat grad_dist, Mat &mask)
{
    Mat a = Mat::zeros(grad_nom.size(), grad_nom.type());
    Mat b = Mat::zeros(grad_dist.size(), grad_dist.type());

    grad_nom.convertTo(a, CV_8UC1);
    grad_dist.convertTo(b, CV_8UC1);

    for (uint32_t r = 0; r < mask.rows; r++)
    {
        //for (uint32_t c = 1; c < mask.cols; c++)
        for (uint32_t c = 0; c < mask.cols; c++)
        {
            // Checks that the verification stays in the boundary of the image definition
            if ((int8_t)a.at<int8_t>(r, c) >= (int8_t)b.at<int8_t>(r, c))
            {
                mask.at<int8_t>(r, c) = (int8_t)(a.at<int8_t>(r, c));
            }
            else
            {
                mask.at<int8_t>(r, c) = (int8_t)(b.at<int8_t>(r, c));
            }
        }
    }
}

//------------------------------------------------------------------------------------------------------------

//void ImageCompose::projection2(const Mat grad_nom, const Mat grad_dist, Mat &mask)
//{
//    Mat a = Mat::zeros(grad_nom.size(), grad_nom.type());
//    Mat b = Mat::zeros(grad_dist.size(), grad_dist.type());

//    grad_nom.convertTo(a, CV_8UC1);
//    grad_dist.convertTo(b, CV_8UC1);

//    mask.setTo(-1);

//    for (uint32_t r = 0; r < mask.rows; r++)
//    {
//        for (uint32_t c = 0; c < mask.cols; c++)
//        {
//            // Checks that the verification stays in the boundary of the image definition
//            if ((int8_t)a.at<int8_t>(r, c) >= (int8_t)b.at<int8_t>(r, c))
//            {
//                mask.at<int8_t>(r, c) = (int8_t)(a.at<int8_t>(r, c));
//            }
//            else
//            {
//                mask.at<int8_t>(r, c) = (int8_t)(b.at<int8_t>(r, c));
//            }
//        }
//    }
//}

//------------------------------------------------------------------------------------------------------------

void ImageCompose::setDistortionProperties(const Distortions distort_id)
{

    m_distort_id = distort_id;

    switch(m_distort_id)
    {
    case DISTORTION_LUMINANCE:
        m_distort_name = "Luminance";
        break;

        //    case DISTORTION_SATURATION:
        //        m_distort_name = "Saturation";
        //        break;

        //    case DISTORTION_HUE:
        //        m_distort_name = "Hue";
        //        break;

    case DISTORTION_GAUSSIAN_BLUR:
        m_distort_name = "GaussianBlur";
        break;

    case DISTORTION_MOTION_BLUR:
        m_distort_name = "MotionBlur";
        break;

    case DISTORTION_SALT_PEPPER_NOISE:
        m_distort_name = "SaltAndPepperNoise";
        break;

    case DISTORTION_CAMERA_NOISE:
        m_distort_name = "CameraNoise";
        break;

    case DISTORTION_SHIFT:
        m_distort_name = "Shift";
        break;

    case DISTORTION_ROTATION:
        m_distort_name = "Rotation";
        break;

        //    case DISTORTION_RESCALE:
        //        m_distort_name = "Rescale";
        //        break;

        //    case DISTORTION_PERSPECTIVE:
        //        m_distort_name = "Perspective";
        //        break;

    case DISTORTION_PROJECTION_3D:
        m_distort_name = "Projection3D";
        break;

        //    case DISTORTION_MOVING_OBJECT:
        //        m_distort_name = "MovingObject";
        //        break;

    case DISTORTION_VIGNETTING:
        m_distort_name = "Vignetting";
        break;

    case DISTORTION_BARREL:
        m_distort_name = "Barrel";
        break;

        //    case DISTORTION_SHADOW:
        //        m_distort_name = "Shadow";
        //        break;
    }
}

//------------------------------------------------------------------------------------------------------------

bool ImageCompose::compose(vector<Mat> gray_in)
{
    Mat rgb_nom;
    Mat nom_float;
    Mat hsv_1;
    Mat ch_1[3];
    Mat gray_nom;

    Mat rgb_dist;
    Mat dist_float;
    Mat hsv_2;
    Mat ch_2[3];
    Mat gray_dist;

    Mat grad_nom;
    Mat grad_dist;
    Mat mask;

    Mat patch_0;
    Mat patch_k;

    Mat interseam;

    vector<float> mean_steps;
    vector<float> mean_images;

    xl_header.clear();
    xl_cols.clear();
    xl_sheet.clear();

    for (uint8_t x = 0; x < m_nbr_distortions; x++)
    {
        mean_images.push_back(0.0);
        mean_steps.push_back(0.0);
    }

    // Initialization of the matrix xl
    for (int j = 0; j < m_nbr_distortions; j++)
    {
        setDistortionProperties((Distortions)j);

        // Appends the Header column
        xl_header.push_back(m_distort_name);
        xl_dict.m_header = xl_header;

        // Appends the empty Data dictionary
        // ExportColumn xl_cols;

        xl_cols.clear();
        for (int k = 0; k < m_dist_man.m_distortion_step; k++)
        {
            // Rgi 08/04/2017
            // xl_cols.push_back(0.0);
            // Rgi 08/04/2017
        }
        xl_sheet.push_back(xl_cols);
    }

#ifdef EXPORT_UNIT_TEST
    //---------------------------------------------------------------------------
    // Begin Unit Test
    // Test Result: The resulting values and the chart are supposed to be zero
    //---------------------------------------------------------------------------
    m_area_cube.clear();

    initCube(m_image_set.size());
    int val = 0;

    // Loops for all images in the imageset
    for (int i = 0; i < nbr_img; i++)
    {
        // Loops for all distortion types
        for (int j = 0; j < m_nbr_distortions; j++)
        {
            val = 0;

            // Loops for all steps
            for (int k = 0; k < m_dist_man.m_distortion_step; k++)
            {
                if (i % 2 == 0)
                {
                    m_area_cube[i][j][k] = val * (j + 1);
                }
                else
                {
                    m_area_cube[i][j][k] = -val * (j + 1);
                }
                val++;
            }
        }
    }

    // Exports the Interseam results to an external sheet
    if (exportToFile(m_area_cube, mean_images) == false)
    {
        return false;
    }
    return true;
    //---------------------------------------------------------------
    // End Unit Test
    //--------------------------------------------------------------
#endif

    for (uint32_t i = 0; i < m_image_set.size(); i++)
    {
        m_set_id = i;

        mean_steps.clear();
        for (uint8_t x = 0; x < m_nbr_distortions; x++)
        {
            mean_steps.push_back(0.0);
        }

        //--------------------------------------------------------------------------
        // Loads the nominal image
        //--------------------------------------------------------------------------
        const string file_src_nom = m_path + "NominalSub/" + m_image_set[i];
        rgb_nom = imread(file_src_nom, CV_LOAD_IMAGE_COLOR);

        if (rgb_nom.rows == 0 || rgb_nom.cols == 0)
        {
#ifdef IS_VERBOSE
            cout << "File: " << file_src_nom << " not found !!" << endl;
#endif
            return false;
        }
        else
        {
#ifdef IS_VERBOSE
            cout << "Nominal File: " << file_src_nom << " loaded" << endl;
#endif
        }
        //--------------------------------------------------------------------------
        // Begin Conversion nominal image RGB -> Gray
        //--------------------------------------------------------------------------
#ifdef IS_VERBOSE
        cout << "--------------------------------------------------------------------" << endl;
        cout << "Image " << i << "  " << m_image_set[i] << " loaded" << endl;
        cout << "--------------------------------------------------------------------" << endl;
#endif
        nom_float = Mat::zeros(rgb_nom.size(), CV_32FC3);

        hsv_1 = Mat::zeros(rgb_nom.size(), CV_32FC3);
        rgb_nom.convertTo(nom_float, CV_32FC3);
        cvtColor(nom_float, hsv_1, COLOR_RGB2HSV);

        ch_1[0] = Mat::zeros(rgb_nom.size(), CV_32FC1);
        ch_1[1] = Mat::zeros(rgb_nom.size(), CV_32FC1);
        ch_1[2] = Mat::zeros(rgb_nom.size(), CV_32FC1);
        split(hsv_1, ch_1);
        gray_nom = ch_1[2];
        gray_nom.convertTo(gray_dist, CV_8UC1);
        beucherGradient(gray_nom, grad_nom);
        //--------------------------------------------------------------------------
        // End Conversion nominal image RGB -> Gray
        //--------------------------------------------------------------------------

        //--------------------------------------------------------------------------
        // Watershed nominal image
        //--------------------------------------------------------------------------
        // Marker table
        Mat *m_ptr_marker = new Mat(grad_nom.size(), CV_32SC1);
        m_ptr_marker->setTo(0);

        // Mask table
        mask = Mat::ones(grad_nom.size(), CV_8UC1);

        projection(grad_nom, grad_nom, mask);

        // Sets the labels to the marker table
        // Markers 1, 2 are assigned to non overlapping regions
        for (uint32_t r0 = 1; r0 < m_ptr_marker->rows - 1; r0++)
        {
            m_ptr_marker->at<int32_t>(r0, 0) = 1;
            m_ptr_marker->at<int32_t>(r0, m_ptr_marker->cols - 1) = 2;
        }

        for (uint32_t c0 = 1; c0 <  m_ptr_marker->cols - 1; c0++)
        {
            m_ptr_marker->at<int32_t>(m_ptr_marker->rows - 1, c0) = 1;
            m_ptr_marker->at<int32_t>(0, c0) = 2;
        }

#ifdef IS_VERBOSE
        cout << "Computing the optimal seam for the nominal image" << endl;
#endif
        m_ptr_marker->convertTo(*m_ptr_marker, CV_32SC1);
        WatershedMarkerSoille ws;
        ws.process(mask, *m_ptr_marker);

        m_ptr_marker->convertTo(*m_ptr_marker, CV_8U);

        imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/marker_after.png", mask);
        // dump_matrix_32_cv("marker", 0, *(img_compo.m_ptr_marker));

        //--------------------------------------------------------------------------
        // Makes the marker mask visible
        //--------------------------------------------------------------------------
        Mat rgb_mosaic = Mat::zeros(rgb_nom.size(), rgb_nom.type());

        patch_0 = Mat::zeros(m_ptr_marker->size(), m_ptr_marker->type());
        interseam = Mat::zeros(m_ptr_marker->size(), CV_8UC1);
        applyMaskToComposite(rgb_nom, rgb_nom, patch_0, rgb_mosaic);
        imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/test_patch_nominal", patch_0);

        //--------------------------------------------------------------------------
        // Loads the distorted image
        //--------------------------------------------------------------------------
        for (uint8_t j = 0; j < m_nbr_distortions; j++)
        {
            setDistortionProperties((Distortions)j);

#ifdef IS_VERBOSE
            cout << "------------------------------------------------------------------------------" << endl;
            cout << " Image [" << m_set_id << "] -  " << m_distort_name << endl;
            cout << "------------------------------------------------------------------------------" << endl;
#endif
            for (uint32_t k = 0; k < m_dist_man.m_distortion_step; k++)
            {
#ifdef IS_VERBOSE
                cout << "[" << m_set_id << "][" << k << "] - " << m_distort_name << endl;
#endif
                m_step_id = k;
                const string file_src_dist = setFileName2(m_distort_name, "Img_");
                rgb_dist = imread(file_src_dist, CV_LOAD_IMAGE_COLOR);

                if (rgb_dist.rows == 0 && rgb_dist.cols == 0)
                {
#ifdef IS_VERBOSE
                    cout << "The file: " << file_src_dist << " could not be opened !!" << endl;
#endif
                    break;
                }
                else if (rgb_nom.rows != rgb_dist.rows || rgb_nom.cols != rgb_dist.cols)
                {
#ifdef IS_VERBOSE
                    cout << "Size mismatch !!" << endl;
                    cout << "Nominal file: " << file_src_nom << " Size: (" << rgb_nom.rows << ", " << rgb_nom.cols << ")"  << endl;
                    cout << "Distort file: " << file_src_dist << " Size: (" << rgb_dist.rows << ", " << rgb_dist.cols << ")" << endl;
#endif
                    return false;
                }
                else
                {
#ifdef IS_VERBOSE
                    cout << "Processing file: " << file_src_dist << endl;
#endif
                }
#ifdef IS_VERBOSE
                cout << "-----------------------------------" << endl;
#endif

                //--------------------------------------------------------------------------
                // Begin Conversion the distorted image RGB -> Gray -> Beucher Gradient
                //--------------------------------------------------------------------------
                hsv_2 = Mat::zeros(rgb_dist.size(), CV_32FC3);
                dist_float = Mat::zeros(rgb_dist.size(), CV_32FC3);
                rgb_dist.convertTo(dist_float, CV_32FC3);
                cvtColor(dist_float, hsv_2, COLOR_RGB2HSV);

                ch_2[0] = Mat::zeros(rgb_dist.size(), CV_32FC1);
                ch_2[1] = Mat::zeros(rgb_dist.size(), CV_32FC1);
                ch_2[2] = Mat::zeros(rgb_dist.size(), CV_32FC1);
                split(hsv_2, ch_2);
                gray_dist = ch_2[2];
                gray_dist.convertTo(gray_dist, CV_8UC1);
                beucherGradient(gray_dist, grad_dist);
                //--------------------------------------------------------------------------
                // End Conversion the distorted image RGB -> Gray -> Beucher Gradient
                //--------------------------------------------------------------------------
                //--------------------------------------------------------------------------
                // Begin of Compositing algorithm
                //--------------------------------------------------------------------------

                // Marker table
                m_ptr_marker = new Mat(grad_nom.size(), CV_32SC1);
                m_ptr_marker->setTo(0);

                // Mask table
                mask = Mat::ones(grad_dist.size(), CV_8UC1);

                // Projection algorithm
#ifdef IS_VERBOSE
                cout << "Applying image Projection" << endl;
#endif
                projection(grad_nom, grad_dist, mask);

                for (uint32_t r0 = 1; r0 < m_ptr_marker->rows - 1; r0++)
                {
                    m_ptr_marker->at<int32_t>(r0, 0) = 1;
                    m_ptr_marker->at<int32_t>(r0, m_ptr_marker->cols - 1) = 2;
                }

                for (uint32_t c0 = 1; c0 <  m_ptr_marker->cols - 1; c0++)
                {
                    m_ptr_marker->at<int32_t>(m_ptr_marker->rows - 1, c0) = 1;
                    m_ptr_marker->at<int32_t>(0, c0) = 2;
                }

                //--------------------------------------------------------------------------
                // Watershed distorted images
                //--------------------------------------------------------------------------
#ifdef IS_VERBOSE
                cout << "Computing the optimal seam for the distorted image" << endl;
#endif
                m_ptr_marker->convertTo(*m_ptr_marker, CV_32SC1);
                WatershedMarkerSoille ws;
                ws.process(mask, *m_ptr_marker);

                m_ptr_marker->convertTo(*m_ptr_marker, CV_8U);

                patch_k = Mat::zeros(m_ptr_marker->size(), m_ptr_marker->type());
                applyMaskToComposite(rgb_nom, rgb_dist, patch_k, rgb_mosaic);
                imwrite(setFileName2(m_distort_name, "Patch_"), patch_k);

                // Test
                m_ptr_marker->convertTo(*m_ptr_marker, CV_32SC1);
                // Test

#ifdef IS_VERBOSE
                cout << "Writing mosaic image into file" << endl;
#endif
                imwrite(setFileName2(m_distort_name, "Mosaic_"), rgb_mosaic);

                //--------------------------------------------------------------------------
                // Computes the interseam
                //--------------------------------------------------------------------------
                interseam.setTo(0);

                uint32_t rows = patch_0.rows;
                uint32_t cols =  patch_0.cols;

                for (int r = 0; r < rows; r++)
                {
                    for (int c = 0; c < cols; c++)
                    {
                        if (patch_0.at<int8_t>(r, c) == patch_k.at<int8_t>(r, c))
                        {
                            interseam.at<uint8_t>(r, c) = 0;
                        }
                        else
                        {
                            interseam.at<uint8_t>(r, c) = 128;

                            // Updates interseam statistics

                            m_area_cube[i][j][k]++;
                        }
                    }

                    // mean_images[j] is the aggregation of the entire set of distostion steps k
                    // for the same distortion operation j
                    mean_images[j] += m_area_cube[m_set_id][m_distort_id][k] / m_dist_man.m_distortion_step;
                    //  mean_steps[j] += m_area_cube[m_set_id][m_distort_id][k] / m_dist_man.m_distortion_step;
                }
                // Rgi 08/04/2017
                // xl_sheet[j][k] += m_area_cube[i][j][k];
                // Rgi 08/04/2017

#ifdef IS_VERBOSE
                cout << "Interseam area: " << m_area_cube[m_set_id][m_distort_id][k] << endl;
                cout << "Writing Interseam image" << endl;
#endif
                imwrite(setFileName2(m_distort_name, "/interseam_").data(), interseam);
            }
        }
    }

    vector<float> val_max(m_nbr_distortions);
    vector<float> val_min(m_nbr_distortions);

    for (uint8_t j = 0; j < m_nbr_distortions; j++)
    {
        val_min[j] = 999999.0;
    }

    // Rgi 08/04/2017
    //    for (uint8_t j = 0; j < m_nbr_distortions; j++)
    //    {
    //        for (uint32_t k = 0; k < m_dist_man.m_distortion_step; k++)
    //        {
    //            if (xl_sheet[j][k] > val_max[j])
    //            {
    //                val_max[j] = xl_sheet[j][k];
    //            }

    //            if (xl_sheet[j][k] <= val_min[j])
    //            {
    //                val_min[j] = xl_sheet[j][k];
    //            }
    //        }
    //    }
    // Rgi 08/04/2017

    // Normalizes the cumulated interseam area upon image size

    for (uint8_t j = 0; j < m_nbr_distortions; j++)
    {
        float range = (val_max[j] - val_min[j]);

        for (uint32_t k = 0; k < m_dist_man.m_distortion_step; k++)
        {
            // Normalization upon operating range
            // Rgi 08/04/2017
            // xl_sheet[j][k] = (xl_sheet[j][k] - val_min[j]) / range;
            // Rgi 08/04/2017

            // Normalization upon image size
            // xl_sheet[j][k] = (xl_sheet[j][k] - val_min[j]) / (float)(rgb_nom.rows * rgb_nom.cols);
        }
    }

    xl_dict.m_data.push_back(xl_sheet);

    if (exportToFile(m_area_cube, mean_images) == false)
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------------------------------------

string FileExtractor(const string input)
{
    string::size_type idx;
    idx = input.rfind('.');

    if (idx != string::npos)
    {
        return input.substr(idx + 1);
    }
    else
    {
        return "none";
    }
}

//------------------------------------------------------------------------------------------------------------

void TestCompose::showFiles()
{
#ifdef IS_VERBOSE
    cout << "--------------------------------------------------------------------" << endl;
    cout << "Loaded files" << endl;
    cout << "--------------------------------------------------------------------" << endl;

    for (uint32_t i = 0; i < m_image_compose.m_image_set.size(); i++)
    {
        cout << "File : " << i << " " << m_image_compose.m_image_set[i] << endl;
    }
    cout << "--------------------------------------------------------------------" << endl;
#endif
}

//------------------------------------------------------------------------------------------------------------

#ifdef IS_WIN

int TestCompose::recursive(const string path, int max_img_set_size)
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
                    recursive(str, max_img_set_size);
                }
                int set_id = 0;

                if (m_image_compose.extractIdFromFileName(string(fd.name), set_id) == true)
                {
                    // Adds a new file to an existing set
                    m_image_compose.m_image_set.push_back(fd.name);

#ifdef IS_VERBOSE
                    cout << "Creating set " << set_id << " - " << fd.name << endl;
#endif
                }
            }
        }
        while (_findnext(hFile, &fd) == 0 && m_image_compose.m_image_set.size() < max_img_set_size);

#ifdef IS_VERBOSE
        cout << "-------------------------------------------------"  << endl;
        cout << m_image_compose.m_image_set.size() << " Images loaded" << endl;
        cout << "-------------------------------------------------"  << endl;
#endif
    }
    _findclose(hFile);  // close the file handle
    return m_image_compose.m_image_set.size();
}
#endif  // IS_WIN

#ifdef IS_LINUX
int TestCompose::recursive(const string path, int max_img_set_size)
{
   int id = 0;
   DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(path.c_str())) == NULL)
    {
#ifdef IS_VERBOSE
        cout << "Error(" << errno << ") opening " << path.c_str() << endl;
        return errno;
#endif
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        m_image_compose.m_image_set.push_back(string(dirp->d_name));
#ifdef IS_VERBOSE
        cout << "Creating set " << id++ << " - " << string(dirp->d_name) << endl;
#endif
    }
    closedir(dp);
    return 0;
}
#endif  // IS_LINUX

//------------------------------------------------------------------------------------------------------------

TestCompose::TestCompose()
{
}

//------------------------------------------------------------------------------------------------------------

TestCompose::TestCompose(const string path, const string sub_path, const uint32_t max_img_set_size, const uint32_t nbr_distortions, const uint32_t nbr_steps, const bool enable_generation)
{
    m_image_compose = ImageCompose(max_img_set_size, nbr_distortions, nbr_steps, path);
    m_image_compose.setDirOrder(SET_TO_OPERATION);

    int nbr_loaded_imgs = recursive(path + sub_path, max_img_set_size);

    if (nbr_loaded_imgs == 0)
    {
        cout << "No images found !!" << endl;
        exit(-1);
    }
    // showFiles();

    // If the number of loaded images is smaller the selected image set size,
    // the image set size to the the number of loaded imges
    // if (nbr_loaded_imgs < max_img_set_size)
    {
        //    img_set_size = nbr_loaded_imgs;
    }

    if (enable_generation == true)
    {
        if (m_image_compose.generateDistortedImageset(sub_path) == false)
        {
            cout << "Distortion generation failed !!" << endl;
            exit(-1);
        }
    }

    vector<Mat> gray_in;
    if (m_image_compose.compose(gray_in) == false)
    {
        cout << "Image Composition failed !! " << endl;
        exit(-1);
    }

    uint32_t cluster_size[16];
    memset(cluster_size, 0, sizeof(uint32_t) * 16);

    if (!m_image_compose.m_ptr_marker)
    {
        exit (-1);
    }
}

//------------------------------------------------------------------------------------------------------------

TestCompose::~TestCompose()
{
}
