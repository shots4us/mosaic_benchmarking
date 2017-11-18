#include "test.h"
#include "tools.h"
#include "lib_math.h"
#include "hsvmanager.h"
#include "objectivetest.h"
#include <math.h>
#include <stdint.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#ifdef IS_LINUX
#include <dirent.h>
#endif

// #include <QApplication>
#include <QCoreApplication>

//#include <opencv2\opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "watershed_soille.h"
#include "qm.h"
#include "image_compose.h"

#include "robustmatcher.h"
#include "stitching_detailed.h"
#include "blending_test.h"
#include "poisson_blender_new.h"
#include "geometry_metrics.h"
#include "stitch_graphcut.h"

int stitch_graphcut_ok(string path, Mat &stitch);

using namespace cv;
using namespace std;
using namespace qm;

int imageShift(const int x, const int y);
int ImageComposition();
bool testCanonicMatch(const Mat image_1, const Mat image_2, const int i, string path_dst);
bool testRobustMatch(const Mat image_1, const Mat image_2, const int i, string path_dst);
void testMatcher(const bool generate_images);

class Morpho
{
    
public:
    Morpho(const int n)
    {
        Mat channel[3];
        for (int i = 0; i < 1; i++)
        {
            QByteArray fileName("D:/PRJ/LITTORALG/Docs/Morphological image compositing/img");
            fileName += QByteArray::number(i);
            fileName += ".png";
            
            Mat rgb(imread(fileName.data(), CV_LOAD_IMAGE_COLOR));
            Mat hsv = Mat::zeros(rgb.rows, rgb.cols, CV_32FC3);
            varKnuth = Mat::zeros(rgb.rows, rgb.cols,CV_32FC1);
            channel[0] = Mat::zeros(rgb.rows, rgb.cols, CV_8UC1);
            channel[1] = Mat::zeros(rgb.rows, rgb.cols, CV_8UC1);
            channel[2] = Mat::zeros(rgb.rows, rgb.cols, CV_8UC1);
            
            cvtColor(rgb, hsv, COLOR_RGB2HSV);
            cv::split(hsv, channel);
            channel[0].convertTo(channel[0], CV_32FC1);
            channel[1].convertTo(channel[1], CV_32FC1);
            channel[2].convertTo(channel[2], CV_32FC1);
            imgIn.append(channel[2]);
            
            // imshow("Hue", channel[0]);
            // imshow("Saturation", channel[1]);
            // imshow("Value", chchannel2]);
            // waitKey(0);
        }
    }
    
    void ImageComposition();
    
    // protected:
    QList<Mat> imgIn;
    Mat mean;
    Mat varKnuth;
};

Mat shiftFrame(Mat frame, const int x, const int y);
Mat shiftFrame(Mat frame, const int x, const int y)
{
    //create a same sized temporary Mat with all the pixels flagged as invalid (-1)
    Mat temp = Mat::zeros(frame.size(), frame.type());
    temp.setTo(cv::Scalar(128,0,0));
    
    frame(cv::Rect(x, y, frame.cols - x, frame.rows - y)).copyTo(temp(cv::Rect(0, 0, temp.cols - x, temp.rows - y)));
    return temp;
}

void Morpho::ImageComposition()
{
}

bool createImageCompose(Mat image[], Point image_offset[128], const uint n, Mat &out);
bool createImageCompose(Mat image[], Point image_offset[128], const uint n, Mat &out)
{
    return true;
}


//------------------------------------------------------------------------------------------------------------

Mat1f exposureTonemap (Mat1f m, float gamma = 2.2, float exposure = 1) {
    // Exposure tone mapping
    Mat1f exp;
    cv::exp( (-m) * exposure, exp );
    Mat1f mapped = 1.0f - exp;
    
    // Gamma correction
    cv::pow(mapped, 1.0f / gamma, mapped);
    
    return mapped;
}

//------------------------------------------------------------------------------------------------------------

Mat3f hsvExposureTonemap(Mat &a) {
    Mat3f hsvComb;
    cvtColor(a, hsvComb, COLOR_RGB2HSV);
    
    Mat1f hsv[3];
    split(hsvComb, hsv);
    
    hsv[2] = exposureTonemap(hsv[2], 2.2, 10);
    
    merge(hsv, 3, hsvComb);
    
    Mat rgb;
    cvtColor(hsvComb, rgb, COLOR_HSV2RGB);
    
    return rgb;
}

#ifdef IS_WIN
int scanDir(const string path, vector <string> &vct_files);
int scanDir(const string path, vector <string> &vct_files)
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
                    scanDir(str, vct_files);
                }
                
                // Adds a new file to an existing set
                vct_files.push_back(fd.name);
#ifdef IS_VERBOSE
                cout << "Creating set " << vct_files.size() - 1 << " - " << fd.name << endl;
#endif
            }
        }
        while (_findnext(hFile, &fd) == 0);
        
#ifdef IS_VERBOSE
        cout << "-------------------------------------------------"  << endl;
        cout << vct_files.size() << " Images loaded" << endl;
        cout << "-------------------------------------------------"  << endl;
#endif
    }
    _findclose(hFile);  // close the file handle
    return vct_files.size();
}
#endif

//-------------------------------------------------------------------
// main
//-------------------------------------------------------------------
#include <string>
#ifdef IS_WIN
#include <windows.h>
#endif

int main(int argc, char *argv[])
{
    std::string argv_str(argv[0]);
    std::string base = argv_str.substr(0, argv_str.find_last_of("/"));
    cout << endl;
    cout << "- Executable directory: " << base << endl;
    
    if (argc == 1)
    {
        cout << endl;
        cout << "ImageCompositing <Benchmark type><Image-set size><Number of Distortions><Steps/Distortion><Image size>" << endl;
        //  exit(0);
    }
    
    string path;
    
#ifdef IS_WIN
    path = "D:/PRJ/Benchmark/OBJECTIVE_TEST/";
#endif
    
#ifdef IS_LINUX
    path = "/data/robyreisen/PRJ/OBJECTIVE_TEST/";
#endif
    
    string src_path = path;
    string dst_path = path;
    
    cout << "- Data Working directory: " << path << endl;
    cout << "--------------------------------------------------------" << endl;

    //----------------------------------------------------------------
    // 1. Test Stitching OpenCv nominal with Warping
    //----------------------------------------------------------------
    //StitchGraphcut::stitch_opencv(path);
    
    //---------------------------------------------
    // 2. Test Blending
    //---------------------------------------------
    // unit_test();
    // blending_test();
    // exit(0);
    
    //---------------------------------------------
    // 3. Test Poisson
    //---------------------------------------------
    // unit_test();
    // exit(0);
    
    //---------------------------------------------
    // 4. Test Blending
    //---------------------------------------------
    // StitchGraphcut::unit_test_plan(path);
    // exit(0);
    
    //---------------------------------------------
    // 5. Test GeometryMetrics
    //---------------------------------------------
    
    //    const string path = "D:/PRJ/Benchmark/OBJECTIVE_TEST/geometry_metrics/";
    //    GeometryMetrics *geo_metr = new GeometryMetrics(path);
    
    //    //-----------------------------------------------------------
    //    // 1. Unit Precision/Reacheability manually driven algorithm
    //    //-----------------------------------------------------------
    //    // geo_metr->unit_test();
    
    //    //-----------------------------------------------------------
    //    // 2. Automated parameterless algorithm
    //    //-----------------------------------------------------------
    //    // Read first image
    //    const Mat rgb_src_1 = imread(path + string("p0.png"));
    //    // Read Second Image
    //    const Mat rgb_src_2 = imread(path + string("p1_58.png"));
    
    //    // Read mosaic Image 1
    //    const Mat rgb_src_mosaic_1 = imread(path + string("mosaic_nominal.png"));
    
    //    // Read mosaic Image 2
    //    const Mat rgb_src_mosaic_2 = imread(path + string("mosaic_distort_64_64.png"));
    
    //   geo_metr->m_vct_src_shift.push_back(Point(58, 0)); // 24, 24)); //58, 0));
    
    //   geo_metr->optimal_test(rgb_src_1, rgb_src_2, rgb_src_mosaic_1, rgb_src_mosaic_2);
    
    //    exit(0);
    
    //---------------------------------------------
    // 6. Test Coovariance Matrix
    //---------------------------------------------
    //    if (UnitTestCreateCovarianceMatrix() == true)
    //    {
    //        cout << "Unit Test Create Covariance Matrix Ok !!" << endl;
    //    }
    
    //---------------------------------------------
    // End Test section
    //---------------------------------------------
    
    //--------------------------------------------------------------------------
    // 1. Enable this option to create a new set of min_img_size x min_img_size icones
    // This function is used to fill up the NormalSub directory
    //--------------------------------------------------------------------------
    bool generate_icons = false;
    
    //--------------------------------------------------------------------------
    // 2. Enable this option every time the nominal and distorted dataset must be
    // generated from the min_img_size x min_img_size icons
    // This function generates Image_xxxx directories
    //--------------------------------------------------------------------------
    bool generate_distort_dataset = false;
    
    // 3. Generate Target Files
    bool generate_target_files = true;
    
    // 4. Max size of the image-set
    uint32_t max_img_set_size = 1000;
    
    // 4. Max size of the image-set per dimension
    uint32_t nbr_of_shifts = 1;
    
    // 5. Total range of the degrading tranform applied to the selected image
    const int distortion_range = 5;
    
    // 6. Number of distortion tranforms to apply to the image sets
    int nbr_distortions = DISTORTION_COUNT;
    
    // 7. Number of steps generated for a selected distortion transform
    // That correpsonds to the actual number of files that will be generated for each distortion
    int nbr_steps = 10;
    
    // 8. Min image size
    int min_img_size = 1800;
    
    // 9. Benchmark Pipeline Selection
    BenchmarkType benchmark_pipeline = StitchingPipeline; //GeometryFidelity;
    
    // 10. Minimum number of good features to find to accept the Feature Matching
    const int min_match_array_size = 10;
    
    //-----------------------------------------------------------------------
    // Command-line format
    // 1. <Generate Icons = I / Generate Distort Dataset = D / Execute Benchmark = X>
    // 2. <Benchmark type: S = Stitching, M = Mosaicing, G = Geometry fidelity>
    // 3. <Max Input Image-set size>
    // 4. <nbr_of_shifts>
    // 6. <Number of Distortions [1..13]>
    // 7. <Steps/Distortion>
    // 8. <Icon size>
    // 9. <Generate Target files = T>
    // 10. <Distort Src Path>
    // 11. <Target Mosaic Image Path>
    //-----------------------------------------------------------------------

    // Param 1
    if (argv[1])
    {
        if (argv[1][0] == 'h' || argv[1][0] == 'H' || argv[1][0] == '?')
        {
            //-----------------------------------------------------------------------
            // Command-line format
            cout <<  "1. <Generate Icons = I / Generate Distort Dataset = D / Execute Benchmark = X>" << endl;
            cout <<  "2. <Benchmark type: S = Stitching, M = Mosaicing, G = Geometry fidelity>" << endl;
            cout <<  "3. <Max Input Image-set size>" << endl;
            cout <<  "4. <Nbr of Shifts>" << endl;
            cout <<  "5. <Number of Distortions [1..13]>" << endl;
            cout <<  "6. <Steps/Distortion>" << endl;
            cout <<  "7. <Icon size>" << endl;
            cout <<  "8. <Generate Target files = T>" << endl;
            cout <<  "9. <Distort Src Path>" << endl;
            cout <<  "10. <Target Mosaic Image Path>" << endl;
            //-----------------------------------------------------------------------
            exit(0);
        }

        int cnt = 1;

        string oper_name = "";
        // Param
        if (argc > 2)
        {
            if (argv[cnt][0] == 'i' || argv[cnt][0] == 'I')
            {
                generate_icons = false;
                oper_name = "Generate Icons";
            }
            else if (argv[cnt][0] == 'd' || argv[cnt][0] == 'D')
            {
                generate_distort_dataset = true;
                oper_name = "Generate Distort Dataset";
            }
            else if (argv[cnt][0] == 'x' || argv[cnt][0] == 'X')
            {
                generate_icons = false;
                generate_distort_dataset = false;
                oper_name = "Execute Becnhmark";
            }
        }
        cout << cnt << ". Operation: " << argv[cnt][0] << " ->  " << oper_name << endl;

        string pipeline_name = "";
        cnt++;

        if (argv[cnt][0] == 's' || argv[cnt][0] == 'S')
        {
            benchmark_pipeline = StitchingPipeline;
            pipeline_name = "Stitching";
        }
        else if (argv[cnt][0] == 'm' || argv[cnt][0] == 'M')
        {
            benchmark_pipeline = MosaicingPipeline;
            pipeline_name = "Mosaicing";
        }
        else if (argv[cnt][0] == 'g' || argv[cnt][0] == 'G')
        {
            benchmark_pipeline = GeometryFidelity;
            pipeline_name = "Geometry Fidelity";
        }

        cout << cnt << ". Benchmark Pipeline: " << argv[cnt][0] << " -> " << pipeline_name << endl;
        
        // Param 2
        cnt++;
        if (argc > cnt && argv[cnt] != "")
        {
            max_img_set_size = atoi(argv[cnt]);
        }
        cout << cnt << ". Max Imageset size: " << max_img_set_size << endl;
        
        // Param 3
        cnt++;
        if (argc > cnt && argv[cnt] != "")
        {
            nbr_of_shifts = atoi(argv[cnt]);
        }
        cout << cnt << ". Number of Shitfs: " << nbr_of_shifts << endl;
        
        // Param 4
        cnt++;
        if (argc > cnt && argv[cnt] != "")
        {
            nbr_distortions = atoi(argv[cnt]);
        }
        cout << cnt << ". Number of Distortions: " << nbr_distortions << endl;
        
        // Param 5
        cnt++;
        if (argc > cnt)
        {
            if (argv[cnt] != "")
            {
                nbr_steps = atoi(argv[cnt]);
            }
        }
        cout << cnt << ". Number of Steps: " << nbr_steps << endl;

        // Param 6
        cnt++;
        if (argc > cnt)
        {
            if (argv[cnt] != "")
            {
                min_img_size = atoi(argv[cnt]);
            }
        }
        cout << cnt << ". Image size: " << min_img_size << endl;
        
        // Param 8
        cnt++;
        if (argc > 8)
        {
            if (argv[cnt][0] == 't' || argv[cnt][0] == 'T')
            {
                generate_target_files = true;
                
            }
            else if (argv[cnt][0] == 'n' || argv[cnt][0] == 'N')
            {
                generate_target_files = false;
            }
            if (generate_icons == true)
            {
                cout << cnt << ". Generate Icons: " << generate_icons << endl;
            }

            if (generate_distort_dataset == true)
            {
                cout << cnt << ". Generate Distort Dataset: " << generate_distort_dataset << endl;
            }

            if (generate_target_files == true)
            {
                cout << cnt << ". Generate Target Files: True" << endl;
            }
            else
            {
                cout << cnt << ". Generate Target Files: False" << endl;
            }

            // Param 9: Distort Src Path
            cnt++;
            if (argc > cnt && string(argv[cnt]) != "")
            {
                src_path = path + string(argv[cnt]);
            }
            else
            {
                if (generate_icons == true)
                {
                    src_path = path + "Nominal/";
                }
                else if (generate_distort_dataset == true)
                {
                    src_path = path + "NominalSub/";
                }
                else
                {
                    src_path = path;
                }
            }
            if (src_path[src_path.size() - 1] != '/')
            {
                src_path += '/';
            }

            cout << cnt << ". Src Path:" << src_path << endl;

            // Param 10: Target Path
            cnt++;
            if (argc > cnt && string(argv[cnt]) != "")
            {
                dst_path = path + string(argv[cnt]);
            }
            else
            {
                if (generate_icons == true)
                {
                    dst_path = path + "NominalSub/";
                }
                else
                {
                    dst_path = path;
                }
            }
            if (dst_path[dst_path.size() - 1] != '/')
            {
                dst_path += '/';

            } cout << cnt << ". Dst Path:" << dst_path << endl;
        }
    }
    cout << "--------------------------------------------------------" << endl;

    //-------------------------------------------------------------------------
    // Reads the contents of the source directory
    //-------------------------------------------------------------------------

    if (generate_icons == true)
    {
        Mat rgb_src;
        Mat rgb_dst;
        
        char val[255];
        memset(val, 0, 64);
        
        //--------------------------------------------------------------------------------
        // Reads the content of the input folder and loads it to a vector of file names
        //--------------------------------------------------------------------------------
        vector <string> vct_files;
        
#ifdef IS_WIN
        src_path = path + "DATASETS/coastline/";
        dst_path = path + "DATASETS/icon_coastline_2000/";
        scanDir(src_path, vct_files);
#endif
        
#ifdef IS_LINUX
        struct dirent **namelist;
        int i,n;

        n = scandir(src_path.c_str(), &namelist, 0, alphasort);
        if (n < 0)
            perror("scandir");
        else
        {
            for (i = 0; i < n; i++)
            {
                if (namelist[i]->d_name != string(".") && namelist[i]->d_name != string(".."))
                {
                    printf("%s\n", namelist[i]->d_name);
                    vct_files.push_back(namelist[i]->d_name);
                    free(namelist[i]);
                }
            }
        }
        free(namelist);
#endif

        // Icon generation parameters
        const int new_size = 2000;
        int cnt_ok = 0;

        for (int i = 0; i < vct_files.size(); i++)
        {
            cout << to_string(i) << "/" << to_string(vct_files.size()) << " - Processing file: " << vct_files[i] << endl;

            sprintf(val, "%04d", i);
            
            string file_in = src_path;
            file_in += vct_files[i];
            
            string file_out = dst_path;
            file_out += "Img_";
            file_out += val;
            file_out += ".png";
            
            cout << file_in << endl;

            rgb_src = imread(file_in.data(), CV_LOAD_IMAGE_COLOR);

            if (rgb_src.rows == 0 || rgb_src.cols == 0)
            {
                cout << "Image: " << i << " missing !!" << endl;
                exit(-1);
                //break;
            }

            cout << "Extracting sub image from: " << i << " image" << endl;

            const float ratio = (float)min(rgb_src.rows, rgb_src.cols) / max(rgb_src.cols, rgb_src.rows);

            if (rgb_src.cols > 0 && rgb_src.rows > 0 &&
                    max(rgb_src.rows, rgb_src.cols) >= new_size && min(rgb_src.rows, rgb_src.cols) >= (new_size * ratio))
            {
                if (rgb_src.cols > rgb_src.rows)
                {
                    resize(rgb_src, rgb_dst, Size(new_size / ratio, new_size), 0, 0, INTER_CUBIC);
                }
                else
                {
                    resize(rgb_src, rgb_dst, Size(new_size, new_size / ratio), 0, 0, INTER_CUBIC);
                }
                
                // imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/DATASETS/Temp/" + to_string(new_size) + "_" + to_string(i) + ".png", rgb_dst);
            }
            else
            {
                cout << "Error Size mismatch, aborting !!" << endl;
                exit(-1);
            }
            // }
            Mat rgb_dst_2;

            // Extracts the ROI from the image center
            if (selectCenterRegion(rgb_dst, rgb_dst_2, min_img_size) == true)
            {
                if (imwrite(file_out.data(), rgb_dst_2) == true)
                {
                    cout << cnt_ok << "/" << vct_files.size() << " files successfully iconized !!" << endl;
                    cnt_ok++;
                }
            }
            else
            {
                cout << "Failed to select ROI, aborting !!" << endl;
                exit(-1);
            }
            //  }
        }
        exit(0);
    }
    
    const int select_test = 0;
    
    if (select_test == 0)
    {
        cout << "-------------------------------------------------------------------" << endl;
        cout << "Welcome to the ImageCompositing benchmark  V 1.1.0" << endl;
        cout << "-------------------------------------------------------------------" << endl;
        cout << endl;
        cout << "-------------------------------------------------------------------" << endl;
        cout << "1. OBJECTIVE MOSAICING QUALITY BENCHMARKS" << endl;
        cout << "-------------------------------------------------------------------" << endl;
        // const string src_path = "roi_";

        // The nbr_of_shifts = 1 corresponds to no shift
        nbr_of_shifts++;

        ObjectiveTest("D:/PRJ/Benchmark/OBJECTIVE_TEST/DATASETS/",
                      "D:/PRJ/Benchmark/OBJECTIVE_TEST/DATASETS/distort_coastline_2000/",
                      "D:/PRJ/Benchmark/OBJECTIVE_TEST/DATASETS/stitch_coastline_2000/",
                      max_img_set_size,
                      nbr_of_shifts,
                      nbr_distortions,
                      distortion_range,
                      nbr_steps,
                      min_img_size,
                      benchmark_pipeline,
                      generate_distort_dataset,
                      generate_target_files,
                      min_match_array_size,
                      UseInternalAlgorithm,
                      Graphcut);
    }
    //    else if (select_test == 1)
    //    {
    //        cout << endl;
    //        cout << "-------------------------------------------------------------------" << endl;
    //        cout << "2. OBJECTIVE STITCHING SEAM QUALITY BENCHMARK" << endl;
    //        cout << "-------------------------------------------------------------------" << endl;
    
    //        // Solution using Intersem metrics
    //        TestCompose test_compose(src_path, src_path, max_img_set_size, nbr_distortions, nbr_steps, generate_distort_dataset);
    //    }
    
    //    //    TestCompose test_compose_nominal("D:/Benchmark/img_n_", cluster_ratio_nominal);
    //    //    TestCompose test_compose_degraded("D:/Benchmark/img_n_", cluster_ratio_degraded);
    
    //    //    int inter_area = interseam_area(testzcompose.m_ptr_marker, test_compose_degraded.m_image_compose.m_ptr_marker);
    
    //    //    Mat m1, m2;
    //    //    test_compose_nominal.m_image_compose.m_ptr_marker->convertTo(m1, CV_64F);
    //    //    test_compose_degraded.m_image_compose.m_ptr_marker->convertTo(m2, CV_64F);
    
    //    //    double psnr_val = psnr(m1, m2, 64); //*test_compose_nominal.m_image_compose.m_ptr_marker, *test_compose_degraded.m_image_compose.m_ptr_marker, 64);
    //    //    cout << "PSNR: " << psnr_val << endl;
    
    //    //    // Human vision based similarity quality metric
    //    //    // SSIM is block based similarity metric
    //    //    double ssim_val = ssim(m1, m1, 64, false); //*test_compose_nominal.m_image_compose.m_ptr_marker, *test_compose_degraded.m_image_compose.m_ptr_marker, 64, false);
    //    //    cout << "SSIM: " << ssim_val << endl;
    return 0;
}

//-----------------------------------------------------------------------------------
// TEST SIFT
//------------------------------------------------------------------------------------

const double THRESHOLD = 400;

/**
      * Calculate euclid distance
      */
double euclidDistance(Mat& vec1, Mat& vec2) {
    double sum = 0.0;
    int dim = vec1.cols;
    for (int i = 0; i < dim; i++) {
        sum += (vec1.at<uchar>(0,i) - vec2.at<uchar>(0,i)) * (vec1.at<uchar>(0,i) - vec2.at<uchar>(0,i));
    }
    return sqrt(sum);
}

/**
      * Find the index of nearest neighbor point from keypoints.
      */
int nearestNeighbor(Mat& vec, vector<KeyPoint>& keypoints, Mat& descriptors) {
    int neighbor = -1;
    double minDist = 1e6;
    
    for (int i = 0; i < descriptors.rows; i++) {
        KeyPoint pt = keypoints[i];
        Mat v = descriptors.row(i);
        double d = euclidDistance(vec, v);
        //printf("%d %f\n", v.cols, d);
        if (d < minDist) {
            minDist = d;
            neighbor = i;
        }
    }
    
    if (minDist < THRESHOLD) {
        return neighbor;
    }
    
    return -1;
}

/**
      * Find pairs of points with the smallest distace between them
      */
void findPairs(vector<KeyPoint>& keypoints1, Mat& descriptors1,
               vector<KeyPoint>& keypoints2, Mat& descriptors2,
               vector<Point2f>& srcPoints, vector<Point2f>& dstPoints) {
    for (int i = 0; i < descriptors1.rows; i++) {
        KeyPoint pt1 = keypoints1[i];
        Mat desc1 = descriptors1.row(i);
        int nn = nearestNeighbor(desc1, keypoints2, descriptors2);
        if (nn >= 0) {
            KeyPoint pt2 = keypoints2[nn];
            srcPoints.push_back(pt1.pt);
            dstPoints.push_back(pt2.pt);
        }
    }
}

//------------------------------------------------------------------------------
//  testRobustMatch
//------------------------------------------------------------------------------
//bool testCanonicMatch(const Mat image_1, const Mat image_2, const int i, string path_dst)
//{
//    cout << endl;
//    cout << "//----------------------------------------------------------------------------" << endl;
//    cout << "/ /1. Start Test Canonic Match" << endl;
//    cout << "//----------------------------------------------------------------------------" << endl;

//    cv::SiftFeatureDetector *detector;
//    //cv::SiftFeatureDetector
//    detector = new SiftFeatureDetector(
//                0, // nFeatures
//                4, // nOctaveLayers
//                0.04, // contrastThreshold
//                10, //edgeThreshold
//                1.6 //sigma
//                );
//    std::vector<cv::KeyPoint> keypoints1;
//    detector->detect(image_1, keypoints1);
//    // Add results to image and save.
//    cv::Mat output1;
//    cv::drawKeypoints(image_1, keypoints1, output1);
//    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/keypoints1.png", output1);

//    //keypoints array for input 2
//    std::vector<cv::KeyPoint> keypoints2;
//    //output array for ouput 2
//    cv::Mat output2;

//    //-- Step 1: Detect the keypoints using SIFT
//    cv::SiftDescriptorExtractor extractor;
//    cv::Mat descriptors1,descriptors2;
//    cv::vector<cv::DMatch> matches;
//    detector->detect(image_2,keypoints2);

//    //-- Step 1: Detect the keypoints using SURF Detector
//    // int minHessian = 100;
//    // SurfFeatureDetector detector( minHessian );
//    // std::vector< KeyPoint > keypoints1, keypoints2;
//    // detector.detect( image_1, keypoints1 );
//    // detector.detect( image_2, keypoints2 );

//    cv::drawKeypoints(image_2,keypoints2,output2);
//    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/keypoints2.png", output2);

//    //-- Step 2: Calculate descriptors (feature vectors)

//    extractor.compute(image_1,keypoints1,descriptors1);
//    extractor.compute(image_2,keypoints2,descriptors2);

//    FlannBasedMatcher matcher;
//    matcher.match( descriptors1, descriptors2, matches );

//    //    SurfDescriptorExtractor extractor;
//    //    Mat descriptors1, descriptors_scene;
//    //    extractor.compute( image_1, keypoints1, descriptors1 );
//    //    extractor.compute( image_2, keypoints2, descriptors_scene );

//    //-- Step 3: Matching descriptor vectors using FLANN matcher
//    //    FlannBasedMatcher matcher;
//    //    std::vector< DMatch > matches;
//    //    matcher.match( descriptors1, descriptors_scene, matches );

//    double max_dist = 0; double min_dist = 100;

//    //-- Quick calculation of max and min distances between keypoints
//    for( int i = 0; i < descriptors1.rows; i++ )
//    {
//        double dist = matches[i].distance;
//        if( dist < min_dist ) min_dist = dist;
//        if( dist > max_dist ) max_dist = dist;
//    }

//    printf("-- Max dist : %f \n", max_dist );
//    printf("-- Min dist : %f \n", min_dist );

//    //-- Use only "good" matches (i.e. whose distance is less than 3*min_dist )
//    std::vector< DMatch > good_matches;

//    for( int i = 0; i < descriptors1.rows; i++ )
//    {
//        if( matches[i].distance < 3*min_dist )
//        {
//            good_matches.push_back( matches[i]);
//        }
//    }

//    if (good_matches.size() == 0)
//    {
//        cout << "No matches found !!" << endl;
//        cout << "//----------------------------------------------------------------------------" << endl;
//        cout << "// Start Test Canonic Match" << endl;
//        cout << "//----------------------------------------------------------------------------" << endl;
//        return false;
//    }

//    std::vector< Point2f > obj;
//    std::vector< Point2f > scene;

//    for( int i = 0; i < good_matches.size(); i++ )
//    {
//        //-- Get the keypoints from the good matches
//        obj.push_back( keypoints1[ good_matches[i].queryIdx ].pt );
//        scene.push_back( keypoints2[ good_matches[i].trainIdx ].pt );
//    }

//    // Find the Homography Matrix
//    Mat H = findHomography( obj, scene, CV_RANSAC);
//    // Use the Homography Matrix to warp the images
//    Mat img_warp;
//    warpPerspective(image_2,img_warp,H, Size(800, 800));

//    string f_name = path_dst + "result/canonic_warp_";
//    char val_i[128];
//    memset(val_i, 0, 128);
//    sprintf(val_i, "%d", i);
//    string str_1 = string(val_i);
//    f_name += str_1;
//    f_name += ".png";
//    // imwrite(f_name, img_warp);

//    //imshow("WARP", result);
//    // cv::Mat half(result,cv::Rect(0,0,image_2.cols,image_2.rows));
//    // image_2.copyTo(half);

//    Mat img_outline;
//    // drawKeypoints(image_1,keypoints2,key,Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//    // drawMatches(image_2, keypoints2, image_1, keypoints1, matches, img_outline);
//    // imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/test_match_canonic_matches.png", img_outline);
//    cout << "End Test Canonic Matching" << endl;

//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    std::vector<Point2f> obj_corners(4);
//    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( image_1.cols, 0 );
//    obj_corners[2] = cvPoint( image_1.cols, image_1.rows ); obj_corners[3] = cvPoint( 0, image_1.rows );
//    std::vector<Point2f> scene_corners(4);

//    perspectiveTransform( obj_corners, scene_corners, H);

//    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//    line( img_outline, scene_corners[0] + Point2f( image_1.cols, 0), scene_corners[1] + Point2f( image_1.cols, 0), Scalar(0, 255, 0), 4 );
//    line( img_outline, scene_corners[1] + Point2f( image_1.cols, 0), scene_corners[2] + Point2f( image_1.cols, 0), Scalar( 0, 255, 0), 4 );
//    line( img_outline, scene_corners[2] + Point2f( image_1.cols, 0), scene_corners[3] + Point2f( image_1.cols, 0), Scalar( 0, 255, 0), 4 );
//    line( img_outline, scene_corners[3] + Point2f( image_1.cols, 0), scene_corners[0] + Point2f( image_1.cols, 0), Scalar( 0, 255, 0), 4 );

//    f_name = path_dst + "result/canonic_outline_";
//    val_i[128];
//    memset(val_i, 0, 128);
//    sprintf(val_i, "%d", i);
//    str_1 = string(val_i);
//    f_name += str_1;
//    f_name += ".png";
//    imwrite(f_name, img_outline);

//    cout << "----------------------------------------------------------------------------" << endl;
//    cout << "End Test Canonic Match" << endl;
//    cout << "----------------------------------------------------------------------------" << endl;
//    return true;
//}

//------------------------------------------------------------------------------
//  testRobustMatch
//------------------------------------------------------------------------------
//bool testRobustMatch(const Mat image_1, const Mat image_2, const int i, string path_dst)
//{
//    cout << endl;
//    cout << "//----------------------------------------------------------------------------" << endl;
//    cout << "// 2. Start Test Robust Match: " << i << endl;
//    cout << "//----------------------------------------------------------------------------" << endl;

//    std::vector<cv::KeyPoint> keypoint_img_1;
//    std::vector<cv::KeyPoint> keypoint_img_2;

//    RobustMatcher *robust_matcher = new RobustMatcher();
//    Mat H = robust_matcher->match(image_1, image_2, &keypoint_img_1, &keypoint_img_2);
//    robust_matcher->show(image_1, image_2, keypoint_img_1, keypoint_img_2,"D:/PRJ/Benchmark/OBJECTIVE_TEST/image_matches.png");

//    Mat img_warp;
//    warpPerspective(image_2,img_warp,H, Size(1000, 1000));
//    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/i1.png", image_1);
//    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/i2.png", image_2);
//    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/test_robust_warped.png", img_warp);

//    // Mat img_outline;
//    // drawMatches(image_1, keypoint_img_1,image_2, keypoint_img_1,
//    //              robust_matcher->matchArray, img_outline, Scalar::all(-1), Scalar::all(-1),
//    //              vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//    std::cout << "Number of good matching " << (int)robust_matcher->matchArray.size() << "\n" << endl;

//    if ((int)robust_matcher->matchArray.size() == 0 )
//    {
//        cout << "No matches found !!" << endl;
//        return false;
//    }

//    if ((int)robust_matcher->matchArray.size() > 5 )
//    {
//        cout << "Good matching !!" << endl;
//    }

//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    std::vector<Point2f> obj_corners(4);
//    obj_corners[0] = cvPoint(0,0);

//    obj_corners[1] = cvPoint(image_2.cols, 0);
//    obj_corners[2] = cvPoint(image_2.cols, image_2.rows);
//    obj_corners[3] = cvPoint(0, image_2.rows);
//    std::vector<Point2f> scene_corners(4);

//    // Some bundle adjustment may be necessary to accomodate rotations
//    perspectiveTransform(obj_corners, scene_corners, H);

//    Mat img_outline = Mat::zeros(1000, 1000, CV_8UC3); // mage_1.cols + image_2.cols, image_1.rows + image_1.rows, CV_8UC3);

//    const int scene_height = round(scene_corners[2].y) - round(scene_corners[0].y);
//    const int scene_width = round(scene_corners[1].x) - round(scene_corners[0].x);

//    const int x0 = round(scene_corners[0].x);
//    const int y0 = round(scene_corners[0].y);
//    Mat warp_crop = img_warp(Rect(x0, y0, scene_width, scene_height));

//    string f_name = path_dst + "result/warp_crop_";
//    char val_i[128];
//    memset(val_i, 0, 128);
//    sprintf(val_i, "%d", i);
//    string str_1 = string(val_i);
//    f_name += str_1;
//    f_name += ".png";
//    imwrite(f_name, warp_crop);

//    // Embeds the image 2
//    // The warped image is shifted to the origin
//    for( int r = 0; r < warp_crop.rows; r++ )
//    {
//        for( int c = 0; c < warp_crop.cols; c++ )
//        {
//            img_outline.at<Vec3b>(r, c)[0] = warp_crop.at<Vec3b>(r, c)[0];
//            img_outline.at<Vec3b>(r, c)[1] = warp_crop.at<Vec3b>(r, c)[1];
//            img_outline.at<Vec3b>(r, c)[2] = warp_crop.at<Vec3b>(r, c)[2];
//        }
//    }

//    // Embeds the image 1
//    // The image 1 is shifted to fit the warped image 2
//    for( int r = 0; r < image_1.rows; r++ )
//    {
//        for( int c = 0; c < image_1.cols; c++ )
//        {
//            const int r0 = round(scene_corners[0].y);
//            const int c0 = round(scene_corners[0].x);

//            img_outline.at<Vec3b>(r + r0, c + c0)[0] = image_1.at<Vec3b>(r, c)[0];
//            img_outline.at<Vec3b>(r + r0, c + c0)[1] = image_1.at<Vec3b>(r, c)[1];
//            img_outline.at<Vec3b>(r + r0, c + c0)[2] = image_1.at<Vec3b>(r, c)[2];
//        }
//    }

//    const int ox = round(obj_corners[2].x) - round(obj_corners[0].x);
//    const int oy = round(obj_corners[2].y) - round(obj_corners[0].y);

//    const int sx = image_1.cols + round(scene_corners[0].x);
//    const int sy = image_1.rows + round(scene_corners[0].y);

//    const int mx = max(ox, sx);
//    const int my = max(oy, sy);

//    const Point corner_min(0, 0);
//    const Point corner_max(mx, my);

//    Mat outer_canvas = img_outline(Rect(corner_min.x, corner_min.y, corner_max.x - corner_min.x, corner_max.y - corner_min.y));

//    f_name = path_dst + "result/robust_mosaic_";
//    val_i[128];
//    memset(val_i, 0, 128);
//    sprintf(val_i, "%d", i);
//    str_1 = string(val_i);
//    f_name += str_1;
//    f_name += ".png";
//    imwrite(f_name, outer_canvas);

//    const bool generate_outline = true;

//    if (generate_outline == true)
//    {
//        std::vector<Point2f> obj_corners2(4);
//        obj_corners2[0] = cvPoint( scene_corners[0].x,  scene_corners[0].y);
//        obj_corners2[1] = cvPoint( scene_corners[0].x, scene_corners[0].y + image_1.rows);
//        obj_corners2[2] = cvPoint(scene_corners[0].x + image_1.cols, scene_corners[0].y + image_1.rows);
//        obj_corners2[3] = cvPoint(scene_corners[0].x + image_1.cols, scene_corners[0].y);

//        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//        line( img_outline, obj_corners2[0], obj_corners2[1], Scalar(0, 0, 255), 1 );
//        line( img_outline, obj_corners2[1], obj_corners2[2], Scalar(0, 0, 255), 1 );
//        line( img_outline, obj_corners2[2], obj_corners2[3], Scalar(0, 0, 255), 1 );
//        line( img_outline, obj_corners2[3], obj_corners2[0], Scalar(0, 0, 255), 1 );

//        line( img_outline, obj_corners[0], obj_corners[1], Scalar(0, 255, 0), 1 );
//        line( img_outline, obj_corners[1], obj_corners[2], Scalar( 0, 255, 0), 1 );
//        line( img_outline, obj_corners[2], obj_corners[3], Scalar( 0, 255, 0), 1 );
//        line( img_outline, obj_corners[3], obj_corners[0], Scalar( 0, 255, 0), 1 );

//        f_name = path_dst + "result/robust_outline_";
//        val_i[128];
//        memset(val_i, 0, 128);
//        sprintf(val_i, "%d", i);
//        str_1 = string(val_i);
//        f_name += str_1;
//        f_name += ".png";
//        imwrite(f_name, img_outline);
//    }
//    cout << "//----------------------------------------------------------------------------" << endl;
//    cout << "// End Test Robust Match: " << i << endl;
//    cout << "//----------------------------------------------------------------------------" << endl;
//    return true;
//}

void computeRotateInnerRectangle();
void computeRotateInnerRectangle()
{
    //    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/rot_roi.png", roi);
    
    //    float scale = 1.0;
    
    //    // Get the rotation matrix with the specifications above
    //    Point2f center(roi.cols / 2.0 , roi.rows / 2.0);
    //    Mat warp_mat = getRotationMatrix2D(center, 45, scale);
    
    //    // Rotate the warped image
    //    // Mat rot_roi = img_src(Rect(img_src.cols / 2 + edge_c / 2, img_src.rows / 2 + edge_r, edge_c, edge_r));    // Rotate the warped image
    //    Mat out;
    //    warpAffine(roi, out, warp_mat, roi.size(), INTER_CUBIC);
    
    //    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/rot_roi_dst_1.png", out);
    
    //    const int prj_c = diag * cos(CV_PI/4);
    //    Mat roi_in = Mat::zeros(Size(prj_c * 2, prj_c * 2), img.type());
    
    //    out(Rect(out.cols / 2 - prj_c, out.rows / 2 - prj_c, prj_c * 2, prj_c * 2)).copyTo(roi_in);
    //    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/rot_roi_in.png", roi_in);
    
    //    //   Mat dst;
    //   resize(img, dst, Size(img.cols * 5.0, img.rows * 5.0), INTER_CUBIC); //Size(m_rgb_src.rows * size_ratio, m_rgb_src.cols * size_ratio), 0, 0, INTER_CUBIC);
    //    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/dst.png", dst);
    //   exit(0);
    
    //    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Nominal/rotate_src.png", img);
    
    //    int roi_h = 128;
    //    int roi_v = 128;
    //    // Rect r1 = Rect(img.cols/2 - roi_h, img.rows/2 - roi_v, img.cols, img.rows);
    
    //    Mat sub_img = img(Rect(img.cols/2 - roi_h/2, img.rows/2 - roi_v/2, roi_h, roi_v)) -= 128;
    //    //  sub_img+=40;
    
    //    int thickness = 4;
    //    int lineType = 8;
    
    //    // img.setTo(255);
    //    Point p_img_0(img.cols/2 - roi_h/2, img.rows/2 - roi_v/2);
    //    Point p_img_1(img.cols/2 + roi_h/2, img.rows/2 - roi_v/2);
    //    Point p_img_2(img.cols/2 + roi_h/2, img.rows/2 + roi_v/2);
    //    Point p_img_3(img.cols/2 - roi_h/2, img.rows/2 + roi_v/2);
    
    //    circle( img,
    //            p_img_0,
    //            2.0,
    //            Scalar( 0, 256, 0 ),
    //            thickness,
    //            lineType );
    
    //    circle( img,
    //            p_img_1,
    //            2.0,
    //            Scalar( 0, 256, 0 ),
    //            thickness,
    //            lineType );
    
    //    circle( img,
    //            p_img_2,
    //            2.0,
    //            Scalar( 0, 256, 0 ),
    //            thickness,
    //            lineType );
    
    //    circle( img,
    //            p_img_3,
    //            2.0,
    //            Scalar( 0, 256, 0 ),
    //            thickness,
    //            lineType );
    
    //    line(img, p_img_0, p_img_1, Scalar(0, 255, 0), 1);
    //    line(img, p_img_1, p_img_2, Scalar(0, 255, 0), 1);
    //    line(img, p_img_2, p_img_3, Scalar(0, 255, 0), 1);
    //    line(img, p_img_3, p_img_0, Scalar(0, 255, 0), 1);
    
    //    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Nominal/rotate_roi.png", img);
    
    //    const float d_e(sqrt(pow(sub_img.cols/2, 2) + pow(sub_img.rows/2, 2)));            // Edge distance: Distance from the center to the edge
    
    //    float angle = 45;
    
    //    if (angle > 90.0)
    //    {
    //        angle -= (int)(angle/90.0);
    //    }
    
    //    float sin_45 = sin(45 * (CV_PI/180.0));
    //    float anglev = (angle - 45) * (CV_PI/180.0);
    
    //    Point p0(img.cols/2, img.rows/2 - roi_v/2);
    //    Point p1(img.cols/2 + roi_h/2, img.rows/2 - roi_v/2);
    //    Point p2(img.cols/2, img.rows/2 + roi_v/2);
    //    Point p3(img.cols/2 - roi_h/2, img.rows/2 + roi_v/2);
    
    //    vector<Point> coord(4);
    
    //    // Points 0-2
    //    float kv = (d_e - sub_img.rows/2) / sin_45;
    //    float res_r = (d_e - sub_img.rows/2) - (kv * abs(sin(anglev)));
    
    //    float kh = (sub_img.cols/2) / sin_45;
    //    float res_c = kh * sin(anglev);
    
    //    coord[0].x = p0.x + res_c;
    //    coord[0].y = p0.y - res_r;
    
    //    coord[2].x = p2.x - res_c;
    //    coord[2].y = p2.y + res_r;
    
    //    // Points 1-3
    //    kv = sub_img.rows/2 / sin_45;
    //    res_r = sub_img.rows/2 + (kv * sin(anglev));
    
    //    kh = (d_e - sub_img.cols/2) / sin_45;
    //    res_c = (d_e - sub_img.cols/2) - (kh * abs(sin(anglev)));
    
    //    coord[1].x = p1.x + res_c;
    //    coord[1].y = p1.y + res_r;
    
    //    coord[3].x = p3.x - res_c;
    //    coord[3].y = p3.y - res_r;
    
    //    circle(img,
    //           coord[0],
    //            2.0,
    //            Scalar(256, 0, 0 ),
    //            thickness,
    //            lineType );
    
    //    circle(img,
    //           coord[1],
    //            2.0,
    //            Scalar(0, 0, 256 ),
    //            thickness,
    //            lineType );
    
    //    circle(img,
    //           coord[2],
    //            2.0,
    //            Scalar(0, 256, 256 ),
    //            thickness,
    //            lineType );
    
    //    circle(img,
    //           coord[3],
    //            2.0,
    //            Scalar(0,
    //                   256, 0, 256 ),
    //            thickness,
    //            lineType );
    
    //    line(img, coord[0], coord[1], Scalar(255, 0, 0), 1);
    //    line(img, coord[1], coord[2], Scalar(255, 0, 0), 1);
    //    line(img, coord[2], coord[3], Scalar(255, 0, 0), 1);
    //    line(img, coord[3], coord[0], Scalar(255, 0, 0), 1);
    
    //    float rct_h_l = max(p_img_0.x, coord[3].x);
    //    float rct_h_r = min(p_img_1.x, coord[1].x);
    
    //    float rct_v_t = max(p_img_1.y, coord[0].y);
    //    float rct_v_b = min(p_img_2.y, coord[3].y);
    
    //    // Inner Rectangle
    //    Rect inner_rect(rct_h_l, rct_v_t, rct_h_r - rct_h_l, rct_v_b - rct_v_t);
    //    Mat inner_rotat(img);
    //    img(Rect(rct_h_l, rct_v_t, rct_h_r - rct_h_l, rct_v_b - rct_v_t)) = 0;
    
    //    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Nominal/rotate_inner_crop.png", img(Rect(rct_v_t, rct_h_l, rct_v_b - rct_v_t, rct_h_r - rct_h_l)));
    
    //    imwrite("D:/PRJ/Benchmark/OBJECTIVE_TEST/Nominal/rotate_crop.png", img);
    
    // string path("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/");
}
