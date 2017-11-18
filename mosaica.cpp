////-------------------------------------------------------------------------------------
//// Project name: COHERENT MOSAICA
////
//// Creation Date: 20/03/2016
//// Created by: Roberto Giudici - OBELIX, UBS, IRISA, COSTEL, Universiy of Rennes 2
//// Description: Super pFxel time coherent based Mosaicing algorithm
////-------------------------------------------------------------------------------------

//#include "spixtools.h"
//#include "mosaica.h"
//#include "mosaicatools.h"
//#include "gnuplotexport.h"
//// #include "registration.h"
//#include "tools.h"
//#include <queue>
//#include <stack>

//MosaicStack::MosaicStack()
//{
//}

//Mosaica::Mosaica()
//{
//}

//////===========================================================================
//////	CreateSuperPixels
//////
//////	The main function
//////===========================================================================
////bool Mosaica::createSuperImageStack(const string src_path, const string dst_path, const int spcount, const double compactness)
////{
////    string file_in;
////    string file_out = dst_path + "_slick";

////    char s_spcount[64];
////    memset(s_spcount, 0, 64);
////    sprintf(s_spcount, "%04d", spcount);

////    char s_compactness[64];
////    memset(s_compactness, 0, 64);
////    sprintf(s_compactness, "%.3f", compactness);

////    file_out += "_";
////    file_out += s_spcount;
////    file_out += "_";
////    file_out += s_compactness;
////    file_out += ".bmp";

////    const int nbr_imgs = 4;

////    for (int src_idx = 0; src_idx < nbr_imgs; src_idx++)
////    {
////        char s_src_idx[64];
////        memset(s_src_idx, 0, 64);
////        sprintf(s_src_idx, "%04d", src_idx);

////        file_in = src_path + "img1_" + string(s_src_idx) + ".jpg";
////        Mat rgb_src = imread(file_in, CV_LOAD_IMAGE_COLOR);

////        if (rgb_src.rows == 0 || rgb_src.cols == 0)
////        {
////            cout << "Source file: " << file_in << " is empty !!" << endl;
////            return false;
////        }


////        cout << "-----------------------------------------------------------------------" << endl;
////        cout << "Simulation of Registering" << endl;
////        cout << "-----------------------------------------------------------------------" << endl;

////        int sz = 0;

////        if (src_idx == 0)
////        {
////            //   imwrite( "D:/PRJ/ALGO_SLIC/src/surf_img0.bmp", rgb_src );
////            m_vct_src_shift.push_back(Point(0, 0)); // Point(H, V)
////            m_vct_rgb_src.push_back(rgb_src);
////        }
////        else if (src_idx > 0)
////        {
////            //   imwrite( "D:/PRJ/ALGO_SLIC/src/surf_img1.bmp", rgb_src );
////            // m_vct_src_shift.push_back(Point(0, 0)); // Point(H, V)

////            // m_vct_src_shift.push_back(Point(0, 100)); // Point(H, V)
////            m_vct_src_shift.push_back(Point(0, 0)); // Point(H, V)

////            cout << "-----------------------------------------------------------------------" << endl;
////            cout << " Registering" << endl;
////            cout << "-----------------------------------------------------------------------" << endl;

////            // Mat rgb_registered;
////            // match_surf(m_vct_rgb_src[0], rgb_src, rgb_registered);

////            m_vct_rgb_src.push_back(rgb_src);
////            // imwrite( "D:/PRJ/ALGO_SLIC/src/surf_img_0_1_registered.bmp", rgb_registered);
////        }

////        cout << "-----------------------------------------------------------------------" << endl;
////        cout << "Processing Image " << src_idx << " - Size (" << rgb_src.rows << ", " << rgb_src.cols << ")" << endl;
////        cout << "-----------------------------------------------------------------------" << endl;

////        //  Adds the image to the source image vector
////        // m_vct_rgb_src.push_back(rgb_src);

////        // Adds an empty labels array to the vector of label arrays
////        // const int sz = rgb_src.rows * rgb_src.cols;
////        int* labels = new int[sz];
////        m_vct_labels.push_back(labels);

////        m_vct_overlap.push_back(vector<int>(sz));

////        GeoImage geo_img(rgb_src);
////        m_vct_simg.push_back(SuperImage(geo_img, src_idx));
////    }

////    cout << "-----------------------------------------------------------------------" << endl;
////    cout << "Photoset correctely loaded !!" << endl;
////    cout << "-----------------------------------------------------------------------" << endl;

////    cout << "-----------------------------------------------------------------------" << endl;
////    cout << "Simulating a 0.01 degre rotation of the source image 1" << endl;
////    cout << "-----------------------------------------------------------------------" << endl;
////    // m_vct_src_shift[1].x = 0.0;
////    //  m_vct_src_shift[1].y = 0.0;

////    for (int src_idx = 0; src_idx < m_vct_src_shift.size(); src_idx++)
////    {
////        CreateSuperPixels(src_idx, spcount, compactness);
////    }
////}

////---------------------------------------------------------------------------------------
//// Converts the SLIC internal RGB format to OpenCv RGB format
////------------------------------------------------------------------------------------
//void convertInternalToRGB(const Mat rgb_int, Mat rgb_dst);
//void convertInternalToRGB(const Mat rgb_int, Mat rgb_dst)
//{
//    unsigned int *p_int_base2 = (unsigned int *)rgb_int.data;
//    unsigned char *p_dst_base2 = rgb_dst.data;
//    unsigned char *p_dst_end2 = rgb_dst.data + (rgb_dst.rows * rgb_dst.cols) * 3;
//    unsigned int *p_int2 = p_int_base2;
//    unsigned char *p_dst = p_dst_base2;
    
//    while (p_dst < p_dst_end2)
//    {
//        // Red
//        *p_dst++ = *p_int2 >> 16 & 0xff;
        
//        // Green
//        *p_dst++ = *p_int2 >> 8 & 0xff;
        
//        // Blue
//        *p_dst++ = *p_int2 & 0xff;
        
//        p_int2++;
//    }
//}

////---------------------------------------------------------------------------------------
//// Converts the  RGB format to OpenCv RGB format to SLIC internal format
////---------------------------------------------------------------------------------------
////oid convertRGBToInternal(const Mat src_rgb, Mat &rgb_int);
//void Mosaica::convertRGBToInternal(const Mat src_rgb, Mat &rgb_int)
//{
//    unsigned char *p_src_base = src_rgb.data;
//    unsigned int *p_int_base = (unsigned int *)rgb_int.data;
//    unsigned int *p_int_end = (unsigned int *)rgb_int.data + (rgb_int.rows * rgb_int.cols);
    
//    unsigned  char *p_src = p_src_base;
//    unsigned int *p_int = p_int_base;
    
//    while (p_int < p_int_end)
//    {
//        // Red
//        *p_int = *p_src << 16;
//        p_src++;
        
//        // Green
//        *p_int |= *p_src << 8;
//        p_src++;
        
//        // Blue
//        *p_int |= *p_src;
//        p_src++;
        
//        p_int++;
//    }
//}

////-----------------------------------------------------------------------------
//// ComputeSpixBoundaries
////-----------------------------------------------------------------------------
//bool Mosaica::ComputeSpixBoundaries(const int src_idx)
//{
//    Mat *p_img = &m_vct_rgb_src[src_idx];
//    Mat rgb_dst = Mat::zeros(p_img->rows, p_img->cols, CV_8UC3);
//    Mat drawing = Mat::zeros(p_img->size(), CV_8UC3);
//    SuperPixel *p_spix = 0;

//    //--------------------------------------------------------------------
//    // Computes the boundary on blank image
//    //--------------------------------------------------------------------
//    for (int i = 0; i < m_vct_simg[src_idx].m_vct_spix.size(); i++)
//    {
//        p_spix = &(m_vct_simg[src_idx].m_vct_spix[i]);

//        if (!p_spix)
//        {
//            return false;
//        }
//        Mat rgb_blank = Mat::zeros(p_img->rows, p_img->cols, CV_8UC3);
//        Mat intern_blank = Mat::zeros(p_img->rows, p_img->cols, CV_32FC3);

//        unsigned int *p_intern_blank = (unsigned int *)intern_blank.data;
//        convertRGBToInternal(rgb_blank, intern_blank);
//        SLIC slic;
//        slic.DrawContoursAroundSegments(p_intern_blank, m_vct_labels[src_idx], rgb_blank.cols, rgb_blank.rows, 0);
//        convertInternalToRGB(intern_blank, rgb_dst);

//        // Show in a window
//        // namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//        // imshow( "Contours", drawing );
//        // waitKey(0);
//    }

//    //--------------------------------------------------------------------
//    // Finds the boundary of the super pixel
//    //--------------------------------------------------------------------
//    vector<vector<Point> > contours;
//    vector<Vec4i> hierarchy;

//    Mat gray = Mat(rgb_dst.size(), CV_8UC1);
//    cvtColor(rgb_dst, gray, COLOR_RGB2GRAY);
//    RNG rng(12345);

//    // Draws a rectangle to close the Pix corresponing to the outer boundaries of the image
//    rectangle(gray, Rect(0, 0, gray.cols, gray.rows), Scalar(255, 255, 255), 2.0);

//    // Finds the most salient contours. The complete set of boundary px will be computed by interpolation
//    findContours(gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

//    // Draw contours
//    vector<vector<Point>> vct_contours;

//    for( int k = 1; k < contours.size(); k++ )
//    {
//        vector<Point>vct;
//        vct_contours.push_back(vct);

//        // Computes the missing point by interpolation
//        for( int z = 0; z <= contours[k].size(); z++ )
//        {
//            int z1 = z%contours[k].size();

//            vct_contours[k - 1].push_back(Point(contours[k][z1].y, contours[k][z1].x));

//            // cout << "---------------------------------------------------------------------------" << endl;
//            //  cout << z << "  Native x: " << contours[k][z1].x << "  y: " << contours[k][z1].y << endl;
//            //  cout << "---------------------------------------------------------------------------" << endl;
//            circle(drawing, Point( contours[k][z1].x, contours[k][z1].y), 2, Scalar(0, 255, 255), CV_FILLED, 8,0);

//            if (z > 0)
//            {
//                // int d = (int)sqrt(pow(contours[k][z].x - contours[k][z - 1].x, 2) + pow(contours[k][z].y - contours[k][z - 1].y, 2));
//                int dx = abs(contours[k][z1].x - contours[k][z - 1].x);
//                int dy = abs(contours[k][z1].y - contours[k][z - 1].y);
//                int d = max(dx, dy);

//                float step_x = (contours[k][z1].x - contours[k][z - 1].x)/d;
//                float step_y = (contours[k][z1].y - contours[k][z - 1].y)/d;

//                for (int j = 0; j < d; j++)
//                {
//                    vct_contours[k - 1].push_back(Point(contours[k][z - 1].y + (j * step_y), contours[k][z - 1].x + (j * step_x)));
//                    // cout << z << "  Interpolation x: " << vct_contours[k - 1][vct_contours[k - 1].size() - 1].x << "  y: " << vct_contours[k - 1][vct_contours[k - 1].size() - 1].y << endl;
//                    // circle(drawing, Point(vct_contours[k - 1][vct_contours[k - 1].size() - 1].y, vct_contours[k - 1][vct_contours[k - 1].size() - 1].x), 1, Scalar(255, 0, 0), CV_FILLED, 8,0);
//                }

//                // Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//                // drawContours( drawing, contours, k, color, 2, 8, hierarchy, 0, Point() );
//            }
//        }
//        // Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//        // drawContours( drawing, contours, k, color, 2, 8, hierarchy, 0, Point() );
//    }

//    // Show in a window
//    // namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//    // imshow( "Contours", drawing );
//    // waitKey(0);

//    //--------------------------------------------------------------------
//    // Matches the OpenCv generated boundary with the SLIC generated sPix
//    //--------------------------------------------------------------------
//    for (int i = 0; i < m_vct_simg[src_idx].m_vct_spix.size(); i++)
//    {
//        p_spix = &(m_vct_simg[src_idx].m_vct_spix[i]);

//        if (!p_spix)
//        {
//            return false;
//        }

//        bool found = false;
//        int cnt_idx = 0;
//        while (found == false && cnt_idx < contours.size())
//        {
//            // Finds the centroid matching with the selected boundary
//            // Makes sure the centroid is properly included in the boundary coordinates

//            // Scans all px belonging to the selected sPix until a px matches with the contours[cnt_idx][0]
//            int px_idx = 0;

//            const int matching_px = 0;
//            while (found == false && px_idx < p_spix->m_spix_pos.size())
//            {
//                if (vct_contours[cnt_idx][matching_px].x == p_spix->m_spix_pos[px_idx].m_r &&
//                        vct_contours[cnt_idx][matching_px].y == p_spix->m_spix_pos[px_idx].m_c)
//                {
//                    // circle(drawing, Point(ay, ax), 15, Scalar(0, 0, 255), CV_FILLED, 8,0);
//                    // circle(drawing, Point(p_spix->m_spix_pos[px_idx].m_c, p_spix->m_spix_pos[px_idx].m_r), 10, Scalar(255, 0, 0), CV_FILLED, 8,0);
//                    // circle(drawing, Point(p_spix->m_gy, p_spix->m_gx), 4, Scalar(255, 255, 0), CV_FILLED, 8,0);

//                    // namedWindow("Contours", CV_WINDOW_AUTOSIZE);
//                    // imshow("Contours", drawing);
//                    // waitKey(0);

//                    // Adds the boundary pixels to the sPix
//                    for (int cnt_px_idx = 0; cnt_px_idx < vct_contours[cnt_idx].size(); cnt_px_idx++)
//                    {
//                        //circle(drawing, Point(contours[cnt_idx][cnt_px_idx].yx, contours[cnt_idx][cnt_px_idx].y), 4, Scalar(0, 0, 255), CV_FILLED, 8,0);
//                        p_spix->m_vct_boundary.push_back(Point(vct_contours[cnt_idx][cnt_px_idx].y, vct_contours[cnt_idx][cnt_px_idx].x));
//                    }
//                    found = true;
//                }
//                px_idx++;
//            }
//            cnt_idx++;
//        }
//    }
//    return true;
//}

////-----------------------------------------------------------------------------
//// ComputeMORE
//// Morphological Oriented Regularity Estimator
////-----------------------------------------------------------------------------
//bool Mosaica::ComputeMORE(const int src_idx)
//{
//    Mat *p_img = &m_vct_rgb_src[src_idx];
//    Mat drawing = Mat::zeros(p_img->size(), CV_8UC3);
//    SuperPixel *p_spix = 0;
//    vector<float> vct_mse(m_vct_simg[src_idx].m_vct_spix.size());

//    //-----------------------------------------------------------------------------------
//    // Computes the sum of cartesian distance
//    //-----------------------------------------------------------------------------------
//    for (int spix_idx = 0; spix_idx < m_vct_simg[src_idx].m_vct_spix.size(); spix_idx++)
//    {
//        p_spix = &(m_vct_simg[src_idx].m_vct_spix[spix_idx]);
//        float module_min = numeric_limits<float>::max();
//        float module_max = 0.0;

//        float phase_min = numeric_limits<float>::max();
//        float phase_max = 0.0;

//        float module_sum = 0.0;
//        float phase_sum = 0.0;

//        int step = 1; //p_spix->m_vct_boundary.size() / 1;

//        int sub_size = p_spix->m_vct_boundary.size()/step;
//        vector<float> vct_dist(sub_size);
//        vector<float> vct_module(sub_size);
//        vector<float> vct_phase(sub_size);

//        // Min ray of the circle internal to the sPix
//        int bnd_min = INT_MIN;

//        for (int bnd_idx = 0; bnd_idx < p_spix->m_vct_boundary.size(); bnd_idx+= step)
//        {
//            int gx =  p_spix->m_gx;
//            int gy =  p_spix->m_gy;
//            int m1_x = p_spix->m_vct_boundary[bnd_idx].x;
//            int m1_y = p_spix->m_vct_boundary[bnd_idx].y;

//            // Distance projections
//            int dx = ((int)p_spix->m_gx - p_spix->m_vct_boundary[bnd_idx].y);
//            int dy = ((int)p_spix->m_gy - p_spix->m_vct_boundary[bnd_idx].x);

//            // arrowedLine(drawing, Point(gy, gx), Point(m1_x, m1_y), Scalar(rand() % 255, rand() % 255, rand() % 255), 1, 8, 0, 0.02);

//            float norm_weight = 4;
//            float dist = sqrt(pow(dx, norm_weight) + pow(dy, norm_weight));
//            vct_dist[bnd_idx] = dist;

//            if (dist < bnd_min)
//            {
//                bnd_min = dist;
//            }
//        }

//        // Computes the Oriented difference of the distance
//        for (int bnd_idx = 0; bnd_idx < p_spix->m_vct_boundary.size(); bnd_idx+= step)
//        {
//            int gx = p_spix->m_gx;
//            int gy = p_spix->m_gy;

//            //   bnd_idx = 64;
//            int by = p_spix->m_vct_boundary[bnd_idx].x;
//            int bx = p_spix->m_vct_boundary[bnd_idx].y;

//            float dx = gx - bx; //p_spix->m_vct_boundary[bnd_idx].x;
//            float dy = gy - by; //p_spix->m_vct_boundary[bnd_idx].y;

//            // Computes module
//            float bnd_mod = vct_dist[bnd_idx] - bnd_min;

//            // Computes the phase
//            float bnd_phase = atan2(dx, dy);
//            bnd_phase *= (180.0/(3.1415));

//            if (dx >= 0.0 && dy >= 0.0)
//            {
//                bnd_phase = 360.0 + bnd_phase;
//                bnd_phase -= 90.0;
//            }
//            else if (dx >= 0.0 && dy < 0.0)
//            {
//                // bnd_phase = 360.0 - bnd_phase;
//                bnd_phase -= 90.0;
//            }
//            else if (dx < 0 && dy >= 0)
//            {
//                bnd_phase = 360.0 + bnd_phase;
//                bnd_phase -= 90.0;
//            }
//            else if (dx < 0 && dy < 0)
//            {
//                bnd_phase = 360.0 + bnd_phase;
//                bnd_phase -= 90.0;
//            }

//            // Coordinates (x, y) of the centroid are inverted to bondary coordinates !!
//            // x, y should be swapped !!

//            // Transforms the angle expressed in cartesian coordinates into image (x, y) coordintes
//            bnd_phase += (bnd_phase > 360.0 ? -360.0 : 0);

//            // arrowedLine(drawing, Point(by, bx), Point(gy, gx), Scalar(255 * bnd_mod/800, 255 * bnd_mod/800, 255 * bnd_mod/800), 1, 8, 0, 0.02);

//            if (bnd_idx % 40 == 0)
//            {
//                circle(drawing, Point(by, bx), 2, Scalar(0, 255, 255), CV_FILLED, 8);
//                //arrowedLine(drawing, Point(by, bx), Point(gy, gx), Scalar(255 * bnd_mod/100, 255 * bnd_mod/100, 255 * bnd_mod/100), 1, 8, 0, 0.02);
//                arrowedLine(drawing, Point(by, bx), Point(gy, gx), Scalar(rand() % 255, rand() % 255, rand() % 255), 1, 8, 0, 0.02);

//                char overlay_txt[64];
//                memset(overlay_txt, 0, 64);
//                float font_size = 0.3;

//                // Draws module
//                sprintf(overlay_txt, "%.2f", bnd_mod);
//                putText(drawing, overlay_txt, cvPoint(by + 0, bx + 5), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(255, 255, 255), 1, CV_AA);

//                // Draws phase
//                sprintf(overlay_txt, "%.2f", bnd_phase);
//                //putText(drawing, overlay_txt, cvPoint(by + 0, bx + 15), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(255, 255, 255), 1, CV_AA);
//            }

//            if (bnd_mod < module_min)
//            {
//                module_min = bnd_mod;
//            }
//            if (bnd_mod >= module_max)
//            {
//                module_max = bnd_mod;
//            }
//            if (bnd_phase < phase_min)
//            {
//                phase_min = bnd_phase;
//            }
//            if (bnd_phase >= phase_max)
//            {
//                phase_max = bnd_phase;
//            }

//            // Sums up the module
//            module_sum += bnd_mod;

//            vct_module[bnd_idx] = bnd_mod;
//            vct_phase[bnd_idx] = bnd_phase;

//            // Show in a window
//            //    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//            //    imshow( "Contours", drawing );
//            //    waitKey(0);
//        }


//        // Computes the means of the module
//        float module_mean = module_sum / (p_spix->m_vct_boundary.size() / step);

//        // Normalizes the module
//        float norm_module_mean = (module_mean - module_min) / (module_max - module_min);

//        float phase_mod_max = 0.0;

//        // Multiplies each phase with the module strength to romote the phases belonging to higher modules
//        for (int bnd_idx = 0; bnd_idx < p_spix->m_vct_boundary.size(); bnd_idx+= step)
//        {
//            float norm_module = (vct_module[bnd_idx] - module_min) / (module_max - module_min);
//            phase_sum += vct_phase[bnd_idx];
//        }

//        // Computes the MSE
//        float mse = 0.0;
//        for (int bnd_idx = 0; bnd_idx < p_spix->m_vct_boundary.size(); bnd_idx+= step)
//        {
//            mse += pow(vct_module[bnd_idx] - module_mean, 2);
//        }

//        mse /= p_spix->m_vct_boundary.size();
//        mse = sqrt(mse);
//        vct_mse[spix_idx] = mse;

//        float phase_mean = phase_sum / p_spix->m_vct_boundary.size();

//        char overlay_txt[64];
//        memset(overlay_txt, 0, 64);
//        float font_size = 0.3;

//        // Draws module
//        sprintf(overlay_txt, "%.2f", norm_module_mean);
//        // putText(drawing, overlay_txt, cvPoint(p_spix->m_gy + 10, p_spix->m_gx + 10), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(255, 255, 255), 1, CV_AA);

//        sprintf(overlay_txt, "%.2f", mse);
//        // putText(drawing, overlay_txt, cvPoint(p_spix->m_gy + 10, p_spix->m_gx + 20), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(255, 255, 255), 1, CV_AA);

//        // Show in a window
//        // namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//        //  imshow( "Contours", drawing );
//        // waitKey(0);
//        //imwrite("D:/PRJ/ALGO_SLIC/src/bound_points_sub_60.jpg", drawing);
//    }

//    // Computes the normalized Variance

//    float norm_mse = 0.0;
//    float sum_mse = 0.0;
//    float mean_mse = 0.0;
//    float min_mse = numeric_limits<float>::max();
//    float max_mse = 0.0;

//    for (int spix_idx = 0; spix_idx < m_vct_simg[src_idx].m_vct_spix.size(); spix_idx++)
//    {
//        if (min_mse >= vct_mse[spix_idx])
//        {
//            min_mse = vct_mse[spix_idx];
//        }

//        if (max_mse < vct_mse[spix_idx])
//        {
//            max_mse = vct_mse[spix_idx];
//        }

//        sum_mse += vct_mse[spix_idx];
//    }

//    mean_mse = sum_mse / m_vct_simg[src_idx].m_vct_spix.size();

//    // Normalizes the mse
//    for (int spix_idx = 0; spix_idx < m_vct_simg[src_idx].m_vct_spix.size(); spix_idx++)
//    {
//        p_spix = &(m_vct_simg[src_idx].m_vct_spix[spix_idx]);

//        if (!p_spix)
//        {
//            return false;
//        }

//        vct_mse[spix_idx] = (vct_mse[spix_idx] - min_mse) / (max_mse - min_mse);

//        char overlay_txt[64];
//        memset(overlay_txt, 0, 64);
//        float font_size = 0.3;
//        sprintf(overlay_txt, "%.2f",  vct_mse[spix_idx]);
//        putText(drawing, overlay_txt, cvPoint(p_spix->m_gy + 10, p_spix->m_gx + 20), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(255, 255, 255), 1, CV_AA);
//    }

//    char overlay_txt[64];
//    memset(overlay_txt, 0, 64);
//    float font_size = 0.3;

//    float norm_mean_mse = (mean_mse - min_mse) / (max_mse - min_mse);
//    sprintf(overlay_txt, "%.2f",  norm_mean_mse);
//    putText(drawing, overlay_txt, cvPoint(p_spix->m_gy + 10, p_spix->m_gx + 40), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(255, 255, 255), 1, CV_AA);

//    // Revoir la phase en calculant la somme des projections horizontales et verticales

//    // Show in a window
//    // namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//    // imshow( "Contours", drawing );
//    // waitKey(0);

//    imwrite("D:/PRJ/ALGO_SLIC/src/bound_regulairy_1.jpg", drawing);
//    // imwrite("D:/PRJ/ALGO_SLIC/src/bound_angle_1.jpg", drawing);

//    return true;
//}

////-----------------------------------------------------------------------------
//// CreateSuperPixels
////-----------------------------------------------------------------------------
//bool Mosaica::CreateSuperPixels(const int src_idx, const int spcount, const double compactness)
//{
//    Mat *p_img = &m_vct_rgb_src[src_idx];
//    Mat rgb_int = Mat::zeros(p_img->rows, p_img->cols, CV_32FC3);
    
//    imwrite("D:/PRJ/ALGO_SLIC/src/a_src.jpg", *p_img);
//    convertRGBToInternal(*p_img, rgb_int);
    
//    int sz = p_img->cols * p_img->rows;
    
//    //---------------------------------------------------------
//    if (spcount < 20 || spcount > sz/4)
//    {
//        cout << "sPix size must be < 200 !!" << endl;
//        // i.e the default size of the superpixel is 200 pixels
//        //  return false;
//    }
    
//    if (compactness < 1.0 || compactness > 80.0)
//    {
//        cout << "Compactness range [1.0..80.0]" << endl;
//        return false;
//    }
//    //---------------------------------------------------------
    
//    int numlabels(0);
    
//    SLIC slic;
//    unsigned int *img = (unsigned int *)rgb_int.data;
//    slic.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels((unsigned int *)rgb_int.data, p_img->cols, p_img->rows, m_vct_labels[src_idx], numlabels, spcount, compactness);

//    // Overlays the boundary over the src image
//    slic.DrawContoursAroundSegments(img, m_vct_labels[src_idx], p_img->cols, p_img->rows, 0);
//    Mat rgb_dst = Mat::zeros(p_img->rows, p_img->cols, CV_8UC3);
//    convertInternalToRGB(rgb_int, rgb_dst);
//    // imwrite("D:/PRJ/ALGO_SLIC/src/over_2.jpg", rgb_dst);
    
//    m_vct_rgb_dst.push_back(rgb_dst);
    
//    vector<int> clusters(p_img->rows * p_img->cols, -1);
    
//    SuperPixel *p_spix = 0;
    
//    for (int r = 0; r < p_img->rows; r++)
//    {
//        for (int c = 0; c < p_img->cols; c++)
//        {
//            const int i = (p_img->cols * r) + c;
            
//            // At each new label, a new superpixel is created
//            if (clusters[m_vct_labels[src_idx][i]] == -1)
//            {
//                // cout << "Creating SuperPixel: " << vct_super_pix.size() << " Label: " << **p_labels[i] << endl;
//                m_vct_simg[src_idx].m_vct_spix.push_back(SuperPixel( m_vct_labels[src_idx][i], PixPos(r, c)));
//                clusters[ m_vct_labels[src_idx][i]]++;
//            }
            
//            p_spix = &(m_vct_simg[src_idx].m_vct_spix[ m_vct_labels[src_idx][i]]);
            
//            if (!p_spix)
//            {
//                return false;
//            }
            
//            if (p_spix->m_min_x > r)
//            {
//                p_spix->m_min_x = r;
//            }
            
//            if (p_spix->m_min_y > c)
//            {
//                p_spix->m_min_y = c;
//            }
            
//            if (p_spix->m_max_x <= r)
//            {
//                p_spix->m_max_x = r;
//            }
            
//            if (p_spix->m_max_y <= c)
//            {
//                p_spix->m_max_y = c;
//            }
            
//            // Sums up the rgb channels into grayscale
//            p_spix->m_mean_luma += p_img->at<Vec3b>(r, c)[0];
//            p_spix->m_mean_luma += p_img->at<Vec3b>(r, c)[1];
//            p_spix->m_mean_luma += p_img->at<Vec3b>(r, c)[2];
            
//            p_spix->m_spix_pos.push_back(PixPos(r, c));
//            clusters[ m_vct_labels[src_idx][i]]++;
//            //  cout << "Adding pixel to SuperPixel" << endl;
//        }
//    }

//    //--------------------------------------------------------------------------
//    // Computes the gravity points and stats of the sPix
//    //--------------------------------------------------------------------------
//    //  Mat drawing = Mat::zeros(this->m_vct_rgb_src[0].size(), CV_8UC3 );
    
//    for (int i = 0; i < m_vct_simg[src_idx].m_vct_spix.size(); i++)
//    {
//        p_spix = &(m_vct_simg[src_idx].m_vct_spix[i]);
        
//        if (!p_spix)
//        {
//            return false;
//        }
        
//        // Calculates the gravity of each sPix
//        p_spix->m_gx = (p_spix->m_min_x + p_spix->m_max_x) / 2;
//        p_spix->m_gy = (p_spix->m_min_y + p_spix->m_max_y) / 2;
        
//        // Calculates the luminance mean
//        p_spix->m_mean_luma /= (p_spix->m_spix_pos.size() * 3);
        
//        // Calculates the elongation
//        const int32_t height = abs(p_spix->m_max_x - p_spix->m_min_x);
//        const int32_t width = abs(p_spix->m_max_y - p_spix->m_min_y);
//        p_spix->m_elongation = (float)height / width;
        
//        // Calculates the cartesian distance between the centroid and the origin
//        p_spix->m_distance = sqrt(pow(p_spix->m_gx, 2) + pow(p_spix->m_gy, 2));
        
//        // Calculates the signature as polynomial model
//        m_vct_simg[src_idx].set_wights(0.2, 1.0, 1.0, 1.0);
//        m_vct_simg[src_idx].calc_signature(p_spix);
        
//        // Calculates the Stats
//        // Sums up the statistical values from each different sPix
//        m_vct_simg[src_idx].m_mean_size += p_spix->m_spix_pos.size();
//        m_vct_simg[src_idx].m_mean_gx += p_spix->m_gx;
//        m_vct_simg[src_idx].m_mean_gy += p_spix->m_gy;
//        m_vct_simg[src_idx].m_mean_luma += p_spix->m_mean_luma;
//        m_vct_simg[src_idx].m_elongation += p_spix->m_elongation;
//        m_vct_simg[src_idx].m_distance += p_spix->m_distance;
//    }
    
//    int nbr_spix = m_vct_simg[src_idx].m_vct_spix.size();
//    m_vct_simg[src_idx].m_mean_size /= nbr_spix;
//    m_vct_simg[src_idx].m_mean_gx /= nbr_spix;
//    m_vct_simg[src_idx].m_mean_gy /= nbr_spix;
//    m_vct_simg[src_idx].m_mean_luma /= nbr_spix;
//    m_vct_simg[src_idx].m_elongation /= nbr_spix;
//    m_vct_simg[src_idx].m_distance /= nbr_spix;

//    // Computes the set of boundary px and stores it into the sPix
//    ComputeSpixBoundaries(src_idx);

//    // Computes the Morphological Oriented Regularity Estimator
//    ComputeMORE(src_idx);


//    //    const bool enable_dump = false;

//    //        if (enable_dump == true)
//    //        {
//    //            for (int i = 0; i < m_vct_simg[src_idx].m_vct_spix.size(); i++)
//    //            {
//    //                p_spix = &(m_vct_simg[src_idx].m_vct_spix[i]);

//    //                cout << "-----------------------------------------------------" << endl;
//    //                cout << "- SPix: " << i << " Label: " << p_spix->m_label << endl;
//    //                cout << "- Size: " <<  p_spix->m_spix_pos.size() << endl;
//    //                cout << "- Gravity (" <<  p_spix->m_gx  << ", " <<   p_spix->m_gy << ")" << endl;
//    //                cout << "- mean Luma: " <<  p_spix->m_mean_luma << endl;
//    //                cout << "- Elongation: " <<  p_spix->m_elongation << endl;
//    //                cout << "- Distance from Origin: " <<  p_spix->m_distance << endl;
//    //                cout << "- Signature: " <<  p_spix->m_signature << endl;
//    //                cout << "-----------------------------------------------------" << endl;
//    //            }
//    //        }

//    //        // Print the list of sPixs, their size and their contents
//    //        cout << "---------------------------------------------------" << endl;
//    //        cout << "Printing statistics" << endl;
//    //        cout << "---------------------------------------------------" << endl;

//    //        cout << "Mean Size: " << m_vct_simg[src_idx].m_mean_size << endl;
//    //        cout << "Mean GX: " << m_vct_simg[src_idx].m_mean_gx << endl;
//    //        cout << "Mean GY: " << m_vct_simg[src_idx].m_mean_gy << endl;
//    //        cout << "Mean Luma: " << m_vct_simg[src_idx].m_mean_luma << endl;
//    //        cout << "Mean Elongation: " << m_vct_simg[src_idx].m_elongation << endl;
//    //        cout << "Mean Distance: " <<  m_vct_simg[src_idx].m_distance << endl;
//    return true;
//}

////-----------------------------------------------------------------------------

//bool Mosaica::spixMatch()
//{
//    const bool enable_dump_spix = false;

//    if (enable_dump_spix == true)
//    {
//        // Creates and stores each sPix for visual matching
//        for (int img_idx = 0; img_idx < m_vct_simg.size(); img_idx++)
//        {
//            SuperImage *p_simg_ref = &(m_vct_simg[img_idx]);

//            for (int spix_idx = 0; spix_idx < p_simg_ref->m_vct_spix.size(); spix_idx++)
//            {
//                // Creates and stores each sPix for visual matching
//                SuperPixel *p_spix = &(p_simg_ref->m_vct_spix[spix_idx]);

//                SPixTools spt("");
//               // spt.dumpSpix(img_idx, spix_idx, m_vct_rgb_src[img_idx], p_spix);
//            }
//        }
//    }

//    float epsilon = 0.5;

//    // Gets the min max super image size
//    int min_simg_size = INT_MAX;
//    int max_simg_size = 0;

//    for (int img_idx = 0; img_idx < m_vct_simg.size(); img_idx++)
//    {
//        if (m_vct_simg[img_idx].m_vct_spix.size() > max_simg_size)
//        {
//            max_simg_size = m_vct_simg[img_idx].m_vct_spix.size();
//        }

//        if (m_vct_simg[img_idx].m_vct_spix.size() < min_simg_size)
//        {
//            min_simg_size = m_vct_simg[img_idx].m_vct_spix.size();
//        }
//    }

//    // Difference in size between the image that contains the highest and the lowest number of sPix
//    // const int diff_simg_size = max_simg_size - min_simg_size;

//    for (int img_idx = 1; img_idx < m_vct_simg.size(); img_idx++)
//    {
//        SuperImage *p_simg_ref = &(m_vct_simg[0]);
//        SuperImage *p_simg_stack = &(m_vct_simg[img_idx]);

//        if (!p_simg_ref)
//        {
//            return false;
//        }

//        // Scans all sPix of each sImage
//        for (int spix_idx = 0; spix_idx < p_simg_ref->m_vct_spix.size(); spix_idx++)
//        {
//            vector<float> vct(1);
//            m_vct_spix_matches.push_back(vct);

//            SuperPixel *p_spix = &(p_simg_ref->m_vct_spix[spix_idx]);

//            for (int j = 0; j < p_simg_ref->m_vct_spix.size(); j++)
//            {

//                float sign1 = p_simg_ref->m_vct_spix[spix_idx].m_signature;
//                float sign2 = p_simg_stack->m_vct_spix[j].m_signature;

//                float err = abs(sign2 - sign1);

//                if (err < epsilon)
//                {
//                    m_vct_spix_matches[spix_idx].push_back(j);
//                    cout  << "found !!" << endl;
//                }
//            }
//        }
//    }
//    return true;
//}

////-----------------------------------------------------------------------------

//bool Mosaica::spixOverlap()
//{
//    //    for (int r = 0; r < m_vct_rgb_src[0].rows; r++)
//    //    {
//    //        for (int c = 0; c < m_vct_rgb_src[0].cols; c++)
//    //        {
//    //            int i = (m_vct_rgb_src[0].cols * r) + c;

//    //            // Updates the label overlapping
//    //            int px_ovr_cnt = 0;

//    //            for (int k = 1; k < m_vct_labels.size(); k++)
//    //            {
//    //                if (i < m_vct_rgb_src[k].rows * m_vct_rgb_src[k].cols)
//    //                {
//    //                    if (m_vct_labels[0][i] == m_vct_labels[k][i])
//    //                    {
//    //                        px_ovr_cnt++;
//    //                    }
//    //                }
//    //            }

//    //            if (px_ovr_cnt >= 1)
//    //            {
//    //                m_vct_overlap[px_ovr_cnt - 1][i] = 255; //m_vct_labels[0][i];
//    //            }
//    //        }
//    //    }

//    //    // Dumps the contents of overlaping SuperImage
//    //    cout << "Overlap 0:" << m_vct_overlap[0].size() << endl;
//    //    cout << "Overlap 1:" << m_vct_overlap[1].size() << endl;
//    //    cout << "Overlap 2:" << m_vct_overlap[2].size() << endl;
//    //    cout << "Overlap 3:" << m_vct_overlap[3].size() << endl;
//    //    cout << "----" << endl;

//    //    for (int ovr_idx = 0; ovr_idx < m_vct_overlap.size(); ovr_idx++)
//    //    {
//    //        Mat overlap = Mat::zeros(m_vct_rgb_src[ovr_idx].rows, m_vct_rgb_src[ovr_idx].cols, CV_8UC3);

//    //        for (int i = 0; i < m_vct_overlap[ovr_idx].size(); i++)
//    //        {
//    //            // const int lab = m_vct_overlap[ovr_idx][i];
//    //            const int r = i / m_vct_rgb_src[ovr_idx].cols;
//    //            const int c = i - (m_vct_rgb_src[ovr_idx].cols * r);

//    //            if (m_vct_overlap[ovr_idx][i] != 0)
//    //            {
//    //                overlap.at<Vec3b>(r, c) = m_vct_rgb_src[ovr_idx].at<Vec3b>(r, c);
//    //            }
//    //        }

//    //        char val[64];
//    //        memset(val, 0, 64);
//    //        sprintf(val, "%d", ovr_idx);
//    //        string file_out = "D:/PRJ/ALGO_SLIC/overlap_";
//    //        file_out += val;
//    //        file_out += ".jpg";
//    //        imwrite(file_out, overlap);
//    //    }
//    return true;
//}

////---------------------------------------------------------------------------
//// detectOOI
////---------------------------------------------------------------------------
//bool Mosaica::detectOOI(const int simg_idx)
//{
//    // Scans the entire vector of ROI and detects the sPix included in its boundaries
//    for (int roi_idx = 0; roi_idx < m_vct_roi.size(); roi_idx++)
//    {
//        // Creates the new OOI corresponding to the ROI region
//        OOI obj_ooi;
//        m_vct_ooi.push_back(obj_ooi);

//        for (int spix_idx = 0; spix_idx < m_vct_simg[simg_idx].m_vct_spix.size(); spix_idx++)
//        {
//            // Checks if the sPix centroid is included in the ROI boudary and adds it to the OOI vector
//            int gx = m_vct_simg[0].m_vct_spix[spix_idx].m_gx;
//            int gy = m_vct_simg[0].m_vct_spix[spix_idx].m_gy;

//            if (m_vct_simg[0].m_vct_spix[spix_idx].m_min_x >= m_vct_roi[roi_idx].y &&
//                    m_vct_simg[0].m_vct_spix[spix_idx].m_min_y >= m_vct_roi[roi_idx].x &&
//                    m_vct_simg[0].m_vct_spix[spix_idx].m_max_x < m_vct_roi[roi_idx].y + m_vct_roi[roi_idx].height - 1 &&
//                    m_vct_simg[0].m_vct_spix[spix_idx].m_max_y < m_vct_roi[roi_idx].x + m_vct_roi[roi_idx].width - 1)
//            {
//                // Adds the Id of the sPix to the OOI
//                m_vct_ooi[roi_idx].m_vct_spix.push_back(spix_idx);
//                cout  << "i: " << roi_idx << " - osPix Id: " << spix_idx << " - Pos: " << gx << "," << gy << endl;
//            }
//        }
//    }
//    return true;
//}
//struct SPixDist
//{
//public:
//    int m_dist;
//    int m_spix_id;
//};

//bool pairCompare(const std::pair<int, int>& firstElem, const std::pair<int, int>& secondElem)
//{
//    return firstElem.second < secondElem.second;
//}

//bool weightCompare(const std::pair<int, int>& firstElem, const std::pair<int, int>& secondElem)
//{
//    return firstElem.second > secondElem.second;
//}

////---------------------------------------------------------------------------
//// CreateSPixPairingSet
////---------------------------------------------------------------------------
//bool Mosaica::CreateSPixPairingSet(const int simg_idx_0, const int spix_idx, SPixPairSet &vct_spix_pairs)
//{
//    int simg_idx_1 = simg_idx_0 + 1;
//    const int sz = m_vct_simg[simg_idx_1].m_vct_spix.size();
//    vector<int> vct_dist(sz);
//    vector<pair<int, int>> pairs;

//    const int min_src_size = min(m_vct_rgb_src[simg_idx_1].rows,  m_vct_rgb_src[simg_idx_1].cols);
//    const int max_dist_ray = min_src_size/4.0;

//    if (m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gx + m_vct_src_shift[simg_idx_1].x < m_vct_rgb_src[simg_idx_1].rows &
//            m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gy + m_vct_src_shift[simg_idx_1].y < m_vct_rgb_src[simg_idx_1].cols)
//    {
//        for (int k = 0; k < sz; k++)
//        {
//            //-----------------------------------------------------------------------------------------
//            // Computes the Cartesian distances
//            // Distances are computed at once
//            // Only the k shortest distances will be considered for photometric sPix Template Matching
//            //-----------------------------------------------------------------------------------------
//            const int d_x = abs((int)m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gx - (int)m_vct_simg[simg_idx_0].m_vct_spix[k].m_gx + m_vct_src_shift[simg_idx_1].x);
//            const int d_y = abs((int)m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gy - (int)m_vct_simg[simg_idx_0].m_vct_spix[k].m_gy + m_vct_src_shift[simg_idx_1].y);

//            // After registring shift vector applied, if no registration error exists, the d_x and d_y should be 0
//            const float d = sqrt(pow(d_x, 2) + pow(d_y, 2));

//            if (d <= max_dist_ray)
//            {
//                vct_dist[k] = d;
//                pair <int, int> pr(k, d);
//                pairs.push_back(pr);
//            }
//        }

//        // Sorts the dist vector to get the k shortest distances
//        sort(pairs.begin(), pairs.end(), pairCompare);
//        int dist_size = pairs.size();

//        //        for (int k = 0; k < dist_size; k++)
//        //        {
//        //            cout << "DIST: " << k << "   " << pairs[k].first << "  " << pairs[k].second << endl;
//        //            cout << "-------------------------------------" << endl;
//        //        }

//        int dist_cnt = 0;

//        for (int k = 0; k < dist_size; k++)
//        {
//            //-----------------------------------------------------------------------
//            // Computes the SPix Template Matching
//            //-----------------------------------------------------------------------
//            //        // 1. rgbSAD matching
//            //        // Interframe Photometric signature check
//            //        bool photom_match = RgbSAD(simg_idx_0, spix_idx, simg_idx_1, k);

//            //        // 2. Hue Gaussian Histogram Matching
//            //        float hue_similarity = 0.0;
//            //        SPixHueMatching(simg_idx_0, spix_idx, simg_idx_1, 0, hue_similarity);

//            // 3. Hue SAD matching
//            // The Template Matching is computed on the k shortest distances
//            bool hue_match = false;
//            float hue_sad = 0.0;
//            int paired_spix_id = pairs[k].first;
//            hue_match = SPixHueSAD(simg_idx_0, spix_idx, simg_idx_1, paired_spix_id, hue_sad);

//            if (hue_match == true)
//            {
//                dist_cnt++;
//                hue_sad = pairs[k].second;
//                // vector <SPixPair>vct1(1);
//                // vct_spix_pairs[simg_idx_0].push_back(vct1);
//                vct_spix_pairs[simg_idx_0][spix_idx].push_back(SPixPair(spix_idx, simg_idx_0, paired_spix_id, simg_idx_1, hue_sad));
//            }

//            if (dist_cnt == 6)
//            {
//                break;
//            }
//        }
//    }
//    return true;
//}

////-----------------------------------------------------------------------------

//struct SImgSPix
//{
//public :
//    SImgSPix() {}
//    SImgSPix(const int simg_id, const int spix_id)
//    {
//        m_simg_id = simg_id;
//        m_spix_id = spix_id;
//    }

//    int m_simg_id;
//    int m_spix_id;

//    // Obj Isomorphisme score
//    float m_isomorp;
//};

////---------------------------------------------------------------------------
//// ObjThreadDetection()
////---------------------------------------------------------------------------
//bool Mosaica::ObjThreadDetection(vector<vector<SPixPair>> intfrm_obj_thread)
//{
//    return true;
//}

////---------------------------------------------------------------------------
//// MotionAffinityMatch()
////---------------------------------------------------------------------------

////---------------------------------------------------------------------------
//// MOTION AFFINITY CAN BE ANALYZED ONLY IF THE ISOMORPHOLOGY OF EVERY SPIX IS RESPECTED;
//// THE MORPHOLOGY OF EACH SPIX SHOULD NOT CHANGE ACROSS FRAMES
//// frames with a different morphology should be discarded
//// OBJECT CYNEMATICS MAY BE OBTAINED FROM CASTING THE DIFFERENT SPIX SPEED CLUSTERS
////---------------------------------------------------------------------------

//bool Mosaica::MotionAffinityMatching(PairedDistanceType intfrm_paired_dist)
//{
//    // Begin of Photometric convolutional maching
//    float hue_similarity = 0.0;
//    // SPixHueMatching(0, 1, 0, 14, hue_similarity);

//    // End of Photometric convolutional matching


//    const int max_frames = 64;
//    // Lifetime of a sPix having the same speed across k consecutive interframe tables
//    vector<vector<int>> vct_spix_persistency(m_max_pairing_dist, vector<int>(max_frames));

//    vector<vector<SImgSPix>> vct_new_spix_persistency(m_max_pairing_dist);

//    vector<int> vct_motion_affinity;

//    vector<vector<int>> obj_set(m_max_pairing_dist);

//    vector<vector<SImgSPix>> obj_thread(m_max_pairing_dist);

//    int sum_dist = 0;
//    float mean_dist = 0.0;

//    int sum_persistency = 0;
//    int mean_persistency = 0;

//    for (int simg_idx_0 = 0; simg_idx_0 < m_vct_simg.size() - 1; simg_idx_0++)
//    {
//        cout << "-----------------------------------------------------------------------" << endl;
//        cout << "Interframe: " << simg_idx_0 << endl;
//        cout << "-----------------------------------------------------------------------" << endl;

//        const int simg_idx_1 = simg_idx_0 + 1;

//        for (int dist_idx = 0; dist_idx < m_max_pairing_dist; dist_idx++)
//        {
//            // Number of equidistant sPix for each distance
//            const int nbr_spix = intfrm_paired_dist[simg_idx_0][dist_idx].size();

//            if (nbr_spix > 0)
//            {
//                // cout << "-------------------------------------------------------" << endl;
//                // cout << "Distance : " << dist_idx << " Nbr sPix: " << nbr_spix << endl;
//                // cout << "-------------------------------------------------------" << endl;
//            }

//            for (int i = 0; i < nbr_spix; i++)
//            {
//                // Speed difference analysis window to match spIx having different speeds
//                int speed_tolerance = 0;
//                const int spix_id_0 = intfrm_paired_dist[simg_idx_0][dist_idx][i].m_id_1;
//                const int spix_id_1 = intfrm_paired_dist[simg_idx_1][dist_idx + speed_tolerance][i].m_id_1;

//                cout << i << "  Distance: " << dist_idx << " - (" << spix_id_0 << ", " << spix_id_1 << ")" << endl;

//                // sPix matching on two different interframe tables may be consided as belonging to a moving object
//                if (spix_id_0 == spix_id_1)
//                {
//                    vct_spix_persistency[dist_idx][spix_id_0]++;

//                    vct_new_spix_persistency[dist_idx].push_back(SImgSPix(simg_idx_0, spix_id_0));


//                    // vct_motion_affinity[dist_idx].push_back(spix_id_0);
//                }
//            }
//        }
//    }

//    // Come test per vct_spx_persistency cercho la distanza 30 e trovero le coordinate del sPix 9 che il disco rosso
//    // che si e spostato di 30 px

//    int cnt = 0;
//    int speed_idx = 0;

//    for (int spix_idx = 0; spix_idx < vct_spix_persistency[speed_idx].size(); spix_idx++)
//    {
//        if (vct_spix_persistency[speed_idx][spix_idx] > 0)
//        {
//            // sum_persistency += vct_spix_persistency[speed_idx][spix_idx];
//            cnt++;
//        }
//    }

//    int cnt2 = 0;
//    for (int speed_idx = 0; speed_idx < m_max_pairing_dist; speed_idx++)
//    {
//        sum_persistency += vct_new_spix_persistency[speed_idx].size();

//        if (vct_new_spix_persistency[speed_idx].size() > 0)
//        {
//            cnt2++;
//        }
//    }
//    mean_persistency = (float)sum_persistency / cnt2;

//    for (int spix_idx = 0; spix_idx < vct_spix_persistency[speed_idx].size(); spix_idx++)
//    {
//        // Filters sPix having the same persistance
//        if (vct_spix_persistency[speed_idx][spix_idx] >= mean_persistency)
//        {
//            obj_set[speed_idx].push_back(spix_idx);
//        }
//    }

//    for (int speed_idx = 0; speed_idx < obj_thread.size(); speed_idx++)
//    {
//        for (int i = 0; i < vct_new_spix_persistency[speed_idx].size(); i++)
//        {
//            if (vct_new_spix_persistency[speed_idx].size() >= mean_persistency)
//            {
//                const int simg_id = vct_new_spix_persistency[speed_idx][i].m_simg_id;
//                const int spix_id = vct_new_spix_persistency[speed_idx][i].m_spix_id;
//                obj_thread[speed_idx].push_back(SImgSPix(simg_id, spix_id));
//            }
//        }
//    }

//    // Temporal obj thread isomorphism analysis
//    // Computes the mean and MSE of the sPix morphology

//    SuperPixel *p_spix = 0;
//    const int spix_ref_id = 0;

//    // Mean Objesct size expressed in number of spix
//    vector<int> vct_sum_spix(64);

//    vector<int> obj_sum_size(64);
//    vector<float> obj_mean_size(64);
//    vector<float> vct_ref_obj_size_score(64);

//    vector<float> vct_spix_isomorph(m_vct_simg.size());

//    // Sums-up the number of sPix composing a single Obj thread
//    for (int speed_idx = 0; speed_idx < m_max_pairing_dist; speed_idx++)
//    {
//        cnt2 = 0;

//        obj_mean_size[speed_idx] = obj_thread[speed_idx].size();

//        for (int i = 0; i < obj_thread[speed_idx].size(); i++)
//        {
//            const int simg_id = obj_thread[speed_idx][i].m_simg_id;

//            obj_sum_size[simg_id]++;

//            if (obj_thread[speed_idx].size() > 0)
//            {
//                cnt2++;
//            }
//        }
//        obj_mean_size[speed_idx] /= cnt2;
//    }

//    // Assigns an Ob Isomorphism score to every object based on the difference to the average Obj size
//    // Computes the variance of Obj Thread size

//    // Computes and assigns the Isomorphic score
//    // Add here SCORING based on:
//    // - Frame aging
//    // - Manually selected Obj Isomorphism

//    vector<int> vct_max_isomorph(m_max_pairing_dist);
//    vector<int> best_isomorph_spix(m_max_pairing_dist);

//    for (int speed_idx = 0; speed_idx < obj_thread.size(); speed_idx++)
//    {
//        for (int i = 0; i < obj_thread[speed_idx].size(); i++)
//        {
//            const float isomorph = pow(obj_thread[speed_idx].size() - obj_mean_size[speed_idx], 2);
//            obj_thread[speed_idx][i].m_isomorp = isomorph;

//            if (isomorph > vct_max_isomorph[speed_idx])
//            {
//                vct_max_isomorph[speed_idx] = isomorph;
//                best_isomorph_spix[speed_idx] = obj_thread[speed_idx][i].m_spix_id;
//            }
//        }
//    }

//    for (int speed_idx = 0; speed_idx < obj_set.size(); speed_idx++)
//    {
//        for (int i = 0; i < obj_thread[speed_idx].size(); i++)
//        {
//            vct_sum_spix[i] = obj_thread[speed_idx][i].m_spix_id;

//            const int spix_id = obj_thread[speed_idx][i].m_spix_id;
//            // Computes the sPix morphology as the bare sum of the boundary px.
//            // This function will be replaced with the MORE

//            int bnd_size = p_spix->m_vct_boundary.size();
//            p_spix = &(m_vct_simg[i].m_vct_spix[spix_id]);

//            vector<int> bnd_sum(bnd_size);
//            float bnd_mean = 0.0;

//            // Computes the Mean
//            for (int bnd_idx = 0; bnd_idx < bnd_size; bnd_size++)
//            {
//                bnd_sum[spix_id] = p_spix->m_vct_boundary[bnd_idx].x + p_spix->m_vct_boundary[bnd_idx].y;
//            }

//            bnd_mean = bnd_sum[spix_id] / bnd_size;

//            // Computes the MSE
//            float bnd_mse = 0.0;

//            for (int simg_idx_0 = 0; simg_idx_0 < bnd_size; simg_idx_0++)
//            {
//                bnd_mse += pow(bnd_sum[spix_id], bnd_mean);
//            }

//            float aa = bnd_mse / bnd_size;
//            bnd_mse = sqrt (bnd_mse / bnd_size);
//        }
//    }

//    // A score is computed with each obj_mean_size mean.
//    // Large Obj size differences to the mean must be highly penalized
//    for (int simg_idx_0 = 0; simg_idx_0 < m_vct_simg.size() - 1; simg_idx_0++)
//    {
//        //  vct_ref_obj_size_score = obj_thread[speed_id]
//    }
//    // Every new Object is identified with a different speed.
//    // Different Objects having the same speed will be cast upon their geographical distance

//    if (obj_set.size() == 1)
//    {
//        cout << "All sPix seem to have the same dstance. No Moving Object found !! " << endl;
//    }

//    // sPix with different speed must be connected to belong to the same object

//    int max_obj_size = 0;
//    int speed_max_obj = 0;

//    for (int speed_idx = 0; speed_idx < obj_set.size(); speed_idx++)
//    {
//        if (obj_set[speed_idx].size() > max_obj_size)
//        {
//            max_obj_size = obj_set[speed_idx].size();
//            speed_max_obj = speed_idx;
//        }
//    }

//    // The set of sPix belonging to the biggest object is likely to classified as Background
//    vector<int> background_set;
//    for (int i = 0; i < obj_set[speed_max_obj].size(); i++)
//    {
//        background_set.push_back(obj_set[speed_max_obj][i]);
//    }

//    // Speed <--> Persistance = Moving Objects
//    // Candidte sPix to be Interest Objecs in motion are sPix bein close to mean_persistency

//    // Coherent Object Isomorph analysis

//    return true;




//    const int obj_size_min = 2;
//    if (vct_motion_affinity.size() < obj_size_min)
//    {
//        //    vct_motion_affinity.clear();
//    }

//    // Per essere affini i sPix devono avere una persistency simile ed una velocità simile
//    // Il bckground é comporto da sPix della stessa velocità

//    for (int i = 0; i < vct_motion_affinity.size(); i++)
//    {
//        cout << "Objet: " << vct_motion_affinity[i] << endl;
//    }
//    // The mean represetas the distance belonging to the overall image
//    // mean_dist /= sum_dist;

//    return true;
//}

////---------------------------------------------------------------------------

//bool Mosaica::RgbSAD(const int simg_idx_0, const int spix_idx_0, const int simg_idx_1, const int spix_idx_1)
//{
//    SuperPixel *p_spix_0 = &(m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0]);
//    int rgb_sum_0 = 0;

//    const int nbr_cmp_px = 16;

//    for (int px_idx = 0; px_idx < nbr_cmp_px; px_idx++)
//        //   for (int px_idx = 0; px_idx < p_spix_0->m_spix_pos.size(); px_idx++)
//    {
//        const int r = p_spix_0->m_spix_pos[px_idx].m_r;
//        const int c = p_spix_0->m_spix_pos[px_idx].m_c;
//        rgb_sum_0 += (m_vct_rgb_src[simg_idx_0].at<Vec3b>(r, c)[0] +
//                m_vct_rgb_src[simg_idx_0].at<Vec3b>(r, c)[1] +
//                m_vct_rgb_src[simg_idx_0].at<Vec3b>(r, c)[2]);
//    }
//    float rgb_mean_0 = rgb_sum_0 / nbr_cmp_px; //p_spix_0->m_spix_pos.size();

//    SuperPixel *p_spix_1 = &(m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1]);
//    int rgb_sum_1 = 0;

//    for (int px_idx = 0; px_idx < nbr_cmp_px; px_idx++)
//        // for (int px_idx = 0; px_idx < p_spix_1->m_spix_pos.size(); px_idx++)
//    {
//        const int r = p_spix_1->m_spix_pos[px_idx].m_r;
//        const int c = p_spix_1->m_spix_pos[px_idx].m_c;
//        rgb_sum_1 += (m_vct_rgb_src[simg_idx_1].at<Vec3b>(r, c)[0] +
//                m_vct_rgb_src[simg_idx_1].at<Vec3b>(r, c)[1] +
//                m_vct_rgb_src[simg_idx_1].at<Vec3b>(r, c)[2]);
//    }
//    float rgb_mean_1 = rgb_sum_1 / nbr_cmp_px; //p_spix_1->m_spix_pos.size();

//    const int epsilon = 1;

//    if (abs(rgb_mean_0 - rgb_mean_1) > epsilon)
//    {
//        return false;
//    }
//    return true;
//}


//bool resolve_graph(const int ref_spix_id, SPixPairSet pairing_graph, SPixPath &vct_spix_paths);
//bool resolve_graph_test(const int ref_spix_id, const vector<int> vct_ref_obj, vector<vector<ObjNode>> vct_spix_paths);
//bool resolve_graph_test(const int ref_spix_id, const vector<int> vct_ref_obj, vector<vector<ObjNode>> vct_spix_paths)
//{
//    vector<vector<vector<SPixPair>>> pairing_graph(4, vector<vector<SPixPair>>(4));

//    SPixPair arc;

//    // Level 0
//    arc.m_simg_id_0 = 0;
//    arc.m_simg_id_1 = 1;
//    arc.m_id_0 = 0;
//    arc.m_id_1 = 0;
//    arc.m_dist = 38;
//    pairing_graph[0][0].push_back(arc);

//    arc.m_simg_id_0 = 0;
//    arc.m_simg_id_1 = 1;
//    arc.m_id_0 = 0;
//    arc.m_id_1 = 1;
//    arc.m_dist = 35;
//    pairing_graph[0][0].push_back(arc);

//    arc.m_simg_id_0 = 0;
//    arc.m_simg_id_1 = 1;
//    arc.m_id_0 = 0;
//    arc.m_id_1 = 2;
//    arc.m_dist = 30;
//    pairing_graph[0][0].push_back(arc);

//    arc.m_simg_id_0 = 1;
//    arc.m_simg_id_1 = 2;
//    arc.m_id_0 = 1;
//    arc.m_id_1 = 0;
//    arc.m_dist = 20;
//    pairing_graph[1][1].push_back(arc);

//    arc.m_simg_id_0 = 1;
//    arc.m_simg_id_1 = 2;
//    arc.m_id_0 = 2;
//    arc.m_id_1 = 1;
//    arc.m_dist = 18;
//    pairing_graph[1][2].push_back(arc);

//    arc.m_simg_id_0 = 1;
//    arc.m_simg_id_1 = 2;
//    arc.m_id_0 = 2;
//    arc.m_id_1 = 2;
//    arc.m_dist = 12;
//    pairing_graph[1][2].push_back(arc);

//    arc.m_simg_id_0 = 2;
//    arc.m_simg_id_1 = 3;
//    arc.m_id_0 = 1;
//    arc.m_id_1 = 0;
//    arc.m_dist = 6;
//    pairing_graph[2][1].push_back(arc);

//    arc.m_simg_id_0 = 2;
//    arc.m_simg_id_1 = 3;
//    arc.m_id_0 = 1;
//    arc.m_id_1 = 1;
//    arc.m_dist = 4;
//    pairing_graph[2][1].push_back(arc);

//    arc.m_simg_id_0 = 2;
//    arc.m_simg_id_1 = 3;
//    arc.m_id_0 = 2;
//    arc.m_id_1 = 2;
//    arc.m_dist = 2;
//    pairing_graph[2][2].push_back(arc);

//    arc.m_simg_id_0 = 3;
//    arc.m_simg_id_1 = 4;
//    arc.m_id_0 = 0;
//    arc.m_id_1 = 0;
//    arc.m_dist = 7;
//    pairing_graph[3][0].push_back(arc);

//    arc.m_simg_id_0 = 3;
//    arc.m_simg_id_1 = 4;
//    arc.m_id_0 = 2;
//    arc.m_id_1 = 1;
//    arc.m_dist = 15;
//    pairing_graph[3][2].push_back(arc);

//    //    // Level 0
//    //    arc.m_simg_id_0 = 0;
//    //    arc.m_simg_id_1 = 1;
//    //    arc.m_id_0 = 0;
//    //    arc.m_id_1 = 0;
//    //    arc.m_dist = 10;
//    //    pairing_graph[0][0].push_back(arc);

//    //    arc.m_simg_id_0 = 0;
//    //    arc.m_simg_id_1 = 1;
//    //    arc.m_id_0 = 0;
//    //    arc.m_id_1 = 1;
//    //    arc.m_dist = 11;
//    //    pairing_graph[0][0].push_back(arc);

//    //    arc.m_simg_id_0 = 0;
//    //    arc.m_simg_id_1 = 1;
//    //    arc.m_id_0 = 0;
//    //    arc.m_id_1 = 2;
//    //    arc.m_dist = 12;
//    //    pairing_graph[0][0].push_back(arc);


//    //    // Level 1
//    //    arc.m_simg_id_0 = 1;
//    //    arc.m_simg_id_1 = 2;
//    //    arc.m_id_0 = 0;
//    //    arc.m_id_1 = 0;
//    //    arc.m_dist = 20;
//    //    pairing_graph[1][0].push_back(arc);

//    //    arc.m_simg_id_0 = 1;
//    //    arc.m_simg_id_1 = 2;
//    //    arc.m_id_0 = 0;
//    //    arc.m_id_1 = 2;
//    //    arc.m_dist = 21;
//    //    pairing_graph[1][0].push_back(arc);

//    //    arc.m_simg_id_0 = 1;
//    //    arc.m_simg_id_1 = 2;
//    //    arc.m_id_0 = 1;
//    //    arc.m_id_1 = 4;
//    //    arc.m_dist = 23;
//    //    pairing_graph[1][1].push_back(arc);

//    //    arc.m_simg_id_0 = 1;
//    //    arc.m_simg_id_1 = 2;
//    //    arc.m_id_0 = 2;
//    //    arc.m_id_1 = 3;
//    //    arc.m_dist = 24;
//    //    pairing_graph[1][2].push_back(arc);

//    //    // Level 2
//    //    arc.m_simg_id_0 = 2;
//    //    arc.m_simg_id_1 = 3;
//    //    arc.m_id_0 = 2;
//    //    arc.m_id_1 = 1;
//    //    arc.m_dist = 30;
//    //    pairing_graph[2][2].push_back(arc);

//    //    arc.m_simg_id_0 = 2;
//    //    arc.m_simg_id_1 = 3;
//    //    arc.m_id_0 = 2;
//    //    arc.m_id_1 = 3;
//    //    arc.m_dist = 31;
//    //    pairing_graph[2][2].push_back(arc);

//    //    arc.m_simg_id_0 = 2;
//    //    arc.m_simg_id_1 = 3;
//    //    arc.m_id_0 = 3;
//    //    arc.m_id_1 = 0;
//    //    arc.m_dist = 32;
//    //    pairing_graph[2][3].push_back(arc);

//    resolve_graph(ref_spix_id, pairing_graph, vct_spix_paths);
//    return true;
//}

//bool resolve_graph(const int spix_id_0, SPixPairSet pairing_graph, SPixPath &vct_spix_paths)
//{

//    // vct_spix_paths is a vector of paths
//    // Every path is a set of nodes from the root to the the selected leaf
//    stack <ObjNode> stack_spix;

//    ObjNode nd;
//    nd.simg_id = 0;
//    nd.spix_id = spix_id_0;
//    nd.weight = 0;

//    stack_spix.push(nd);
//    float path_weight = 0.0;
//    float weight_sum = 0.0;

//    ObjNode cur_spix;

//    vector<ObjNode> obj_nd;
//    vct_spix_paths.push_back(obj_nd);
//    int path_id = 0;
//    bool is_leaf = false;

//    while (!stack_spix.empty())
//    {
//        // Searches all the sPix_id of the the selected Obj in the list of the first simg
//        // and recursively follows thir paths and builds the Object thread trees
//        cur_spix = stack_spix.top();
//        stack_spix.pop();
//        weight_sum += cur_spix.weight;

//        if (is_leaf == true)
//        {
//            is_leaf = false;
//            vct_spix_paths.push_back(vector<ObjNode>(0));
//            path_id++;

//            // Replicates the k-1 sub tree up to the current node
//            for (int i = 0; i < cur_spix.simg_id; i++)
//            {
//                vct_spix_paths[path_id].push_back(vct_spix_paths[path_id - 1][i]);
//            }
//        }

//        // Updates the cumulative weight
//        cur_spix.weight_sum += weight_sum;
//        path_weight += cur_spix.weight;
//        vct_spix_paths[path_id].push_back(cur_spix);

//        int simg_idx_0 = cur_spix.simg_id;
//        int cnt_sons = 0;

//        // Explores all nodes at the same level k
//        if (simg_idx_0 < pairing_graph.size() && cur_spix.spix_id < pairing_graph[simg_idx_0].size())
//        {
//            for (int j = 0; j < pairing_graph[simg_idx_0][cur_spix.spix_id].size(); j++)
//            {
//                nd.simg_id = pairing_graph[simg_idx_0][cur_spix.spix_id][j].m_simg_id_1;
//                nd.spix_id = pairing_graph[simg_idx_0][cur_spix.spix_id][j].m_id_1;
//                nd.weight = pairing_graph[simg_idx_0][cur_spix.spix_id][j].m_dist;

//                stack_spix.push(nd);
//                cnt_sons++;
//            }
//        }

//        // Checks if the node is a leaf
//        // A node is leaf when it is not found in the pairing_graph at the level simg_k
//        if (cnt_sons == 0)
//        {
//            weight_sum = 0.0;
//            is_leaf = true;
//        }
//    }
//    return true;
//}

////---------------------------------------------------------------------------
//// spixOverlapMatch
////---------------------------------------------------------------------------
//bool Mosaica::spixOverlapMatch()
//{
//    float hue_similarity = 0.0;
//    // SPixHueMatching(0, 1, 0, 0, hue_similarity);

//    // float mean_ air_dist = 0.0;

//    const int nbr_spix = m_vct_simg[0].m_vct_spix.size();
//    float *dist_min = new float[nbr_spix];

//    // Creates the vector of inter-frame sPix distances
//    vector<vector<vector<SPixPair>>> vct_spix_pairs(m_vct_simg.size(), vector<vector<SPixPair>>(64));
//    m_max_pairing_dist = 32;

//    vector<vector<vector<SuperPixelPair>>> intfrm_paired_dist(m_vct_simg.size() - 1, vector<vector<SuperPixelPair>>(64)); //, vector<vector<SuperPixelPair>>); //m_vct_simg.size() - 1, vector<vector<int>>(m_max_pairing_dist, vector<int>(max_nbr_spix)));

//    const int nbr_ref_obj = 2; // 2
//    vector<vector<int>> vct_ref_obj(nbr_ref_obj);

//    // Obj 0
//    vct_ref_obj[0].push_back(14); // 34);
//    vct_ref_obj[0].push_back(15);

//    // Obj 1
//    //   vct_ref_obj[1].push_back(23);
//    //    vct_ref_obj[1].push_back(24);
//    //   vct_ref_obj[1].push_back(25);

//    // vector<vector<SPixPair>> intfrm_obj_thread(m_vct_simg.size() - 1); //, vector<vector<SuperPixelPair>vct_ref_obj.size()>); //m_vct_simg.size() - 1, vector<vector<int>>(m_max_pairing_dist, vector<int>(max_nbr_spix)));

//    // SPix Affinity Graph
//    // vector<vector<vector<SPixPair>>> pairing_graph(m_vct_simg.size() - 1); //, vector<vector<SuperPixelPair>vct_ref_obj.size()>); //m_vct_simg.size() - 1, vector<vector<int>>(m_max_pairing_dist, vector<int>(max_nbr_spix)));

//    vector<SPixPath> vct_propagation_paths;
//    // vector<ObjectThread> vct_new_obj_thread;

//    for (int obj_idx = 0; obj_idx < vct_ref_obj.size(); obj_idx++)
//    {
//        for (int i = 0; i < vct_ref_obj[obj_idx].size(); i++)
//        {
//            // Loops the entire list of sPix of the selected sImage
//            for (int simg_idx = 0; simg_idx < m_vct_simg.size() - 1; simg_idx++)
//            {
//                const int simg_idx_0 = simg_idx;
//                // const int spix_size = m_vct_simg[simg_idx_0].m_vct_spix.size();
//                // PairedSuperPixel paired_spix;

//                if (simg_idx == 0)
//                {

//                    // Computes set of cross-frame minimal cartesian distances among sPix and stores them into a distance table
//                    CreateSPixPairingSet(simg_idx_0, vct_ref_obj[obj_idx][i], vct_spix_pairs);
//                }
//                else
//                {
//                    // Computes set of cross-frame minimal cartesian distances among sPix and stores them into a distance table
//                    //  const int spix_idx_0 = vct_spix_pairs[simg_idx - 1][0][0].m_id_0; // vct_ref_obj[obj_idx][i];

//                    for (int k = 0; k < vct_spix_pairs[simg_idx - 1].size(); k++)
//                    {
//                        // Loops all distance matching occurrencies
//                        for (int j = 0; j < vct_spix_pairs[simg_idx - 1][k].size(); j++)
//                        {
//                            const int spix_idx_0 = vct_ref_obj[obj_idx][i];
//                            // const int spix_idx_0 = vct_spix_pairs[simg_idx - 1][k][j].m_id_1; // vct_ref_obj[obj_idx][i];
//                            CreateSPixPairingSet(simg_idx_0, spix_idx_0, vct_spix_pairs);
//                        }
//                    }
//                }
//            }
//            const int ref_spix_id = vct_ref_obj[obj_idx][i];
//            SPixPath spix_path;

//            // Resolves the Propagation Tree and generates the serialized paths
//            resolve_graph(ref_spix_id, vct_spix_pairs, spix_path);
//            vct_propagation_paths.push_back(spix_path);
//        }
//    }

//    //---------------------------------------------------------------------
//    // Dumps the serialized Propagation Paths
//    //---------------------------------------------------------------------

//    vector<ObjPath> obj_path(vct_ref_obj.size());
//    obj_path[0].m_vct_pair = vct_propagation_paths[0][0];
//    //  obj_path[1].m_vct_pair = vct_propagation_paths[1][0];

//    dumpPropagationTree(this, obj_path, 0);
//    dumpPropagationTree(this, obj_path, 1);
//    dumpPropagationTree(this, obj_path, 2);
//    dumpPropagationTree(this, obj_path, 3);

//    //    //---------------------------------------------------------------------
//    //    // Multi Path Emulator
//    //    //---------------------------------------------------------------------
//    //    vct_propagation_paths.clear();
//    //    SPixPath sp;
//    //    vector<ObjNode> vct_nd;
//    //    ObjNode nd;

//    //    //-------------------------------
//    //    // Spix 0
//    //    //-------------------------------
//    //    // Path 0
//    //    nd.simg_id = 0;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 1;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 1;
//    //    nd.spix_id = 0;
//    //    nd.weight = 2;
//    //    nd.weight_sum = 3;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 2;
//    //    nd.spix_id = 0;
//    //    nd.weight = 3;
//    //    nd.weight_sum = 6;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 2;
//    //    nd.spix_id = 0;
//    //    nd.weight = 4;
//    //    nd.weight_sum = 10;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 3;
//    //    nd.spix_id = 0;
//    //    nd.weight = 3;
//    //    nd.weight_sum = 6;
//    //    vct_nd.push_back(nd);

//    //    sp.push_back(vct_nd);
//    //    vct_nd.clear();

//    //    // Path 1
//    //    nd.simg_id = 0;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 1;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 1;
//    //    nd.spix_id = 1;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 2;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 2;
//    //    nd.spix_id = 0;
//    //    nd.weight_sum = 3;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 3;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 4;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 4;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 5;
//    //    vct_nd.push_back(nd);

//    //    sp.push_back(vct_nd);
//    //    vct_nd.clear();

//    //    // Path 2
//    //    nd.simg_id = 0;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 1;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 1;
//    //    nd.spix_id = 2;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 2;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 2;
//    //    nd.spix_id = 2;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 3;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 3;
//    //    nd.spix_id = 3;
//    //    nd.weight = 12;
//    //    nd.weight_sum = 15;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 4;
//    //    nd.spix_id = 4;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 16;
//    //    vct_nd.push_back(nd);

//    //    sp.push_back(vct_nd);
//    //    vct_propagation_paths.push_back(sp);
//    //    sp.clear();
//    //    vct_nd.clear();

//    //    //-------------------------------
//    //    // Spix 1
//    //    //-------------------------------
//    //    // Path 0
//    //    nd.simg_id = 0;
//    //    nd.spix_id = 0;
//    //    nd.weight = 3;
//    //    nd.weight_sum = 3;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 1;
//    //    nd.spix_id = 0;
//    //    nd.weight = 2;
//    //    nd.weight_sum = 3;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 2;
//    //    nd.spix_id = 0;
//    //    nd.weight = 5;
//    //    nd.weight_sum = 8;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 3;
//    //    nd.spix_id = 0;
//    //    nd.weight = 7;
//    //    nd.weight_sum = 15;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 4;
//    //    nd.spix_id = 0;
//    //    nd.weight = 3;
//    //    nd.weight_sum = 6;
//    //    vct_nd.push_back(nd);

//    //    sp.push_back(vct_nd);
//    //    vct_nd.clear();

//    //    // Path 1
//    //    nd.simg_id = 0;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 1;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 1;
//    //    nd.spix_id = 1;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 2;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 2;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 3;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 3;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 4;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 4;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 5;
//    //    vct_nd.push_back(nd);

//    //    sp.push_back(vct_nd);
//    //    vct_nd.clear();

//    //    // Path 2
//    //    nd.simg_id = 0;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 1;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 1;
//    //    nd.spix_id = 2;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 2;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 2;
//    //    nd.spix_id = 2;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 3;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 3;
//    //    nd.spix_id = 3;
//    //    nd.weight = 12;
//    //    nd.weight_sum = 15;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 4;
//    //    nd.spix_id = 4;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 16;
//    //    vct_nd.push_back(nd);

//    //    sp.push_back(vct_nd);
//    //    vct_propagation_paths.push_back(sp);
//    //    sp.clear();
//    //    vct_nd.clear();

//    //    //-------------------------------
//    //    // Spix 2
//    //    //-------------------------------
//    //    // Path 0
//    //    nd.simg_id = 0;
//    //    nd.spix_id = 0;
//    //    nd.weight = 3;
//    //    nd.weight_sum = 3;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 1;
//    //    nd.spix_id = 0;
//    //    nd.weight = 2;
//    //    nd.weight_sum = 3;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 2;
//    //    nd.spix_id = 0;
//    //    nd.weight = 5;
//    //    nd.weight_sum = 8;
//    //    vct_nd.push_back(nd);

//    ////    nd.simg_id = 2;
//    ////    nd.spix_id = 0;
//    ////    nd.weight = 7;
//    ////    nd.weight_sum = 15;
//    ////    vct_nd.push_back(nd);

//    ////    nd.simg_id = 3;
//    ////    nd.spix_id = 0;
//    ////    nd.weight = 3;
//    ////    nd.weight_sum = 6;
//    ////    vct_nd.push_back(nd);

//    //    sp.push_back(vct_nd);
//    //    vct_nd.clear();

//    //    // Path 1
//    //    nd.simg_id = 0;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 1;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 1;
//    //    nd.spix_id = 1;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 2;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 2;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 3;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 3;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 4;
//    //    vct_nd.push_back(nd);

//    ////    nd.simg_id = 4;
//    ////    nd.spix_id = 0;
//    ////    nd.weight = 1;
//    ////    nd.weight_sum = 5;
//    ////    vct_nd.push_back(nd);

//    //    sp.push_back(vct_nd);
//    //    vct_nd.clear();

//    //    // Path 2
//    //    nd.simg_id = 0;
//    //    nd.spix_id = 0;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 1;
//    //    vct_nd.push_back(nd);

//    //    nd.simg_id = 1;
//    //    nd.spix_id = 2;
//    //    nd.weight = 1;
//    //    nd.weight_sum = 2;
//    //    vct_nd.push_back(nd);

//    ////    nd.simg_id = 2;
//    ////    nd.spix_id = 2;
//    ////    nd.weight = 1;
//    ////    nd.weight_sum = 3;
//    ////    vct_nd.push_back(nd);

//    ////    nd.simg_id = 3;
//    ////    nd.spix_id = 3;
//    ////    nd.weight = 12;
//    ////    nd.weight_sum = 15;
//    ////    vct_nd.push_back(nd);

//    ////    nd.simg_id = 4;
//    ////    nd.spix_id = 4;
//    ////    nd.weight = 1;
//    ////    nd.weight_sum = 16;
//    ////    vct_nd.push_back(nd);

//    //    sp.push_back(vct_nd);
//    //    vct_propagation_paths.push_back(sp);
//    //    sp.clear();
//    //    vct_nd.clear();

//    //---------------------------------------------------------------------
//    // Path affinity estimation
//    //---------------------------------------------------------------------
//    // Metrics : Length, weigth, shared nodes among paths
//    //---------------------------------------------------------------------
//    // 1. SPix based parameter: Path Spread (number of leafs for every root spix)
//    // 2. Path length
//    // 3. Path based parameter: Path scatterness (path criss-crossing)
//    // 4. Path based parameter: Photometric weight
//    // The weight of the last node contains the sum of all weights of the entire path
//    // 5. Path based parameter: sPix size (in px)
//    //---------------------------------------------------------------------

//    vector<int> vct_obj_thread_length(vct_propagation_paths.size());
//    vector<int> vct_selected_path(vct_propagation_paths.size());

//    // Weight components
//    for (int i = 0; i < vct_propagation_paths.size(); i++)
//    {
//        const int tree_spread = vct_propagation_paths[i].size();
//        const float tree_spread_weight = 1.0 / tree_spread;

//        // All paths must be explored prior to determine any path length
//        int path_length_sum = 0;

//        // This map is used to mesure the path scatterness
//        // The scatterness is the number of different spix_id used in a path
//        map<int,int> vct_unique_spix;
//        vector<float>vct_spix_scatterness(tree_spread);
//        vector<float>vct_path_photometric_sum(tree_spread);
//        vector<float>vct_path_length(tree_spread);

//        int path_length  = 0;
//        float path_photometric_sum = 0.0;
//        float path_scatterness_sum = 0.0;

//        for (int j = 0; j < tree_spread; j++)
//        {
//            vct_unique_spix.clear();
//            path_length = vct_propagation_paths[i][j].size();
//            path_length_sum += path_length;

//            const int max_idx = vct_propagation_paths[i][j].size() - 1;

//            // Sums-up the weight_sums along the same path
//            path_photometric_sum += vct_propagation_paths[i][j][max_idx].weight_sum;
//            vct_path_photometric_sum[j] = vct_propagation_paths[i][j][max_idx].weight_sum;
//            vct_path_length[j] = path_length;

//            for (int k = 0; k < path_length; k++)
//            {
//                const int idx = vct_propagation_paths[i][j][k].spix_id;
//                vct_unique_spix[idx]++;
//            }
//            vct_spix_scatterness[j] = vct_unique_spix.size();
//            path_scatterness_sum += vct_unique_spix.size();
//        }
//        const float path_length_mean = path_length_sum / tree_spread;
//        const float path_photometric_mean = path_photometric_sum / tree_spread;
//        const float path_scatterness_mean = path_scatterness_sum / tree_spread;

//        // Propagation Path Granularity vector definitions
//        vector<float>vct_size_error(path_length);

//        // Propagation Tree Granularity vector definitions
//        vector<float>vct_length_error(tree_spread);
//        vector<float>vct_photometric_error(tree_spread);
//        vector<float>vct_scatterness_error(tree_spread);

//        // Stores the size variance for every spix in each path
//        vector<float> vct_path_size_mean(tree_spread);
//        vector<float> vct_size_variance(tree_spread);

//        for (int j = 0; j < tree_spread; j++)
//        {
//            vct_length_error[j] = pow(vct_propagation_paths[i][j].size() - path_length_mean, 2);

//            const int max_idx = vct_propagation_paths[i][j].size() - 1;
//            vct_photometric_error[j] = pow(vct_propagation_paths[i][j][max_idx].weight_sum - path_photometric_mean, 2);
//            vct_scatterness_error[j] = pow(vct_spix_scatterness[j] - path_scatterness_mean, 2);

//            int spix_size_sum = 0;

//            // Computes the mean of te size of a single path
//            for (int k = 0; k < path_length; k++)
//            {
//                const int idx = vct_propagation_paths[i][j][k].spix_id;
//                const int spix_size = m_vct_simg[k].m_vct_spix[idx].m_spix_pos.size();
//                spix_size_sum += spix_size;
//            }

//            vct_path_size_mean[j] = spix_size_sum / path_length;

//            for (int k = 0; k < path_length; k++)
//            {
//                const int idx = vct_propagation_paths[i][j][k].spix_id;
//                const int spix_size = m_vct_simg[k].m_vct_spix[idx].m_spix_pos.size();
//                float size_error = abs(spix_size - vct_path_size_mean[j]);
//                vct_size_error[k] = size_error;
//            }
//            float size_error_sum = 0.0;

//            for (int k = 0; k < path_length; k++)
//            {
//                size_error_sum += vct_size_error[k];
//            }
//            vct_size_variance[j] = size_error_sum / path_length;
//        }

//        vector<float> vct_size_weight;
//        vector<float> vct_photometric_weight;
//        vector<float> vct_length_weight;
//        vector<float> vct_scatterness_weight;

//        normalize(vct_path_size_mean, vct_size_weight, 0.0, 1.0, NORM_MINMAX, CV_32FC1);
//        normalize(vct_path_photometric_sum, vct_photometric_weight, 0.0, 1.0, NORM_MINMAX, CV_32FC1);
//        normalize(vct_path_length, vct_length_weight, 0.0, 1.0, NORM_MINMAX, CV_32FC1);
//        normalize(vct_spix_scatterness, vct_scatterness_weight, 0.0, 1.0, NORM_MINMAX, CV_32FC1);

//        // Weight vector stores the pairs <Id, Weight> of the vct_propagation_paths sorted with the highest weight
//        vector<pair<int, float>> weight(tree_spread);

//        vector<float> vct_photometric_variance;
//        vector<float> vct_length_variance;
//        vector<float> vct_scatterness_variance;

//        normalize(vct_size_error, vct_size_variance, 0.1, 1.0, NORM_MINMAX, CV_32FC1);
//        normalize(vct_photometric_error, vct_photometric_variance, 0.1, 1.0, NORM_MINMAX, CV_32FC1);
//        normalize(vct_length_error, vct_length_variance, 0.1, 1.0, NORM_MINMAX, CV_32FC1);
//        normalize(vct_scatterness_error, vct_scatterness_variance, 0.0, 1.0, NORM_MINMAX, CV_32FC1);

//        //-----------------------------------------------------------
//        // Propagation Path selection policy
//        //-----------------------------------------------------------
//        // Longer <--> Photometric fidelity
//        // PolicyWeight = Longer / Photo Fidelity
//        // PilicyWeight high = High PhotoFidelity
//        // PolicyWeight small = Maximise path length
//        //-----------------------------------------------------------
//        float policy_weight = 1.0;

//        // Values with the highet variance should be higly penalized
//        for (int j = 0; j < tree_spread; j++)
//        {
//            // float w_pht = vct_photometric_weight[j] / vct_photometric_variance[j];
//            // float w_lng = vct_length_weight[j] / vct_length_variance[j];
//            // float w_sct = vct_scatterness_weight[j] / vct_scatterness_variance[j];

//            // Photometric Weight/Photometric Error: Additive parameter
//            // Length Weight/Lenght Error: Additive parameter
//            // Scatterness Weight/Scatterness Error: Subtractive parameter
//            // SPix Path mean size : Not used
//            // SPix Path mean error : Subtractive

//            pair<int, float> nd;
//            nd.first = j;
//            nd.second = ((vct_photometric_weight[j] / vct_photometric_variance[j]) * policy_weight)
//                    + ((vct_length_weight[j] / vct_length_variance[j]) / policy_weight)
//                    - (vct_scatterness_weight[j] / vct_scatterness_variance[j])
//                    - (vct_size_variance[j]);
//            weight[j] = nd;

//            cout << "--------------------------------------------------" << endl;
//            cout << nd.first << " - Photometric <Mean, Error, Weight>" << vct_photometric_weight[j] << " " << vct_photometric_variance[j] << " " << (float)nd.second << endl;
//        }
//        sort(weight.begin(), weight.end(), weightCompare);

//        const int selected_idx = weight[0].first;
//        vct_selected_path[i] = selected_idx;

//        // Keeps the length of the selected path
//        vct_obj_thread_length[i] = vct_path_length[selected_idx];
//    }

//    //---------------------------------
//    // Persistency analysis
//    //---------------------------------

//    // Computes the mean Obj thread length
//    // A higher confidence will be given to the paths closer to the mean
//    float obj_length_mean = 0.0;

//    for (int i = 0; i < vct_obj_thread_length.size(); i++)
//    {
//        obj_length_mean += vct_obj_thread_length[i];
//    }
//    obj_length_mean /= vct_obj_thread_length.size();

//    // Computes the Obj Thread variance of the length
//    float obj_length_variance = 0.0;
//    vector<float> vct_obj_length_error(vct_propagation_paths.size());
//    for (int i = 0; i < vct_obj_thread_length.size(); i++)
//    {
//        vct_obj_length_error[i] = pow(vct_obj_thread_length[i] - obj_length_mean, 2);
//        obj_length_variance += vct_obj_length_error[i];
//    }

//    normalize(vct_obj_length_error, vct_obj_length_error, 0.0, 1.0, NORM_MINMAX, CV_32FC1);

//    obj_length_variance /= vct_obj_thread_length.size();
//    obj_length_variance = sqrt(obj_length_variance);

//    // The confidence of each spix depends from the level of path spreadness,
//    // if the spix generated has a too wide propagation tree, it may have to be discarded

//    // ObjThread granularity: <RefObj><RefSPix><PathId>
//    vector<ObjThread> vct_obj_thread;
//    ObjThread obj_thd;

//    // Adds the selected path of each sPix belonging to the selected Object to the Obj thread
//    for (int i = 0; i < vct_selected_path.size(); i++)
//    {
//        const int path_idx = vct_selected_path[i];

//        ObjPath obj_path;
//        vector<ObjNode> obj_nod(vct_propagation_paths[i][path_idx].size());
//        obj_nod = vct_propagation_paths[i][path_idx];
//        obj_path.m_vct_pair = obj_nod;

//        // Remplacer avec le Weight !!!!
//        obj_path.m_weight = 0;
//        obj_thd.m_vct_path.push_back(obj_path);
//    }

//    obj_thd.m_aggregation_confidence = 1.0 / (1 + obj_length_variance);

//    // vct_obj_thread contains a vector of sPix for every Ref. sPix
//    vct_obj_thread.push_back(obj_thd);

//    dumpPropagationTree(this, obj_thd.m_vct_path, 0);

//    return true;


//    //    for (int spix_idx = 0; spix_idx < spix_size; spix_idx++)
//    //    {
//    //        // Computes set of cross-frame minimal cartesian distances among sPix and stores them into a distance table
//    //        if (CreateSPixPairingSet(simg_idx_0, spix_idx, vct_spix_pairs) == false)
//    //        {
//    //            return false;
//    //        }

//    //        // const int paired_spix_id = vct_spix_pairs[simg_idx_0][spix_idx].m_spix_id;
//    //        //  const float paired_dist = vct_spix_pairs[simg_idx_0][spix_idx][0].m_dist;

//    //        // const int paired_dist_id = abs(spix_idx - paired_spix_id);
//    //        // float mean_paired_dist = (float)paired_dist / m_vct_simg[simg_idx_0].m_vct_spix.size();

//    //        bool enable_vector_dump = false;
//    //        // Scans all sPix belonging to the first sImage
//    //        if (enable_vector_dump == true)
//    //        {
//    //            SPixTools spt;
//    //            // spt.dumpSPixDistanceVectors(m_vct_rgb_src[0], *this, simg_idx_0, spix_idx, vct_spix_pairs);
//    //        }

//    //        float obj_match_val = 0.0;

//    //        // Creates the SAG graph
//    //        // The vct_spix_pairs contains the entire set of crossframe temporal arcs
//    //        // The manual selection is applied at this level and only the manually selected ROI sPix
//    //        // will be copied to the the pairing_graph
//    //        for (int i = 0; i < vct_spix_pairs.size(); i++)
//    //        {
//    //            const int spix_pair_id = vct_spix_pairs[simg_idx_0][spix_idx][i].m_spix_id;
//    //            const int spix_pair_affin = vct_spix_pairs[simg_idx_0][spix_idx][i].m_dist;
//    //            const int simg_idx_1 = simg_idx_0 + 1;
//    //            SPixPair spix_pair(spix_idx, simg_idx_0, spix_pair_id, simg_idx_1, (int)spix_pair_affin);
//    //            pairing_graph[simg_idx_0][spix_idx].push_back(spix_pair);

//    //            //-------------------------------------------------------------------------------------
//    // Photometric matching test between the current sPix and the Reference Object pattern
//    //-------------------------------------------------------------------------------------
//    //                if (simg_idx_0 == 0)
//    //                {
//    //                    //   ref_obj_match = RgbSAD(0, vct_ref_obj[0], simg_idx_0, spix_idx);
//    //                    //    ref_obj_match = SPixHueSAD(0, vct_ref_obj[0], simg_idx_0, spix_idx, obj_match_val);
//    //                }
//    //                else
//    //                {
//    //                    const int simg_idx_1 = simg_idx_0;
//    //                    //   const int spix_idx_1 = intfrm_obj_thread[simg_idx_0 - 1][0].m_id_1;
//    //                    //  ref_obj_match = RgbSAD(simg_idx_0, spix_idx, simg_idx_1, spix_idx_1);
//    //                    //   ref_obj_match = SPixHueSAD(img_idx_0, spix_idx, simg_idx_1, spix_idx_1, obj_match_val);
//    //                }

//    //                if (ref_obj_match == true)
//    //                {
//    //                    // SuperPixelPair spix_pair(spix_idx, vct_spix_pairs[simg_idx_0][spix_idx][0].m_spix_id);
//    //                    // intfrm_paired_dist[simg_idx_0][(int)paired_dist].push_back(spix_pair);
//    //                    //  SPixPair spix_pair(spix_idx, vct_spix_pairs[simg_idx_0][spix_idx][i].m_spix_id, (int)paired_dist);
//    //                    //   intfrm_obj_thread[simg_idx_0].push_back(spix_pair);
//    //                }
//    ////      }
//}
////}

//// vector<ObjectThread> vct_new_obj_thread;

//// Graph resolution and creation of of Object Thread Tree

////-----------------------------------------------------------
//// PROXIMITY / PERSISTENCY model
//// Proximity : Neighbooring affinities
//// Crossframe temporal persistency of the spix set
//// Try to use PRIOR KNOWLEDGE
//// Try to better detail how SPIx Labelling is performed and the proper metrics for proximity
//// MARGINAL (componentwise) strategy : Every SPix is analyzed independently
//// A Marginal Tree is built for every SPix and I get several correlated paths
//// The interaction among SPix is analyzed after to better discriminate the different Paths
//// The TREE represents the time SPix propagation
//// PREVOIR 2D Histogram Hue-Distances ou Shape
////-----------------------------------------------------------


////for (int obj_idx = 0; obj_idx < vct_ref_obj.size(); obj_idx++)
////{
////    for (int i = 0; i < vct_ref_obj[obj_idx].size(); i++)
////    {
////        int ref_spix_id = 0; //vct_ref_obj[obj_idx][i];
////        resolve_graph(ref_spix_id, pairing_graph, vct_ref_obj[0], vct_new_obj_thread);
////    }
////}

////int size_min = INT_MAX;
////int size_max = 0;
////int size_sum = 0;
////int cnt = 0;

//// Computes the mean of the size of the Obj
////for (int simg_idx = 0; simg_idx < m_vct_simg.size() - 1; simg_idx++)
////{
////    if (intfrm_obj_thread[simg_idx].size() > 0)
////    {
////        if (intfrm_obj_thread[simg_idx].size() <= size_min)
////        {
////            size_min = intfrm_obj_thread[simg_idx].size();
////        }

////        if (intfrm_obj_thread[simg_idx].size() > size_max)
////        {
////            size_max = intfrm_obj_thread[simg_idx].size();
////        }
////        size_sum += intfrm_obj_thread[simg_idx].size();
////        cnt++;
////    }

////}

////float size_mean = 0.0;
////if (cnt > 0)
////{
////    size_mean = size_sum / cnt;
////}

//// Computes the mean of the Object size expressed as number of px for each sPix
////vector<int> sum_obj_size(intfrm_obj_thread.size());
////float mean_obj_size = 0.0;

////for (int simg_idx = 0; simg_idx < intfrm_obj_thread.size(); simg_idx++)
////{
////    for (int spix_idx = 0; spix_idx < intfrm_obj_thread[simg_idx].size(); spix_idx++)
////    {
////        const int sz = m_vct_simg[simg_idx].m_vct_spix[spix_idx].m_spix_pos.size();
////        sum_obj_size[simg_idx] += sz;
////    }
////}

////for (int simg_idx = 0; simg_idx < m_vct_simg.size() - 1; simg_idx++)
////{
////    mean_obj_size += sum_obj_size[simg_idx];
////}

////mean_obj_size /= intfrm_obj_thread.size();

////float variance_obj_size = 0.0;

////// Computes the Variance of the Object Size
////for (int simg_idx = 0; simg_idx < intfrm_obj_thread.size(); simg_idx++)
////{
////    variance_obj_size += pow(sum_obj_size[simg_idx] - mean_obj_size, 2);
////}

////variance_obj_size /= intfrm_obj_thread.size();
////variance_obj_size = sqrt(variance_obj_size);

////vector<int> score_obj_size(sum_obj_size.size());
////int max_size_obj_id = 0;
////int best_obj_id_2 = 0;

////// The highest shift to the variance_obj_size is considered as the Object to replace
////for (int simg_idx = 0; simg_idx < sum_obj_size.size(); simg_idx++)
////{
////    score_obj_size[simg_idx] = pow(sum_obj_size[simg_idx] - mean_obj_size, 2);
////}

////// Finds te max and the spix_id corresponding to the max
////for (int simg_idx = 0; simg_idx < sum_obj_size.size(); simg_idx++)
////{
////    if (score_obj_size[simg_idx] > max_size_obj_id)
////    {
////        max_size_obj_id = score_obj_size[simg_idx];
////        best_obj_id_2 = simg_idx + 1;
////    }
////}

////vector<ObjThread> vct_obj_thread(m_vct_simg.size() - 1);

////// Crossframe object counter
////int obj_cnt = 0;

////for (int simg_idx = 0; simg_idx < m_vct_simg.size() - 1; simg_idx++)
////{

////    if (intfrm_obj_thread[simg_idx].size() > 0)
////    {
////        obj_cnt++;
////    }
////}

////const int min_persistency =  (float)m_vct_simg.size() / 4;

////float min_size_diff = 99999.999;
////int best_obj_id = -1;
////// vector<int> vct_maj_score(m_vct_simg.size() - 1);


////if (obj_cnt > min_persistency)
////{

////    // Size expressed as number of sPix per Object
////    float size_variance = 0.0;
////    int obj_size_2 = 0;

////    for (int simg_idx = 0; simg_idx < m_vct_simg.size() - 1; simg_idx++)
////    {
////        size_variance += pow(intfrm_obj_thread[simg_idx].size() - size_mean, 2);
////    }

////    size_variance /= m_vct_simg.size() - 1;
////    size_variance = sqrt(size_variance);

////    for (int simg_idx = 0; simg_idx < m_vct_simg.size() - 1; simg_idx++)
////    {
////        // Normalized size difference
////        const int size_diff = pow(intfrm_obj_thread[simg_idx].size() - size_variance, 2);
////        // const float norm_size_diff = size_diff / size_max;

////        if (size_diff < min_size_diff)
////        {
////            min_size_diff = size_diff;
////            best_obj_id = simg_idx + 1;
////            //  vct_maj_score[simg_idx]++;
////        }

////        vct_obj_thread[simg_idx].m_isomorph = size_diff;

////        for (int i = 0; i < intfrm_obj_thread[simg_idx].size(); i++)
////        {
////            SPixPair spix_pair = intfrm_obj_thread[simg_idx][i];
////            vct_obj_thread[simg_idx].m_vct_pair.push_back(spix_pair);
////        }


////        // Computes the size as number of px composing each spix belonging to each Obj


////    }

////    // Replacing the Obj corresonding to the highest MSE

////    Mat dst_rgb(m_vct_rgb_src[0].size(), m_vct_rgb_src[0].type());

////    for (int simg_idx = 0; simg_idx < m_vct_rgb_src.size(); simg_idx++)
////    {

////        m_vct_rgb_src[simg_idx].copyTo(dst_rgb);

////        const int obj_idx = 0;

////        for (int i = 0; i < vct_obj_thread[obj_idx].m_vct_pair.size(); i++)
////        {
////            const int spix_idx = vct_obj_thread[obj_idx].m_vct_pair[i].m_id_0;
////            SuperPixel *p_spix = &(m_vct_simg[best_obj_id_2].m_vct_spix[spix_idx]);

////            for (int px_idx = 0; px_idx < p_spix->m_spix_pos.size(); px_idx++)
////            {
////                const int r = p_spix->m_spix_pos[px_idx].m_r;
////                const int c = p_spix->m_spix_pos[px_idx].m_c;

////                dst_rgb.at<Vec3b>(r, c)[0] = m_vct_rgb_src[0].at<Vec3b>(r, c)[0];
////                dst_rgb.at<Vec3b>(r, c)[1] = m_vct_rgb_src[0].at<Vec3b>(r, c)[1];
////                dst_rgb.at<Vec3b>(r, c)[2] = m_vct_rgb_src[0].at<Vec3b>(r, c)[2];
////            }
////        }

////        char val[128];
////        memset(val, 0, 128);
////        sprintf(val, "%d", simg_idx);
////        string file_out = "D:/PRJ/ALGO_SLIC/REPLACE/replace_";
////        file_out += val;
////        file_out += ".jpg";
////        imwrite(file_out, dst_rgb);
////    }

////    // std::vector<int>::iterator result;
////    // result = std::max_element(vct_maj_score.begin(), vct_maj_score.end());

////    // int best_obj_id2 = std::distance(vct_maj_score.begin(), result);
////    // Computes the pair-wise inter-frame sPix motion affinities

////    MotionAffinityMatching(intfrm_paired_dist);

////    return true;
////    int simg_idx_0 = 0;
////    for (int simg_idx = 0; simg_idx <m_vct_simg.size(); simg_idx++)
////    {
////        // Vector of inter frame mean centroid shift
////        vector <float> mean_label_shift(m_vct_simg[simg_idx_0].m_vct_spix.size());

////        for (int spix_idx = 0; spix_idx < m_vct_simg[simg_idx_0].m_vct_spix.size(); spix_idx++)
////        {
////            PairedSuperPixel paired_spix;

////            cout << "SPix match: (" << spix_idx <<  " - ";

////            // The value of i is the nominal sPix label
////            mean_label_shift[spix_idx] += spix_idx;

////            //            for (int simg_idx = 0; simg_idx < m_vct_simg.size() - 1; simg_idx++)
////            //            {
////            //                paired_spix = vct_spix_pairs[simg_idx][spix_idx][0];
////            //                mean_label_shift[spix_idx] += paired_spix.m_spix_id;

////            //                cout << paired_spix.m_spix_id;

////            //                if (simg_idx < m_vct_simg.size() - 2)
////            //                {
////            //                    cout << " - ";
////            //                }
////            //            }
////            //            cout << ")" << endl;

////            //            mean_label_shift[spix_idx] /= m_vct_simg.size();

////            //            cout << "Mean label shift: " << mean_label_shift[spix_idx] << endl;
////            //            cout << "-----------------------------------------------------------" << endl;
////            //            cout << endl;
////        }

////    }




////    // Calculates the Max coordinates for the entire src image stack
////    int max_r = 0;
////    int max_c = 0;

////    for (int simg_idx = 0; simg_idx < m_vct_rgb_src.size(); simg_idx++)
////    {
////        if (m_vct_rgb_src[simg_idx].rows > max_r)
////        {
////            max_r = m_vct_rgb_src[simg_idx].rows;
////        }

////        if (m_vct_rgb_src[simg_idx].cols > max_c)
////        {
////            max_c = m_vct_rgb_src[simg_idx].cols;
////        }
////    }

////    // Vector of pairwise photometric SuperImage overlaps
////    vector<Mat> overlap_image(m_vct_simg.size());

////    // Image of stack photometric SuperImage overlaps
////    Mat overlap_stack = Mat::zeros(max_r, max_c, CV_32FC3);

////    vector<Mat> ovelap_mask(m_vct_simg.size() - 1);

////    // Super Pixel Pairwise Stability Gradient Table

////    // Allocation of the overlap image vector
////    for (int simg_idx = 0; simg_idx < m_vct_simg.size() - 1; simg_idx++)
////    {
////        overlap_image[simg_idx] = Mat::zeros(max_r, max_c, CV_32FC3);
////        ovelap_mask[simg_idx] = Mat::zeros(max_r, max_c, CV_8UC3);
////    }

////    for (int simg_idx = 0; simg_idx < m_vct_simg.size(); simg_idx++)
////    {
////        for (int spix_idx = 0; spix_idx < m_vct_simg[simg_idx].m_vct_spix.size(); spix_idx++)
////        {
////            // Allocation of the graph edge vector for every sPix belonging to each sImg
////            m_vct_simg[simg_idx].m_vct_spix[spix_idx].m_vct_overlap_ratio = vector<float>(m_vct_simg.size());
////        }
////    }

////    // Cross frame pixel matching threshold values
////    const float error_red = 3.0;
////    const float error_green = 3.0;
////    const float error_blue = 3.0;

////    // Converts the images into float
////    vector<Mat> vct_src_float(m_vct_rgb_src.size());

////    for (int simg_idx = 0; simg_idx < m_vct_rgb_src.size(); simg_idx++)
////    {
////        m_vct_rgb_src[simg_idx].copyTo(vct_src_float[simg_idx]);
////        vct_src_float[simg_idx].convertTo(vct_src_float[simg_idx], CV_32FC3);
////    }

////    // Calculates the overlapping regions between crossframe sPix
////    const int simg_size = m_vct_simg[simg_idx_0].m_vct_spix.size();

////    for (int spix_idx_0 = 0; spix_idx_0 < simg_size; spix_idx_0++)
////    {
////        // cout << "-----------------------------------------------------------" << endl;
////        // cout <<  "SPix: " << spix_idx_0 << endl;
////        // cout << "-----------------------------------------------------------" << endl;

////        for (int simg_idx_0 = 0; simg_idx_0 < m_vct_simg.size() - 1; simg_idx_0++)
////        {
////            // Scans all pixels belonging to the i reference sPix of the reference image 0
////            const int spix_size_0 = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_spix_pos.size();

////            //            const int min_spix_x = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_min_x;
////            //            const int min_spix_y = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_min_y;
////            //            const int max_spix_x = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_max_x;
////            //            const int max_spix_y = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_max_y;

////            //for (int sh_x = 2-10; sh_x < 10; sh_x++)
////            // {
////            //     for (int sh_y = -10; sh_y < 10; sh_y++)
////            //     {

////            int px_match = 0;

////            for (int px_idx = 0; px_idx < spix_size_0; px_idx++)
////            {
////                const int r_0 = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_spix_pos[px_idx].m_r;
////                const int c_0 = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_spix_pos[px_idx].m_c;

////                for (int simg_idx_1 = simg_idx_0 + 1; simg_idx_1 < m_vct_simg.size(); simg_idx_1++)
////                {
////                    // Returns the sPix id of the paired sPix
////                    const int spix_idx_1 = vct_spix_pairs[simg_idx_1][spix_idx_0][0].m_id_1;

////                    if (px_idx < m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1].m_spix_pos.size())
////                    {
////                        // Calculates the cartesian difference between the centroid of the reference image
////                        // and the crossframe centroid
////                        const int dg_x = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1].m_gx - m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_gx - m_vct_src_shift[simg_idx_1].x;
////                        const int dg_y = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1].m_gy - m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_gy - m_vct_src_shift[simg_idx_1].y;
////                        const int r_1 = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1].m_spix_pos[px_idx].m_r - dg_x; // - m_vct_src_shift[simg_idx_1].x;
////                        const int c_1 = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1].m_spix_pos[px_idx].m_c - dg_y; // - m_vct_src_shift[simg_idx_1].y;

////                        // Checks for matrix overflow. r_1 and c_1 are calculated on the simg_id matrix coordinates
////                        // thay may differ to the simgidx = 0 reference matrix size

////                        if (r_1 >= 0 &&
////                                c_1 >= 0 &&
////                                r_1 + m_vct_src_shift[simg_idx_0].x < m_vct_rgb_src[simg_idx_0].rows &&
////                                c_1 + m_vct_src_shift[simg_idx_0].y < m_vct_rgb_src[simg_idx_0].cols) // &&
////                        {
////                            // cout << px_idx << "  r_0: " << r_0 << "   c_0: " << c_0 << "   r_1: " << r_1 << "   c_1: " << c_1 << endl;

////                            // Cross frame operation selection
////                            const bool csom = 0;

////                            if (csom == 0)
////                            {
////                                //------------------------------------------------------------------
////                                // 1. Cross-frame sPix overlap matcher : Differences of RGB intensity
////                                // Basic cross-frame overlap matcher
////                                //------------------------------------------------------------------
////                                // All photometric differencies are mesured in float arcce
////                                px_match = (abs(vct_src_float[simg_idx_1].at<Vec3f>(r_1, c_1)[0] - vct_src_float[simg_idx_0].at<Vec3f>(r_0, c_0)[0]) <= error_blue &&
////                                        abs(vct_src_float[simg_idx_1].at<Vec3f>(r_1, c_1)[1] - vct_src_float[simg_idx_0].at<Vec3f>(r_0, c_0)[1]) <= error_green &&
////                                        abs(vct_src_float[simg_idx_1].at<Vec3f>(r_1, c_1)[2] - vct_src_float[simg_idx_0].at<Vec3f>(r_0, c_0)[2]) <= error_red);

////                                // Pairwise overlapping propagation
////                                if (px_match == 1)
////                                {
////                                    m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_vct_overlap_ratio[simg_idx_1]++;
////                                    m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1].m_vct_overlap_ratio[simg_idx_0]++;

////                                    overlap_image[simg_idx_1 - 1].at<Vec3f>(r_1, c_1) = vct_src_float[simg_idx_0].at<Vec3f>(r_0, c_0);
////                                }
////                            }
////                            else // if (csom == 1)
////                            {
////                                //------------------------------------------------------------------
////                                // 2. Cross-frame sPix overlap matching : sPix photometric convolution
////                                //------------------------------------------------------------------

////                                //------------------------------------------------------------------
////                                // 3. Cross-frame sPix overlap matching : sPix geometric convolution
////                                //------------------------------------------------------------------
////                                float hue_similarity = 0.0;
////                                SPixHueMatching(simg_idx_0, simg_idx_1, spix_idx_0, spix_idx_1, hue_similarity);
////                            }
////                        }
////                        else  // Overflow pixel matching
////                        {
////                            // overlap_image[simg_idx_1 - 1].at<Vec3f>(r_0, c_0) = Vec3b(0.0, 255.0, 255.0);
////                        }
////                    }
////                }
////            }

////            // Calculates the ratio from the quantity
////            for (int simg_idx_1 = simg_idx_0 + 1; simg_idx_1 < m_vct_simg.size(); simg_idx_1++)
////            {
////                m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_vct_overlap_ratio[simg_idx_1] /= (float)spix_size_0;


////                //----------------------------------------------------------------------
////                // Disable this section to apply simmetry filter to overlapping weights
////                //----------------------------------------------------------------------
////                // const float a1 = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_spix_pos.size();
////                // const float a2 = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_0].m_spix_pos.size();
////                // float temp = pow(a2/a1, 10);
////                // float val_before = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_vct_overlap_ratio[simg_idx_1];
////                // m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_vct_overlap_ratio[simg_idx_1] *= temp;
////                // float val_after = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_vct_overlap_ratio[simg_idx_1];
////                // End of section

////                const int spix_idx_1 = vct_spix_pairs[simg_idx_1][spix_idx_0][0].m_id_1;
////                const int spix_size_1 = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1].m_spix_pos.size();

////                m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1].m_vct_overlap_ratio[simg_idx_0] /= (float)spix_size_1;
////            }
////        }
////    }

////    //    // Dumps the spix overlapping ratio
////    //    for (int simg_idx = 0; simg_idx < m_vct_rgb_src.size() - 1; simg_idx++)
////    //    {
////    //        cout << "--------------------------------------" << endl;

////    //        for (int spix_idx_0 = 0; spix_idx_0 < m_vct_simg[simg_idx].m_vct_spix.size(); spix_idx_0++)
////    //        {
////    //            // float ratio = m_vct_ovr_nbr[simg_idx][spix_idx_0];

////    //            float ratio2 = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0].m_vct_overlap_ratio[1] ;
////    //            cout << "SImage: " << simg_idx << " - SPix: " << spix_idx_0 << " -  Ratio: " << ratio2 << endl;
////    //        }
////    //    }

////    const string path_dst = "D:/PRJ/ALGO_SLIC/";
////    SPixTools spt;

////    //-----------------------------------------------------------
////    // Dumps the simg of the overlay ratios
////    //-----------------------------------------------------------
////    spt.dumpAllPairwiseOverlaps(*this, path_dst);

////    //-----------------------------------------------------------
////    // Dumps the overlapping px in the sPix
////    //-----------------------------------------------------------
////    for (int ovr_idx = 0; ovr_idx < overlap_image.size(); ovr_idx++)
////    {
////        // Converts the overlapping image to CV_8UC3
////        overlap_image[ovr_idx].convertTo(overlap_image[ovr_idx], CV_8UC3);

////        char val[128];
////        memset(val, 0, 128);
////        sprintf(val, "%d", ovr_idx);
////        string file_out = "D:/PRJ/ALGO_SLIC/mask_overlap_";
////        file_out += val;
////        file_out += ".jpg";
////        imwrite(file_out, overlap_stack); //ovelap_mask[ovr_idx]);

////        file_out = "D:/PRJ/ALGO_SLIC/image_overlap_";
////        file_out += val;
////        file_out += ".jpg";
////        imwrite(file_out, overlap_image[ovr_idx]);
////    }

////    stackMerge(0, 1);

////    for (int j = 0; j < m_vct_simg.size() - 1; j++)
////    {
////        cout << "-----------------------------------------------------------" << endl;
////        cout <<  "Image: " << j << endl;
////        cout << "-----------------------------------------------------------" << endl;

////        for (int i = 0; i < m_vct_simg[0].m_vct_spix.size(); i++)
////        {
////            int  paired_id = vct_spix_pairs[j][i][0].m_spix_id;
////            cout << "Stack level: " << j << "  - sPix match: (" << i << " - " << paired_id << ")" << endl;
////        }
////    }
////   return true;
//// }

////-----------------------------------------------------------------------------

//bool Mosaica::createHueHistogram(const int simg_idx, const int spix_idx, Mat src, Mat &hue_histogram)
//{
//    const Size src_size = src.size();
//    Mat vct_channel[3];
//    vct_channel[0] = Mat::zeros(src_size, CV_32FC1);
//    vct_channel[1] = Mat::zeros(src_size, CV_32FC1);
//    vct_channel[2] = Mat::zeros(src_size, CV_32FC1);

//    // Color conversion RGB to HSV
//    Mat hsv = Mat::zeros(src_size, CV_32FC3);
//    cvtColor(src, hsv, COLOR_BGR2HSV);
//    split(hsv, vct_channel);

//    // One Dimension integer Matrix
//    hue_histogram = Mat::zeros(1, 360, CV_32SC1);

//    // Creates a sPix matrix frm the sPix position vector
//    // SuperPixel *p_spix_0 = &(m_vct_simg[simg_idx].m_vct_spix[spix_idx]);

//    for (int r = 0; r < src.rows; r++)
//    {
//        for (int c = 0; c < src.cols; c++)
//        {
//            float hue_val = vct_channel[0].at<int8_t>(r, c) * 2.0;

//            if (hue_val < 0)
//            {
//                hue_val = 180.0 + hue_val;
//            }
//            hue_histogram.at<int>(0, (uint8_t)hue_val)++;
//        }
//    }
//    return true;
//}

////-----------------------------------------------------------------------------

//bool Mosaica::createHueHistogram(const int simg_idx, const int spix_idx, Mat &hue_histogram)
//{
//    const Size src_size = m_vct_rgb_src[simg_idx].size();
//    Mat vct_channel[3];
//    vct_channel[0] = Mat::zeros(src_size, CV_32FC1);
//    vct_channel[1] = Mat::zeros(src_size, CV_32FC1);
//    vct_channel[2] = Mat::zeros(src_size, CV_32FC1);

//    // Color conversion RGB to HSV
//    Mat hsv = Mat::zeros(src_size, CV_32FC3);
//    cvtColor(m_vct_rgb_src[simg_idx], hsv, COLOR_BGR2HSV);
//    split(hsv, vct_channel);

//    // One Dimension integer Matrix
//    hue_histogram = Mat::zeros(1, 360, CV_32SC1);

//    // Creates a sPix matrix frm the sPix position vector
//    SuperPixel *p_spix_0 = &(m_vct_simg[simg_idx].m_vct_spix[spix_idx]);

//    for (int i = 0; i < p_spix_0->m_spix_pos.size(); i++)
//    {
//        const int r = p_spix_0->m_spix_pos[i].m_r;
//        const int c = p_spix_0->m_spix_pos[i].m_c;

//        float hue_val = vct_channel[0].at<int8_t>(r, c) * 2.0;

//        if (hue_val < 0)
//        {
//            hue_val = 256 + (256 + hue_val);
//        }
//        hue_histogram.at<int>(0, (uint8_t)hue_val)++;
//        // int val = hue_histogram.at<int>(0, (uint8_t)hue_val);
//        // cout << "spixIdx: " << i << " - Bean: " << hue_val << "  - Val: " << val << endl;
//    }
//    return true;
//}
//// Template matching : Hue convolutional
//// Alternative : SAD (Sum of Absolute Differences)
//bool Mosaica::SPixHueMatching(const int simg_idx_0, const int simg_idx_1, const int spix_idx_0, const int spix_idx_1, float &hue_similarity){
//    // Mat src_0 = imread("D:/PRJ/ALGO_SLIC/CONVOLUTION/img_2.bmp");
//    // Mat src_1 = imread("D:/PRJ/ALGO_SLIC/CONVOLUTION/img_3.bmp");

//    const int rows = 1;
//    const int cols = 360;

//    Mat hist_0 = Mat::zeros(rows, cols, CV_32FC1);
//    Mat hist_1 = Mat::zeros(rows, cols, CV_32FC1);

//    createHueHistogram(simg_idx_0, spix_idx_0, hist_0);
//    createHueHistogram(simg_idx_1, spix_idx_1, hist_1);

//    hist_0.convertTo(hist_0, CV_32F);
//    hist_1.convertTo(hist_1, CV_32F);

//    // const float alpha = 180.0;
//    const int kern_width = 31;

//    //  for (int i = 0; i < kern_width; i++)
//    // {
//    //   int sz = src_0.rows * src_0.cols;
//    //    hist_0.at<float>(0, alpha) = sz;
//    //   int j = alpha - (float)kern_width/2.0 + i;
//    //    hist_1.at<float>(0, j) = sz;

//    normalize(hist_0, hist_0, 0, 1.0, NORM_MINMAX, CV_32FC1);
//    normalize(hist_1, hist_1, 0, 1.0, NORM_MINMAX, CV_32FC1);

//    double min_val, max_val;
//    Point min_idx, max_idx;
//    minMaxLoc(hist_0, &min_val, &max_val, &min_idx, &max_idx);

//    const Mat sub_hist = hist_1(Rect(max_idx.x - (kern_width/2), 0, kern_width, 1));
//    sub_hist.copyTo(sub_hist);

//    // The kernel vector must be a column vector as required by OpenCv
//    Mat kernel = Mat::zeros(kern_width, 1, CV_32F);
//    kernel = getGaussianKernel(kern_width, 2.0, CV_32F);
//    normalize(kernel, kernel, 0, 1.0, NORM_MINMAX, CV_32FC1);
//    Mat mat_prd = sub_hist * kernel;
//    hue_similarity = mat_prd.at<float>(0, 0);

//    //     cout << "Hue: " << j << " - Product: " << prd << endl;
//    // }
//    return true;
//}

//bool Mosaica::RgbToHue(const int simg_idx, Mat &hue_dst)
//{
//    Mat vct_channel[3];

//    // Color conversion RGB to HSV
//    Mat hsv;
//    cvtColor(m_vct_rgb_src[simg_idx], hsv, COLOR_BGR2HSV);
//    split(hsv, vct_channel);
//    vct_channel[0].copyTo(hue_dst);
//    hue_dst.convertTo(hue_dst, CV_32F);
//    return true;
//}

//bool Mosaica::SPixHueSAD(const int simg_idx_0, const int spix_idx_0, const int simg_idx_1, const int spix_idx_1, float &hue_sad)
//{
//    Mat hue_dst_0;
//    RgbToHue(simg_idx_0, hue_dst_0);

//    // Mat vct_channel[3];

//    // int size_x = shared_max_x - shared_min_x;
//    // int size_y = shared_max_y - shared_min_y;
//    // const Size src_size(size_x, size_y);

//    //  vct_channel[0] = Mat::zeros(size_x, size_y, CV_32FC1);
//    //  vct_channel[1] = Mat::zeros(size_x, size_y, CV_32FC1);
//    //  vct_channel[2] = Mat::zeros(size_x, size_y, CV_32FC1);

//    // Color conversion RGB to HSV
//    // Mat hsv = Mat::zeros(size_x, size_y, CV_32FC3);
//    // cvtColor(m_vct_rgb_src[simg_idx_0](Rect(min_y_0, min_x_0, size_y, size_x)), hsv, COLOR_BGR2HSV);
//    // split(hsv, vct_channel);

//    SuperPixel *p_spix_0 = &(m_vct_simg[simg_idx_0].m_vct_spix[spix_idx_0]);
//    int rgb_sum_0 = 0;

//    // vct_channel[0].convertTo(vct_channel[0], CV_32F);

//    int sz_0 = p_spix_0->m_spix_pos.size();
//    int nbr_cmp_px = sz_0;

//    for (int px_idx = 0; px_idx < sz_0; px_idx++)
//    {
//        const int r = p_spix_0->m_spix_pos[px_idx].m_r;
//        const int c = p_spix_0->m_spix_pos[px_idx].m_c;

//        //  const int r1 = r - shared_min_x - 1;
//        //  const int c1 = c - shared_min_y - 1;

//        //  if (r1 >= 0 && c1 >= 0 && r1 < size_x && c1 < size_y)
//        {
//            rgb_sum_0 += hue_dst_0.at<float>(r, c);
//        }

//    }
//    float rgb_mean_0 = rgb_sum_0 / nbr_cmp_px;

//    Mat hue_dst_1;
//    RgbToHue(simg_idx_1, hue_dst_1);

//    SuperPixel *p_spix_1 = &(m_vct_simg[simg_idx_1].m_vct_spix[spix_idx_1]);
//    int rgb_sum_1 = 0;

//    const int sz_1 = p_spix_1->m_spix_pos.size();
//    nbr_cmp_px = sz_1;

//    for (int px_idx = 0; px_idx < sz_1; px_idx++)
//    {
//        const int r = p_spix_1->m_spix_pos[px_idx].m_r;
//        const int c = p_spix_1->m_spix_pos[px_idx].m_c;

//        //   const int r1 = r - shared_min_x - 1;
//        //   const int c1 = c - shared_min_y - 1;

//        //if (r1 >= 0 && c1 >= 0 && r1 < size_x && c1 < size_y)
//        {
//            rgb_sum_1 += (hue_dst_1.at<float>(r, c));
//        }
//    }
//    float rgb_mean_1 = rgb_sum_1 / nbr_cmp_px;

//    hue_sad = abs(rgb_mean_1 - rgb_mean_0);
//    const int epsilon = 1;

//    if (abs(hue_sad) > epsilon)
//    {
//        return false;
//    }
//    return true;
//}

//bool Mosaica::SPixHueHistogramDiff(const int simg_idx_0, const int simg_idx_1, const int spix_idx_0, const int spix_idx_1, float &hue_diff)
//{

//    Mat src_0 = imread("D:/PRJ/ALGO_SLIC/CONVOLUTION/img_2.bmp");
//    Mat src_1 = imread("D:/PRJ/ALGO_SLIC/CONVOLUTION/img_3.bmp");

//    const int rows = 1;
//    const int cols = 360;

//    Mat hist_0 = Mat::zeros(rows, cols, CV_32FC1);
//    Mat hist_1 = Mat::zeros(rows, cols, CV_32FC1);

//    createHueHistogram(simg_idx_0, spix_idx_0, src_0, hist_0);
//    createHueHistogram(simg_idx_1, spix_idx_1, src_1, hist_1);

//    hist_0.convertTo(hist_0, CV_32F);
//    hist_1.convertTo(hist_1, CV_32F);

//   // dump_matrix_f32_cv("Hist_0", 0, hist_0);
//  //  dump_matrix_f32_cv("Hist_1", 0, hist_1);

//    //  normalize(hist_0, hist_0, 0, 1.0, NORM_MINMAX, CV_32FC1);
//    //  normalize(hist_1, hist_1, 0, 1.0, NORM_MINMAX, CV_32FC1);

//    float val_0, val_1;

//    for (int i = 0; i < 360; i++)
//    {
//        val_0 = hist_0.at<float>(0, i);
//        val_1 = hist_1.at<float>(0, i);
//        hue_diff += abs(val_1 - val_0);
//    }
//    return true;
//}

//void RGBToGray(const Mat src, Mat &dst)
//{
//    Mat vct_channel[3];
//    vct_channel[0] = Mat::zeros(src.size(), CV_8UC1);
//    vct_channel[1] = Mat::zeros(src.size(), CV_8UC1);
//    vct_channel[2] = Mat::zeros(src.size(), CV_8UC1);

//    // Color conversion RGB to HSV
//    Mat hsv = Mat::zeros(src.size(), CV_32FC3);

//    cvtColor(src, hsv, COLOR_RGB2HSV);
//    split(hsv, vct_channel);

//    vct_channel[0].convertTo(vct_channel[0], CV_32FC1);
//    vct_channel[1].convertTo(vct_channel[1], CV_32FC1);
//    vct_channel[2].convertTo(vct_channel[2], CV_32FC1);
//    vct_channel[2].copyTo(dst);
//}

////--------------------------------------------------------------------------------------------------------------------------
//// SPix Crossframe overlapping penetration
////-----------------------------------------------------------------------------
//bool Mosaica::crossframOverlapPenetration(const int simg_idx_0, const int simg_idx_1, vector<float> vct_cross_overlap)
//{
//    const int simg_size = m_vct_simg[simg_idx_0].m_vct_spix.size();

//    for (int spix_idx = 0; spix_idx < simg_size; spix_idx++)
//    {
//        float cross_ovr = 1.0;

//        for (int simg_idx = simg_idx_0 + 1; simg_idx <= simg_idx_1; simg_idx++)
//        {
//            // Note: Replace this function with more advanced weigthed or norm_k ones
//            cross_ovr *= m_vct_simg[simg_idx_0].m_vct_spix[spix_idx].m_vct_overlap_ratio[simg_idx];
//        }
//        vct_cross_overlap.push_back(cross_ovr);
//    }

//    const string path_dst = "D:/PRJ/ALGO_SLIC/";
//    SPixTools spt(path_dst);

//    // spt.dumpPairwiseOverlap(*this, path_dst, simg_idx_0, simg_idx_1, vct_cross_overlap);
//    return true;
//}

////-----------------------------------------------------------------------------

//bool Mosaica::stackMerge(const int t0, const int t1)
//{
//    const int t_w = t1 - t0;

//    // Converts the images into float
//    vector<Mat> vct_src_float(m_vct_rgb_src.size());
//    Mat vct_channel[4][3];

//    // int simg_idx = 0;
//    for (int simg_idx = 0; simg_idx < m_vct_rgb_src.size(); simg_idx++)
//    {
//        m_vct_rgb_src[simg_idx].copyTo(vct_src_float[simg_idx]);
//        vct_src_float[simg_idx].convertTo(vct_src_float[simg_idx], CV_32FC3);
//        vct_src_float[simg_idx].setTo(0);

//        vct_channel[simg_idx][0] = Mat::zeros(m_vct_rgb_src[simg_idx].size(), CV_8UC1);
//        vct_channel[simg_idx][1] = Mat::zeros(m_vct_rgb_src[simg_idx].size(), CV_8UC1);
//        vct_channel[simg_idx][2] = Mat::zeros(m_vct_rgb_src[simg_idx].size(), CV_8UC1);

//        // Color conversion RGB to HSV
//        Mat hsv = Mat::zeros(m_vct_rgb_src[simg_idx].size(), CV_32FC3);

//        cvtColor(vct_src_float[simg_idx], hsv, COLOR_RGB2HSV);

//        split(hsv, vct_channel[simg_idx]);

//        vct_channel[simg_idx][0].convertTo(vct_channel[simg_idx][0], CV_32FC1);
//        vct_channel[simg_idx][1].convertTo(vct_channel[simg_idx][1], CV_32FC1);
//        vct_channel[simg_idx][2].convertTo(vct_channel[simg_idx][2], CV_32FC1);

//    }

//    Point max_shift(0, 0);

//    // The input images are assumed to have all the same size

//    // Creates the mosaic image canvas
//    for (int sh_idx = 0; sh_idx < m_vct_src_shift.size(); sh_idx++)
//    {
//        if (m_vct_src_shift[sh_idx].x > max_shift.x)
//        {
//            max_shift.x = m_vct_src_shift[sh_idx].x;
//        }

//        if (m_vct_src_shift[sh_idx].y > max_shift.y)
//        {
//            max_shift.y = m_vct_src_shift[sh_idx].y;
//        }
//    }

//    // Creates the dest mosaic empty canvas
//    Mat mask_canvas = Mat::zeros(m_vct_rgb_src[0].rows + max_shift.x, m_vct_rgb_src[0].cols + max_shift.y, CV_8UC3);
//    Mat mosaic_canvas = Mat::zeros(m_vct_rgb_src[0].rows + max_shift.x, m_vct_rgb_src[0].cols + max_shift.y, CV_8UC3);
//    Mat mosaic_canvas_float = Mat::zeros(m_vct_rgb_src[0].rows + max_shift.x, m_vct_rgb_src[0].cols + max_shift.y, CV_32FC3);

//    // Detects the sPix that don't overlap with other images and adds them to the dst canvas image

//    //   int simg_idx_0 = 0;
//    for (int simg_idx_0 = 0; simg_idx_0 < m_vct_rgb_src.size(); simg_idx_0++)
//    {
//        const int simg_size = m_vct_simg[simg_idx_0].m_vct_spix.size();

//        for (int spix_idx = 0; spix_idx < simg_size; spix_idx++)
//        {
//            int ovr_depth = 0;

//            //   int simg_idx_1 = 0;
//            for (int simg_idx_1 = 0; simg_idx_1 < m_vct_rgb_src.size(); simg_idx_1++)
//            {
//                if (simg_idx_0 != simg_idx_1)
//                {
//                    if (m_vct_simg[simg_idx_0].m_vct_spix[spix_idx].m_vct_overlap_ratio[simg_idx_1] > 0.0001)
//                    {
//                        ovr_depth++;
//                    }
//                }
//            }

//            if (ovr_depth == 0)    // Adds the non overlapped sPix
//            {
//                SuperPixel *p_spix = &(m_vct_simg[simg_idx_0].m_vct_spix[spix_idx]);

//                // Adds the contents of the sPix to the canvas image
//                for (int px_idx = 0; px_idx < p_spix->m_spix_pos.size(); px_idx++)
//                {
//                    PixPos pos = p_spix->m_spix_pos[px_idx];

//                    mask_canvas.at<Vec3b>(pos.m_r + max_shift.x - m_vct_src_shift[simg_idx_0].x, pos.m_c + max_shift.y - m_vct_src_shift[simg_idx_0].y)[2] = 255;

//                    mosaic_canvas.at<Vec3b>(pos.m_r + max_shift.x - m_vct_src_shift[simg_idx_0].x, pos.m_c + max_shift.y - m_vct_src_shift[simg_idx_0].y)[0] = m_vct_rgb_src[simg_idx_0].at<Vec3b>(pos.m_r, pos.m_c)[0];
//                    mosaic_canvas.at<Vec3b>(pos.m_r + max_shift.x - m_vct_src_shift[simg_idx_0].x, pos.m_c + max_shift.y - m_vct_src_shift[simg_idx_0].y)[1] = m_vct_rgb_src[simg_idx_0].at<Vec3b>(pos.m_r, pos.m_c)[1];
//                    mosaic_canvas.at<Vec3b>(pos.m_r + max_shift.x - m_vct_src_shift[simg_idx_0].x, pos.m_c + max_shift.y - m_vct_src_shift[simg_idx_0].y)[2] = m_vct_rgb_src[simg_idx_0].at<Vec3b>(pos.m_r, pos.m_c)[2];
//                }
//            }
//            else              // Adds the overlapped sPix
//            {
//                // Distantiometric weight : A non liner weight is assigned to each sPix based on the euclidean distance
//                // of the sPix to the center of its simg
//                const int c_x = m_vct_rgb_src[simg_idx_0].rows / 2;
//                const int c_y = m_vct_rgb_src[simg_idx_0].cols / 2;

//                float dist_weight_min = 10000.0;
//                float spix_score_min = 10000.0;
//                vector<float> vct_spix_score(m_vct_simg.size());

//                // Index of the sPix with the minimum distance weight
//                int idx_weight_min = 0;

//                for (int simg_idx_1 = 0; simg_idx_1 < m_vct_rgb_src.size(); simg_idx_1++)
//                {
//                    int gx1 = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gx;
//                    int gy1 = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gy;
//                    int dx = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gx - c_x;
//                    int dy = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gy - c_y;
//                    int pow1 = pow(dx, 2);
//                    int pow2 = pow(dy, 2);
//                    // int pow_ko_1 = pow(m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gx - c_x, 2);
//                    // int pow_ko_2 = pow(m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gy - c_y, 2);

//                    float sqrt1 = sqrt(pow1 + pow2);

//                    float dist_weight = sqrt1; //sqrt(pow(m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gx - c_x, 2) + pow(m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_gy - c_y, 2));

//                    if (dist_weight < dist_weight_min)
//                    {
//                        dist_weight_min = dist_weight;
//                    }

//                    // The distance weight from the image center and the overlap ration are summed up to obtin the final score
//                    // SPix selection score


//                    //---------------------------------------------------------------------------
//                    // SPix replacement policy
//                    //---------------------------------------------------------------------------

//                    //-------------------------------------------------------
//                    // 1. SPix overlapping symmetry weight
//                    //-------------------------------------------------------
//                    // Creates a quadratic weight that rise rapidly with the distance from the image edge
//                    const int dw = dist_weight_min - c_y;
//                    float exp_dist_weight = pow(dw, 2) / pow(c_y, 2);
//                    float ovr_ratio = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx].m_vct_overlap_ratio[simg_idx_1];
//                    // Final score function to be used to select the spix

//                    // The overlapping ratio is multiplied by te sPix size ratio
//                    // Improved stability : sPix of the simg_idx_1 > sPix simg_idx_0
//                    // Loose stability : sPix of the simg_idx_1 < sPix simg_idx_0
//                    // Keep stability : sPix of the simg_idx_1 = sPix simg_idx_0
//                    // If sPix sizes are equals, the overlap ratio wouldn't change
//                    const float a1 = m_vct_simg[simg_idx_0].m_vct_spix[spix_idx].m_spix_pos.size();
//                    const float a2 = m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_spix_pos.size();

//                    float temp = pow(a2/a1, 10);

//                    ovr_ratio *= temp; //(m_vct_simg[simg_idx_1].m_vct_spix[spix_idx].m_spix_pos.size() / m_vct_simg[simg_idx_0].m_vct_spix[spix_idx].m_spix_pos.size());

//                    // ETUDIER LA RELATION DE PARTAGE ENTRE SPIX DE TAILLE DIFFERENTE; VERIFIER SI CA DONNE DE L'INSTBILITE
//                    // THE OVR_RATIO MUST BE NORMALIZED !!

//                    // Strategies :
//                    // 0. sPix replacement with the sPix with the highest score
//                    // 1. sPix blending

//                    //---------------------------------------------------------
//                    // The following parameters must be chosen manually at once
//                    //---------------------------------------------------------
//                    const float weight_to_dist = 1; //32.0;
//                    const float weight_to_overlap = 100; //32.0;

//                    const float spix_score = ((exp_dist_weight * weight_to_dist) + (ovr_ratio * weight_to_overlap));

//                    vct_spix_score[simg_idx_1] = spix_score;

//                    if (spix_score < spix_score_min)
//                    {
//                        spix_score_min = spix_score;
//                        idx_weight_min = simg_idx_1;
//                    }
//                }

//                SuperPixel *p_spix = &(m_vct_simg[idx_weight_min].m_vct_spix[spix_idx]);

//                // Scans the entire stack of simages to calculate the weight of the superpixel
//                // Adds the contents of the sPix to the canvas image
//                for (int px_idx = 0; px_idx < p_spix->m_spix_pos.size(); px_idx++)
//                {
//                    PixPos pos = p_spix->m_spix_pos[px_idx];

//                    // Fills up the mask canvas
//                    if (idx_weight_min == 0)
//                    {
//                        mask_canvas.at<Vec3b>(pos.m_r + max_shift.x - m_vct_src_shift[simg_idx_0].x, pos.m_c + max_shift.y - m_vct_src_shift[simg_idx_0].y)[0] = vct_spix_score[simg_idx_0] * 64; //255;
//                    }
//                    else if (idx_weight_min == 1)
//                    {
//                        mask_canvas.at<Vec3b>(pos.m_r + max_shift.x - m_vct_src_shift[simg_idx_0].x, pos.m_c + max_shift.y - m_vct_src_shift[simg_idx_0].y)[1] =vct_spix_score[simg_idx_0] * 64; // 255;
//                    }

//                    //  int strategy = 0;

//                    //  if (strategy == 0)
//                    {
//                        // Fills up the mosaic canvas
//                        mosaic_canvas.at<Vec3b>(pos.m_r + max_shift.x - m_vct_src_shift[simg_idx_0].x, pos.m_c + max_shift.y - m_vct_src_shift[simg_idx_0].y)[0] = m_vct_rgb_src[simg_idx_0].at<Vec3b>(pos.m_r, pos.m_c)[0];
//                        mosaic_canvas.at<Vec3b>(pos.m_r + max_shift.x - m_vct_src_shift[simg_idx_0].x, pos.m_c + max_shift.y - m_vct_src_shift[simg_idx_0].y)[1] = m_vct_rgb_src[simg_idx_0].at<Vec3b>(pos.m_r, pos.m_c)[1];
//                        mosaic_canvas.at<Vec3b>(pos.m_r + max_shift.x - m_vct_src_shift[simg_idx_0].x, pos.m_c + max_shift.y - m_vct_src_shift[simg_idx_0].y)[2] = m_vct_rgb_src[simg_idx_0].at<Vec3b>(pos.m_r, pos.m_c)[2];
//                    }

//                    // The blend is performed in the HSV color arcce

//                    // else if (strategy == 1)
//                    {
//                        // The dst sPix will be the weighted sum of the cross stack sPix weight
//                        for (int simg_idx_1 = 0; simg_idx_1 < m_vct_rgb_src.size(); simg_idx_1++)
//                        {
//                            vct_channel[simg_idx_0][2].at<float>(pos.m_r, pos.m_c) += (vct_spix_score[simg_idx_1] * 1);
//                        }
//                    }
//                }
//            }
//        }
//    }

//    merge(vct_channel[0], 3, mosaic_canvas_float);
//    cvtColor(mosaic_canvas_float, mosaic_canvas_float, COLOR_HSV2RGB);

//    imwrite("D:/PRJ/ALGO_SLIC/mosaic_mask_1.jpg", mask_canvas);
//    imwrite("D:/PRJ/ALGO_SLIC/mosaic_1.jpg", mosaic_canvas);
//    imwrite("D:/PRJ/ALGO_SLIC/mosaic_blend_1.jpg", mosaic_canvas_float);

//    m_mosa_stack.push_back(mosaic_canvas);
//    return true;
//}
