////-------------------------------------------------------------------------------------
//// Project name: COHERENT MOSAICA
////
//// Creation Date: 20/03/2016
//// Created by: Roberto Giudici - OBELIX, UBS, IRISA, COSTEL, Universiy of Rennes 2
//// Description: Super pixel time coherent based Mosaicing algorithm
////-------------------------------------------------------------------------------------

//#include "spixtools.h"
//#include "superpixel.h"

//SPixTools::SPixTools(string path)
//{
//    m_path = path;
//}

////-----------------------------------------------------------------------------

//bool SPixTools::getImageFromSpix(SuperPixel *p_spix, const Mat rgb_src, Mat &spix_img)
//{
//    const int height = p_spix->m_max_x - p_spix->m_min_x + 1;
//    const int width = p_spix->m_max_y - p_spix->m_min_y + 1;

//    spix_img = Mat::zeros(height, width, CV_8UC3);

//    for (int i = 0; i < p_spix->m_spix_pos.size(); i++)
//    {
//        const int r = p_spix->m_spix_pos[i].m_r;
//        const int c = p_spix->m_spix_pos[i].m_c;
//        spix_img.at<Vec3b>(r - p_spix->m_min_x, c - p_spix->m_min_y) = rgb_src.at<Vec3b>(r, c);

//        //  const int r_merge = p_spix->m_spix_pos[i].m_r;
//        //  const int c_merge = p_spix->m_spix_pos[i].m_c;
//        //  spix_img_merge.at<Vec3b>(r_merge, c_merge) = rgb_src.at<Vec3b>(r_merge, c_merge);
//    }
//    return true;
//}

////-----------------------------------------------------------------------------

//bool SPixTools::dumpSpixImage(const GeometryMetrics &msa,
//                              const int simg_idx,
//                              const string path_dst,
//                              const enum SPixInfoOverlay info_type)
//{
//    const Mat *p_rgb_src = &(msa.m_vct_rgb_dst[simg_idx]);
//    const SuperImage *p_simg = &(msa.m_vct_simg[simg_idx]);

//    if (!p_simg->m_geo_img)
//    {
//        return false;
//    }

//    string overlay_name = "";
//    Mat rgb_dst = Mat::zeros(p_rgb_src->rows, p_rgb_src->cols, CV_8UC3);

//    // Copies the slic image to the dst image
//    p_rgb_src->copyTo(rgb_dst);

//    for (int spix_idx = 0; spix_idx < p_simg->m_vct_spix.size(); spix_idx++)
//    {
//        const int gx = p_simg->m_vct_spix[spix_idx].m_gx;
//        const int gy = p_simg->m_vct_spix[spix_idx].m_gy;
//        int txt_gray = 255;
//        int txt_yellow = 128;

//        if (p_simg->m_vct_spix[spix_idx].m_mean_luma > 120)
//        {
//            txt_gray = 0;
//        }
//        circle(rgb_dst, Point(gy, gx), 1, Scalar(txt_gray, txt_gray, txt_gray), CV_FILLED, 8,0);

//        char overlay_txt[64];
//        memset(overlay_txt, 0, 64);

//        float font_size = 0.3;

//        // Overlays the number of the sPix to each sPix
//        switch (info_type)
//        {
//        case INFO_NONE:
//            overlay_name = "";
//            //  sprintf(overlay_txt, "%d", spix_idx);
//            break;

//        case INFO_ID:
//            overlay_name = "id";
//            sprintf(overlay_txt, "%d", spix_idx);
//            break;

//        case INFO_SIZE:
//            overlay_name = "size";
//            break;

//        case INFO_MEAN_LUMA:
//            overlay_name = "luma";
//            break;

//        case INFO_GRAVITY:
//            overlay_name = "gravity";
//            font_size = 0.25;
//            sprintf(overlay_txt, "%d,%d", gx, gy);
//            break;

//        case INFO_SIGNATURE:
//            overlay_name = "signature";
//            break;

//        case INFO_MATCHING:
//            overlay_name = "matching";

//            //            if (spix_idx < msa.m_vct_spix_matches.size())
//            //            {
//            //                const int nbr_matches = msa.m_vct_spix_matches[spix_idx].size();
//            //                sprintf(overlay_txt, "%d", nbr_matches);
//            //            }
//            break;
//        }

//        putText(rgb_dst, overlay_txt, cvPoint(gy - 8, gx + 10), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(txt_gray, txt_gray, txt_gray), 1, CV_AA);

//        for (int ooi_idx = 0; ooi_idx < msa.m_vct_ooi.size(); ooi_idx++)
//        {
//            for (int i = 0; i < msa.m_vct_ooi[ooi_idx].m_vct_spix.size(); i++)
//            {
//                // putText(rgb_dst, overlay_txt, cvPoint(gy - 8, gx + 10), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(txt_gray, txt_gray, txt_gray), 1, CV_AA);

//                if (msa.m_vct_ooi[ooi_idx].m_vct_spix[i] != spix_idx)
//                {
//                    //    putText(rgb_dst, overlay_txt, cvPoint(gy - 8, gx + 10), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(txt_gray, txt_gray, txt_gray), 1, CV_AA);
//                }
//                else
//                {
//                    // Recolors the contents of the sPix
//                    const SuperImage *p_simg = &(msa.m_vct_simg[simg_idx]);

//                    if (!p_simg->m_geo_img)
//                    {
//                        return false;
//                    }

//                    for (int px_idx = 0; px_idx < p_simg->m_vct_spix[spix_idx].m_spix_pos.size(); px_idx++)
//                    {
//                        const int r = p_simg->m_vct_spix[spix_idx].m_spix_pos[px_idx].m_r;
//                        const int c = p_simg->m_vct_spix[spix_idx].m_spix_pos[px_idx].m_c;

//                        if (rgb_dst.at<Vec3b>(r, c)[0] - 64 > 0)
//                        {
//                            rgb_dst.at<Vec3b>(r, c)[0] = rgb_dst.at<Vec3b>(r, c)[0] - 64;
//                        }

//                        if (rgb_dst.at<Vec3b>(r, c)[1] - 64 > 0)
//                        {
//                            rgb_dst.at<Vec3b>(r, c)[1] = rgb_dst.at<Vec3b>(r, c)[1] - 64;
//                        }

//                        if (rgb_dst.at<Vec3b>(r, c)[2] - 64 > 0)
//                        {
//                            rgb_dst.at<Vec3b>(r, c)[2] = rgb_dst.at<Vec3b>(r, c)[2] - 64;
//                        }
//                    }

//                    putText(rgb_dst, overlay_txt, cvPoint(gy - 8, gx + 10), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(0, 255, 255), 1, CV_AA);
//                }
//            }
//        }
//    }

//    char s_src_idx[64];
//    memset(s_src_idx, 0, 64);
//    sprintf(s_src_idx, "%04d", p_simg->m_id);

//    // Draws the set of User defined ROI
//    for (int ooi_idx = 0; ooi_idx < msa.m_vct_ooi.size(); ooi_idx++)
//    {
//        rectangle(rgb_dst, msa.m_vct_roi[ooi_idx], Scalar(0, 255, 255), 2);
//    }

//    string file_dst = path_dst;
//    file_dst += "slic_";
//    file_dst += string(overlay_name);
//    file_dst += "_";
//    file_dst += string(s_src_idx);
//    file_dst += ".jpg";
//    imwrite(file_dst.data(), rgb_dst);
//}

////-----------------------------------------------------------------------------

//bool SPixTools::dumpPairwiseOverlap(const GeometryMetrics &msa, const string path_dst, const int simg_idx_0, const int simg_idx_1, const vector<float> vct_spix_overlap)
//{
//    const Mat *p_rgb_src = &(msa.m_vct_rgb_dst[simg_idx_1]);
//    Mat rgb_dst = Mat::zeros(p_rgb_src->rows, p_rgb_src->cols, CV_8UC3);

//    // Copies the slic image to the dst image
//    p_rgb_src->copyTo(rgb_dst);

//    const SuperImage *p_simg = &(msa.m_vct_simg[simg_idx_1]);

//    if (!p_simg->m_geo_img)
//    {
//        return false;
//    }

//    char s_src_idx_0[64];
//    memset(s_src_idx_0, 0, 64);
//    sprintf(s_src_idx_0, "%d", simg_idx_0);

//    string file_dst = path_dst;
//    file_dst += "slic_crossframe_overlay";
//    file_dst += "_";
//    file_dst += string(s_src_idx_0);

//    for (int spix_idx = 0; spix_idx < p_simg->m_vct_spix.size(); spix_idx++)
//    {
//        const int gx = p_simg->m_vct_spix[spix_idx].m_gx;
//        const int gy = p_simg->m_vct_spix[spix_idx].m_gy;
//        int txt_gray = 255;

//        if (p_simg->m_vct_spix[spix_idx].m_mean_luma > 120)
//        {
//            txt_gray = 0;
//        }
//        circle(rgb_dst, Point(gy, gx), 1, Scalar(txt_gray, txt_gray, txt_gray), CV_FILLED, 8,0);

//        char overlay_txt[64];
//        memset(overlay_txt, 0, 64);

//        float font_size = 0.28;

//        float ovr_ratio = vct_spix_overlap[spix_idx];

//        if (ovr_ratio < 0.001)
//        {
//            ovr_ratio = 0.0;
//        }

//        ovr_ratio *= 100;
//        sprintf(overlay_txt, "%3.0f", ovr_ratio);
//        putText(rgb_dst, overlay_txt, cvPoint(gy - 8, gx + 10), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(txt_gray, txt_gray, txt_gray), 1, CV_AA);
//    }

//    char s_src_idx_1[64];
//    memset(s_src_idx_1, 0, 64);
//    sprintf(s_src_idx_1, "%d", simg_idx_1);

//    file_dst += "_";
//    file_dst += string(s_src_idx_1);
//    file_dst += ".jpg";
//    imwrite(file_dst.data(), rgb_dst);
//}

////-----------------------------------------------------------------------------

//bool SPixTools::dumpAllPairwiseOverlaps(const GeometryMetrics &msa, const string path_dst)
//{
//    string overlay_name = "";

//    // int simg_idx_0 = 0;
//    for (int simg_idx_0 = 0; simg_idx_0 < msa.m_vct_simg.size(); simg_idx_0++)
//    {
//        for (int simg_idx_1 = 0; simg_idx_1 < msa.m_vct_simg.size(); simg_idx_1++)
//        {
//            if (simg_idx_0 != simg_idx_1)
//            {
//                const Mat *p_rgb_src = &(msa.m_vct_rgb_dst[simg_idx_0]);
//                Mat rgb_dst = Mat::zeros(p_rgb_src->rows, p_rgb_src->cols, CV_8UC3);

//                // Copies the slic image to the dst image
//                p_rgb_src->copyTo(rgb_dst);

//                const SuperImage *p_simg = &(msa.m_vct_simg[simg_idx_0]);

//                if (!p_simg->m_geo_img)
//                {
//                    return false;
//                }

//                for (int spix_idx = 0; spix_idx < p_simg->m_vct_spix.size(); spix_idx++)
//                {
//                    const int gx = p_simg->m_vct_spix[spix_idx].m_gx;
//                    const int gy = p_simg->m_vct_spix[spix_idx].m_gy;
//                    int txt_gray = 255;

//                    if (p_simg->m_vct_spix[spix_idx].m_mean_luma > 120)
//                    {
//                        txt_gray = 0;
//                    }
//                    circle(rgb_dst, Point(gy, gx), 1, Scalar(txt_gray, txt_gray, txt_gray), CV_FILLED, 8,0);

//                    char overlay_txt[64];
//                    memset(overlay_txt, 0, 64);

//                    float font_size = 0.24;
//                    overlay_name = "overlay_ratio";

//                    float ovr_ratio = msa.m_vct_simg[simg_idx_0].m_vct_spix[spix_idx].m_vct_overlap_ratio[simg_idx_1];

//                    if (spix_idx == 125)
//                    {
//                        float a = ovr_ratio;
//                        a++;
//                    }

//                    if (ovr_ratio < 0.001)
//                    {
//                        ovr_ratio = 0.0;
//                    }

//                    // ovr_ratio *= 100;
//                    sprintf(overlay_txt, "%3.3f", ovr_ratio);
//                    putText(rgb_dst, overlay_txt, cvPoint(gy - 12, gx + 10), FONT_HERSHEY_SIMPLEX, font_size, cvScalar(txt_gray, txt_gray, txt_gray), 1, CV_AA);
//                }
//                char s_src_idx_0[64];
//                memset(s_src_idx_0, 0, 64);
//                sprintf(s_src_idx_0, "%d", simg_idx_0);

//                char s_src_idx_1[64];
//                memset(s_src_idx_1, 0, 64);
//                sprintf(s_src_idx_1, "%d", simg_idx_1);

//                string file_dst = path_dst;
//                file_dst += "slic_";
//                file_dst += string(overlay_name);
//                file_dst += "_";
//                file_dst += string(s_src_idx_0);
//                file_dst += "_";
//                file_dst += string(s_src_idx_1);
//                file_dst += ".jpg";
//                imwrite(file_dst.data(), rgb_dst);
//            }
//        }
//    }
//}

////-----------------------------------------------------------------------------

//bool SPixTools::dumpSpixImageStack(const GeometryMetrics &msa,
//                                   const vector<SuperImage> &vct_simg,
//                                   const string path_dst,
//                                   const enum SPixInfoOverlay info_type)
//{
//    for (int simg_idx = 0; simg_idx < msa.m_vct_simg.size(); simg_idx++)
//    {
//        if (dumpSpixImage(msa, simg_idx, path_dst, info_type) == false)
//        {
//            return false;
//        }
//    }
//    return true;
//}

////-----------------------------------------------------------------------------

//bool SPixTools::dumpSPixDistanceVectors(Mat rgb_src, GeometryMetrics &msa, const int simg_idx, const int spix_idx,  vector<float *> vct_label_dist_min)
//{
//    // Dumps and plots the image of the distance vector among paired spix couples
//    Mat rgb_dst = Mat::zeros(rgb_src.rows, rgb_src.cols, CV_8UC3);

//    for (int spix_idx = 0; spix_idx < msa.m_vct_simg[simg_idx].m_vct_spix.size(); spix_idx++)
//    {
//        int simg_idx = 1;
//        // int for (int simg_idx = 1; simg_idx < m_msa.m_vct_simg[simg_idx_0].size() - 1; simg_idx++)
//        {
//            const int gx_0 = msa.m_vct_simg[simg_idx].m_vct_spix[spix_idx].m_gx;
//            const int gy_0 = msa.m_vct_simg[simg_idx].m_vct_spix[spix_idx].m_gy;

//            int label_dist_min = vct_label_dist_min[simg_idx][spix_idx];
//            const int gx_1 = msa.m_vct_simg[simg_idx].m_vct_spix[label_dist_min].m_gx + msa.m_vct_src_shift[simg_idx].x;
//            const int gy_1 = msa.m_vct_simg[simg_idx].m_vct_spix[label_dist_min].m_gy + msa.m_vct_src_shift[simg_idx].y;

//            Point p_0 = Point(gy_0, gx_0);
//            Point p_1 = Point(gy_1, gx_1);

//            if (abs(gx_1 - gx_0) > 80 || abs(gy_1 - gy_0) > 80)
//            {
//                arrowedLine(rgb_dst, p_1, p_0, Scalar(rand() % 255, rand() % 255, rand() % 255), 1, 8, 0, 0.02);
//            }
//        }
//    }
//    // cout << "sPix match: (" << spix_idx << " - " << matching_label << ")" << endl;
//    return true;
//}
