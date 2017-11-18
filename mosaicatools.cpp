////-------------------------------------------------------------------------------------
//// Project name: COHERENT MOSAICA
////
//// Creation Date: 20/03/2016
//// Created by: Roberto Giudici - OBELIX, UBS, IRISA, COSTEL, Universiy of Rennes 2
//// Description: Super pixel time coherent based Mosaicing algorithm
////-------------------------------------------------------------------------------------

//#include "mosaicatools.h"

//#ifdef IS_WIN
//#include <direct.h>
//#endif

//#ifdef  IS_LINUX
//#include <errno.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#endif

//bool dumpPropagationTree2(const Mosaica &msa, const vector<ObjPath> &spix_path)
//{
//    for (int i = 0; i < spix_path.size(); i++)
//    {
//        // dumpPropagationPaths(spix_path[i]);
//        return true;
//    }
//}

//bool dumpPropagationTree(Mosaica *msa, vector<ObjPath> &vct_path, const int dst_rgb_idx)
//{
//    Mat dst_rgb(msa->m_vct_rgb_src[dst_rgb_idx].size(), msa->m_vct_rgb_src[dst_rgb_idx].type());

//    Mat mask1 = Mat::zeros(dst_rgb.size(), dst_rgb.type());
//    msa->m_vct_rgb_src[dst_rgb_idx].copyTo(dst_rgb);
//    Mat dst_rgb0;
//    addWeighted(dst_rgb, 0.2, mask1, 1.0, 1.0, dst_rgb0);

//    // Obj Node Loop
//    for (int i = 0; i < vct_path.size(); i++)
//    {
//        vector<ObjNode> obj_node = vct_path[i].m_vct_pair;

//        // sImg Loop
//        for (int j = 0; j < obj_node.size(); j++)
//        {
//            dst_rgb0.copyTo(dst_rgb);

//            //  sp[j].simg_id; and j have the same contents !!
//            // const int simg_idx = sp[j].simg_id;
//            const int spix_idx = obj_node[j].spix_id;
//            SuperPixel *p_spix = &(msa->m_vct_simg[j].m_vct_spix[spix_idx]);

//            for (int px_idx = 0; px_idx < p_spix->m_spix_pos.size(); px_idx++)
//            {
//                const int r = p_spix->m_spix_pos[px_idx].m_r;
//                const int c = p_spix->m_spix_pos[px_idx].m_c;

//                dst_rgb.at<Vec3b>(r, c)[0] = msa->m_vct_rgb_src[j].at<Vec3b>(r, c)[0];
//                dst_rgb.at<Vec3b>(r, c)[1] = msa->m_vct_rgb_src[j].at<Vec3b>(r, c)[1];
//                dst_rgb.at<Vec3b>(r, c)[2] = msa->m_vct_rgb_src[j].at<Vec3b>(r, c)[2];
//            }
//            char val[128];
//            memset(val, 0, 128);
//            sprintf(val, "%d", dst_rgb_idx);
//            string file_out = "D:/PRJ/ALGO_SLIC/REPLACE/IMG_";
//            file_out += val;

//#ifdef IS_WIN
//            if (mkdir(file_out.c_str()) != 0 && errno != EEXIST)
//            {
//                cout << "cannot create folder [" << file_out << "] : " << strerror(errno) << endl;
//            }
//#endif

//#ifdef IS_LINUX
//            if (mkdir(file_out.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0 && errno != EEXIST)
//            {
//                cout << "cannot create folder [" << file_out << "] : " << strerror(errno) << endl;
//            }
//#endif

//            file_out += "/";
//            memset(val, 0, 128);
//            sprintf(val, "%d_%d_%d", dst_rgb_idx, i,  j);

//            file_out += val;
//            file_out += ".jpg";
//            imwrite(file_out, dst_rgb);
//        }
//    }
//    return true;
//}
