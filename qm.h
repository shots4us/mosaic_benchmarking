#ifndef QM_H
#define QM_H

#include "global.h"
#include <iostream>
#include <sstream>
#include <math.h>
#include <stdint.h>

using namespace cv;

namespace qm
{
double sigma(Mat & m, int i, int j, int block_size);
double cov(Mat & m1, Mat & m2, int i, int j, int block_size);
double eqm(Mat & img1, Mat & img2);float eqm_vct(vector<float> vct_in);
float eqm_vct(vector<float> vct_in);
double psnr(Mat & img_src, Mat & img_compressed, int block_size);
double ssim(Mat & img_src, Mat & img_compressed, int block_size, bool show_progress = false);
void compute_quality_metrics(char * file1, char * file2, int block_size);
void compute_quality_metrics_2(const Mat rgb_src_1, const Mat rgb_src_2, int block_size, double &psnr_val, double &ssim_val);
}

#endif // QM_H
