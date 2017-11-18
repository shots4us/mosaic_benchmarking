#include <stdio.h>
#include "tools.h"

#define FILE_PATH "D:/PRJ/ALGO_SLIC/Dump"

void dump_cv(char *name, Mat grayIn)
{
    const char *path = "D:/PRJ/LITTORALG/Docs/Morphological image compositing";
    char f_name[256];
    sprintf(f_name, "%s/%s.txt", path, name);
    FILE *fid=fopen(f_name,"w+");

    for (int i=0;i<grayIn.rows;i++)
    {
        for (int j=0;j<grayIn.cols;j++)
        {
            fprintf(fid,"(%d:%d) %f\n",i + 1, j + 1, grayIn.at<float>(Point(i,j)));
        }
        fprintf(fid,"\n");
    }
    fclose(fid);
}

void dump8_cv(char *name, Mat grayIn)
{
    const char *path = "D:/PRJ/LITTORALG/Docs/Morphological image compositing";
    char f_name[256];
    sprintf(f_name, "%s/%s.txt", path, name);
    FILE *fid=fopen(f_name,"w+");

    for (int i=0;i<grayIn.rows;i++)
    {
        for (int j=0;j<grayIn.cols;j++)
        {
            int val = (int)grayIn.at<uint8_t>(i, j);
            fprintf(fid,"(%d:%d) %d\n" ,i, j, val);
            fflush(fid);
        }

        fprintf(fid,"\n");
    }
    fclose(fid);
}

void dump_matrix_8_cv(char *name, const uint idx, Mat grayIn)
{
    const char *path = "D:/PRJ/Benchmark/Dump";
    char f_name[256];
    sprintf(f_name, "%s/%s_%03d.txt", path, name, idx);
    FILE *fid=fopen(f_name,"w+");

    for (int i=0;i<grayIn.rows;i++)
    {
        for (int j=0;j<grayIn.cols;j++)
        {
            int val = (int)grayIn.at<int8_t>(i, j);
            fprintf(fid, "%03ld ", val);
            fflush(fid);
        }

        fprintf(fid,"\n");
    }
    fclose(fid);
}

void dump_matrix_32_cv(char *name, const uint idx, Mat grayIn)
{
    const char *path = "D:/PRJ/Dump/";
    char f_name[256];
    sprintf(f_name, "%s/%s_%05d.txt", path, name, idx);
    FILE *fid=fopen(f_name,"w+");

    for (int i=0;i<grayIn.rows;i++)
    {
        for (int j=0;j<grayIn.cols;j++)
        {
            int val = (int)grayIn.at<int32_t>(i, j);
            fprintf(fid, "%d ", val);
            fflush(fid);
        }

        fprintf(fid,"\n");
    }
    fclose(fid);
}

void dump_matrix_f32_cv(char *name, const uint idx, Mat grayIn)
{
    const char *path = FILE_PATH;
    char f_name[256];
    sprintf(f_name, "%s/%s_%05d.txt", path, name, idx);
    FILE *fid=fopen(f_name,"w+");

    for (int i=0;i<grayIn.rows;i++)
    {
        for (int j=0;j<grayIn.cols;j++)
        {
            float val = grayIn.at<float>(i, j);
            fprintf(fid, "%.6f ", val);
            fflush(fid);
        }

        fprintf(fid,"\n");
    }
    fclose(fid);
}
//------------------------------------------------------------------------------------------------

void dump_complex_cv(char *name, Mat in)
{
    Mat planes[] = {Mat_<double>(in), Mat::zeros(Size(in.cols, in.rows), CV_64F)};
    split(in, planes);
    const char *path = "D:/PRJ/LITTORALG/Docs/Morphological image compositing/log.jpg";
    char f_name[256];

    if (planes[0].rows != planes[1].rows)
    {
        return;
    }
    sprintf(f_name, "%s/%s.txt", path, name);
    FILE *fid=fopen(f_name,"w+");

    for
            (int i=0;i<planes[0].rows;i++)
    {
        for (int j=0;j<planes[0].cols;j++)
        {
            fprintf(fid, "(%d:%d) %f + i%f\n", i, j, planes[0].at<double>(i,j), planes[1].at<double>(i,j));
        }
        fprintf(fid,"\n");
    }
    fclose(fid);
}

//------------------------------------------------------------------------------------------------

const string currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tstruct);

    return buf;
}

//------------------------------------------------------------------------------------------------

double diffclock(clock_t clock1,clock_t clock2)
{
    double diffticks=clock1-clock2;
    double diffms=(diffticks)/(CLOCKS_PER_SEC/1000);
    return diffms;
}

//------------------------------------------------------------------------------------------------------------

bool selectRandomRegion(const Mat rgb_src, Mat &rgb_dst, const int h_0, const int w_0)
{
    if (rgb_src.rows < h_0 || rgb_src.cols < w_0)
    {
        cout << "Image (" << rgb_src.rows << "," << rgb_src.cols << ") is smaller than ROI " << h_0 << "," << w_0 << " size. Image skipped !!" << endl;
        return false;
    }

    const int border_x = 8;
    const int border_y = 8;

    const int max_x = (rgb_src.rows - h_0 - border_x);
    const int max_y = (rgb_src.cols - w_0 - border_y);

    const int x_0 = rand() % (max_x);
    const int y_0 = rand() % (max_y);

    cout << "------------------------------------------------------------------------------" << endl;
    cout << "Image (" << rgb_src.rows << ", " << rgb_src.cols << ") - Extracts a random Region (" << x_0 << ", " << y_0 << ", " << h_0 << ", " << w_0 << ")" << endl;
    cout << "------------------------------------------------------------------------------" << endl;

    const Mat roi(rgb_src, Rect(y_0, x_0, h_0, w_0));
    resize(roi, rgb_dst, roi.size(), 0, 0, INTER_CUBIC);
    return true;
}

//------------------------------------------------------------------------------------------------------------

bool selectCenterRegion(const Mat rgb_src, Mat &rgb_dst, const int min_img_size)
{
    if (min(rgb_src.rows, rgb_src.cols) < min_img_size)
    {
        cout << "Image (" << rgb_src.rows << "," << rgb_src.cols << ") is smaller than ROI " << min_img_size << "," << min_img_size << " size. Image skipped !!" << endl;
        return false;
    }

    const int border_x = 0;
    const int border_y = 0;

    const int max_x = rgb_src.rows - min_img_size - border_x;
    const int max_y = rgb_src.cols - min_img_size - border_y;

    const int x_0 = max_x / 2;
    const int y_0 = max_y / 2;

    cout << "------------------------------------------------------------------------------" << endl;
    cout << "Image (" << rgb_src.rows << ", " << rgb_src.cols << ") - Extracts a random Region (" << x_0 << ", " << y_0 << ", " << min_img_size << ", " << min_img_size << ")" << endl;
    cout << "------------------------------------------------------------------------------" << endl;

    const Mat roi(rgb_src, Rect(y_0, x_0, min_img_size, min_img_size));
    roi.copyTo(rgb_dst);
    return true;
}
