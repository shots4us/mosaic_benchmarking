#ifndef TEST
#define TEST

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <math.h>
#include "tools.h"

// #include <QCoreApplication>
// #include <QDebug>

using namespace cv;
using namespace std;

typedef unsigned int uint;

// This class performs a watershed segmentation using the Soille algorithm (with hierarchical //queues). It works by default on Byte resolution. The maximum number of created segment  is // 2^31-1. It return an IntegerImage, the first segment as label Integer.MIN_VALUE.
// @author Aptoula, Derivaux, Weber
class Watershed
{
public:

    // The input image
    // The resolution considered, 8 by default
    const int resolution = 8;

    bool TRACE;
    bool RGI;

    Mat inputImage;

    // The output image
    Mat outputImage;

    Fifo fifo;

    unsigned int h[256];

    // A constant to represent watershed lines
    const int WSHED = 0;
    const int INIT = -1;
    const int MASK = -2;

    Point fictitious;

public:
    // Constructor

    Watershed(Mat in, Mat markers, Mat g, unsigned int k, Mat &out);
    void launch( Mat markers, Mat g, unsigned int k);
    bool ifEltDiff(Mat input, int val);
    bool areThereLabelledNeighbours(Mat img, Point p0);
    void calculateDistro(Mat &img, vector<Point> distro[]);
};
#endif // TEST

