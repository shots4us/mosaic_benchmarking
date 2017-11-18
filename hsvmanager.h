#ifndef HSVMANAGER_H
#define HSVMANAGER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <ctime>
#include <sstream>
#include <vector>
#include <stdint.h>

using namespace std;
using namespace cv;

#define DEBUG_MODE 1

typedef unsigned int uint;

enum HSVTest
{
    TestLuminance = 0,
    TestSaturation,
    TestHue
};

class HSVManager
{
public:
    HSVManager();
    HSVManager(const float range_min, const float range_max, const float step);
    ~HSVManager();

    bool unitTest(const HSVTest hsv_tst, const uint32_t cycles);
    bool modifyLuminance(const Mat rgb_src, Mat &rgb_distort, const float strength);
    bool modifySaturation(const Mat rgb_in, Mat &rgb_distort, const float strength);
    bool modifyHue(const Mat rgb_in, Mat &rgb_distort, const float strength);


// protected:
    float m_range_min;
    float m_range_max;
    float m_stength;
    float m_step;

    uint m_clip_cnt[3];
    uint m_clip_ratio[3];

   vector<Mat> m_distort_img;

    // private:
};

#endif // HSVMANAGER_H
