#include "hsvmanager.h"

HSVManager::HSVManager()
{
    m_range_min = 0.0;
    m_range_max = 0.0;
    m_step = 0.0;
}

HSVManager::HSVManager(const float range_min, const float range_max, const float step)
{
    m_range_min = range_min;
    m_range_max = range_max;
    m_step = step;
}

//------------------------------------------------------------------------------------------------------------

HSVManager::~HSVManager()
{
}

//------------------------------------------------------------------------------------------------------------

bool HSVManager::unitTest(const HSVTest hsv_tst, const uint32_t cycles)
{
    Mat rgb_src = imread("D:/PRJ/Benchmark/Nominal/Fiat5001_small.png", CV_LOAD_IMAGE_COLOR);
    Mat rgb_dst = Mat::zeros(rgb_src.size(), CV_8UC3);
    string f_out = "Img_";

    switch (hsv_tst)
    {
    case TestLuminance:
        // Luminance
        m_range_min = -100.0;
        m_range_max = 100.0;
        m_step = (m_range_max - m_range_min) / cycles;
        break;

    case TestSaturation:
        // Saturation
        m_range_min = -1.0;
        m_range_max = 1.0;
        m_step = (m_range_max - m_range_min) / cycles;
        break;

    case TestHue:
        // Hue
        m_range_min = -180.0;
        m_range_max = 180.0;
        m_step =(m_range_max - m_range_min) / cycles;
        break;
    }

    int file_cnt = 0;

    string distort_name = "";

    for (float k = m_range_min; k < m_range_max; k += m_step)
    {
        switch (hsv_tst)
        {
        case TestLuminance:
            distort_name = "Luminance";
            if (modifyLuminance(rgb_src, rgb_dst, (float)k) == false)
            {
                return false;
            }
            break;

        case TestSaturation:
            distort_name = "Saturation";
            if (modifySaturation(rgb_src, rgb_dst, (float)k) == false)
            {
                return false;
            }
            break;

        case TestHue:
            distort_name = "Hue";
            if (modifyHue(rgb_src, rgb_dst, (float)k) == false)
            {
                return false;
            }
            break;
        }

        char s_str[256];
        if (hsv_tst == TestLuminance)
        {
            sprintf(s_str, "%d", (int)k);
        }
        else if (hsv_tst == TestSaturation)
        {
            sprintf(s_str, "%d", (int)(k * 100));
        }
        else //  if (hsv_tst == TestHue)
        {
            sprintf(s_str, "%d", (int)k);
        }
        imwrite((f_out + string(s_str) + ".png").c_str(), rgb_dst);

        const long n = rgb_src.rows * rgb_src.cols;

        for (uint i = 0; i < 3; i++)
        {
            m_clip_ratio[i] = m_clip_cnt[i] / (float)(n);
        }

        const float in_range_ratio = m_clip_ratio[0];
        const float clipped_ratio = m_clip_ratio[1] + m_clip_ratio[2];

        char s_cnt[3];
        sprintf(s_cnt, "%03d", (uint)(file_cnt));

        char s_k[4];
        sprintf(s_k, "%.03f", (float)k);

        if (clipped_ratio <= 0.001)
        {
#ifdef IS_VERBOSE
            cout << s_cnt << "  Strength: " << s_k << "  In/Clip " << in_range_ratio << "/" << clipped_ratio << " -> No clipping !!" << endl;
#endif
        }
        else if (clipped_ratio > 0.00 && clipped_ratio <= 0.2)
        {
#ifdef IS_VERBOSE
            cout << s_cnt << "  Strength: " << s_k << "  In/Clip " << in_range_ratio << "/" << clipped_ratio << " -> Clipping Low !!" << endl;
#endif
        }
        else if (clipped_ratio > 0.2 && clipped_ratio < 0.5)
        {
#ifdef IS_VERBOSE
            cout << s_cnt << "  Strength: " << s_k << "  In/Clip " << in_range_ratio << "/" << clipped_ratio << " -> Clipping Moderate !!" << endl;
#endif
        }
        else // if (clipping_ratio > 0.5)
        {
#ifdef IS_VERBOSE
            cout << s_cnt << "  Strength: " << s_k << "  In/Clip " << in_range_ratio << "/" << clipped_ratio << " -> Clipping Severe !!" << endl;
#endif
        }

        file_cnt++;
        imwrite("D:/PRJ/Benchmark/Distorted/TestHSV/" + distort_name + "/" + string(s_cnt) + "_" + string(s_k) + ".png",  rgb_dst);
    }

    return true;
}

//------------------------------------------------------------------------------------------------------------
// HSV modifiy methods
// Input/Output matrix is Mat rgb CV_8UC3 format
//
// Input valid range is:
// 1. Luminance: [0..255]
// 2. Saturation: [0..1]
// 3. Hue: [-180..180]
//------------------------------------------------------------------------------------------------------------
bool HSVManager::modifyLuminance(const Mat rgb_src, Mat &rgb_distort, const float strength)
{
    Mat rgb_float(rgb_src.size(), CV_32FC3);
    rgb_float.setTo(0);

    Mat hsv = Mat::zeros(rgb_src.size(), CV_32FC3);
    rgb_src.convertTo(rgb_float, CV_32FC3);
    cvtColor(rgb_float, hsv, COLOR_RGB2HSV);

    Mat channel[3];
    channel[0] = Mat::zeros(rgb_src.size(), CV_32FC1);
    channel[1] = Mat::zeros(rgb_src.size(), CV_32FC1);
    channel[2] = Mat::zeros(rgb_src.size(), CV_32FC1);
    split(hsv, channel);

    float *p_px = (float *)channel[2].data;
    float *p_base = p_px;
    long n = channel[2].rows * channel[2].cols;

    for (p_px = p_base; p_px < p_base + n; p_px++)
    {
        if (p_px == 0)
        {
            return false;
        }

        if (*p_px + strength < 0.0)
        {
            // Down range Clipping
            //  *p_px = 0.0;

#ifdef DEBUG_MODE
            m_clip_cnt[0]++;
#endif
        }
        else if (*p_px + strength > 255)
        {
            // Up range Clipping
            //   *p_px = 255;

#ifdef DEBUG_MODE
            m_clip_cnt[1]++;
#endif
        }
        // else
        {
            *p_px += strength;

#ifdef DEBUG_MODE
            m_clip_cnt[2]++;
#endif
        }
    }

    rgb_distort = Mat::zeros(rgb_src.size(), CV_8UC3);
    merge(channel, 3, rgb_float);
    cvtColor(rgb_float, rgb_float, COLOR_HSV2RGB);
    rgb_float.convertTo(rgb_distort, CV_8U);
    return true;
}

//------------------------------------------------------------------------------------------------------------

bool HSVManager::modifySaturation(const Mat rgb_src, Mat &rgb_distort, const float strength)
{
    Mat rgb_float(rgb_src.size(), CV_32FC3);
    rgb_float.setTo(0);

    Mat hsv = Mat::zeros(rgb_src.size(), CV_32FC3);
    rgb_src.convertTo(rgb_float, CV_32FC3);
    cvtColor(rgb_float, hsv, COLOR_RGB2HSV);

    Mat channel[3];
    channel[0] = Mat::zeros(rgb_src.size(), CV_32FC1);
    channel[1] = Mat::zeros(rgb_src.size(), CV_32FC1);
    channel[2] = Mat::zeros(rgb_src.size(), CV_32FC1);
    split(hsv, channel);

    float *p_px = (float *)channel[1].data;
    float *p_base = p_px;
    long n = channel[1].rows * channel[1].cols;

    for (p_px = p_base; p_px < p_base + n; p_px++)
    {
        if (p_px == 0)
        {
            return false;
        }

        if (*p_px + strength < 0.0)
        {
            // Down range Clipping
            *p_px = 0.0;

#ifdef DEBUG_MODE
            m_clip_cnt[0]++;
#endif
        }
        else if (*p_px + strength >= m_range_max)
        {
            // Up range Clipping
            *p_px = m_range_max;

#ifdef DEBUG_MODE
            m_clip_cnt[1]++;
#endif
        }
        else
        {
            *p_px += strength;

#ifdef DEBUG_MODE
            m_clip_cnt[2]++;
#endif
        }
    }

    rgb_distort = Mat::zeros(rgb_src.size(), CV_8UC3);
    merge(channel, 3, rgb_float);
    cvtColor(rgb_float, rgb_float, COLOR_HSV2RGB);
    rgb_float.convertTo(rgb_distort, CV_8U);
    return true;
}

//------------------------------------------------------------------------------------------------------------

bool HSVManager::modifyHue(const Mat rgb_src, Mat &rgb_distort, const float strength)
{
    Mat rgb_float(rgb_src.size(), CV_32FC3);
    rgb_float.setTo(0);

    Mat hsv = Mat::zeros(rgb_src.size(), CV_32FC3);
    rgb_src.convertTo(rgb_float, CV_32FC3);
    cvtColor(rgb_float, hsv, COLOR_RGB2HSV);

    Mat channel[3];
    channel[0] = Mat::zeros(rgb_src.size(), CV_32FC1);
    channel[1] = Mat::zeros(rgb_src.size(), CV_32FC1);
    channel[2] = Mat::zeros(rgb_src.size(), CV_32FC1);
    split(hsv, channel);

    float *p_px = (float *)channel[0].data;
    float *p_base = p_px;
    long n = channel[0].rows * channel[0].cols;

    for (p_px = p_base; p_px < p_base + n; p_px++)
    {
        if (p_px == 0)
        {
            return false;
        }

        if (*p_px + strength < 0.0)
        {
            // Down range Clipping
            *p_px = 0.0;

#ifdef DEBUG_MODE
            m_clip_cnt[0]++;
#endif
        }
        else if (*p_px + strength >= m_range_max * 2)
        {
            // Up range Clipping
            *p_px = m_range_max;

#ifdef DEBUG_MODE
            m_clip_cnt[1]++;
#endif
        }
        else
        {
            *p_px += strength;

#ifdef DEBUG_MODE
            m_clip_cnt[2]++;
#endif
        }
    }
    rgb_distort = Mat::zeros(rgb_src.size(), CV_8UC3);
    merge(channel, 3, rgb_float);
    cvtColor(rgb_float, rgb_float, COLOR_HSV2RGB);
    rgb_float.convertTo(rgb_distort, CV_8U);
    return true;
}
