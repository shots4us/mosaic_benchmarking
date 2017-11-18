#ifndef TOOLS
#define TOOLS

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <ctime>
#include <math.h>
#include <stdint.h>

// #include <QCoreApplication>
// #include <QDebug>

using namespace cv;
using namespace std;

typedef uint32_t uint;

class Fifo
{
public:
    vector<Point> v;

    Fifo()
    {
        v = vector<Point>();
    }

    void add(Point p)
    {
        v.push_back(p);
    }

    Point retrieve()
    {
        // Removes the item 0
        Point o = v[0];
        v.erase(v.begin());
        return o;
    }

    bool isEmpty()
    {
        return (v.size() == 0);
    }

    int size()
    {
        return v.size();
    }

    void reset()
    {
        v.clear();
    }
};

class FifoInt32
{
public:
    vector<uint32_t> v;

    FifoInt32()
    {
        v = vector<uint32_t>();
    }

    void add(uint32_t p)
    {
        v.push_back(p);
    }

    uint32_t retrieve()
    {
        // Removes the item 0
        uint32_t o = v[0];
        v.erase(v.begin());
        return o;
    }

    bool isEmpty()
    {
        return (v.size() == 0);
    }

    int size()
    {
        return v.size();
    }

    void reset()
    {
        v.clear();
    }
};


class QueueInt32
{
public:
    vector<int32_t> v;

    QueueInt32()
    {
        v = vector<int32_t>();
    }

    void add(int32_t p)
    {
        v.push_back(p);
    }

    int32_t get(uint32_t i)
    {
       return v[i];
    }

    int32_t retrieve()
    {
        // Removes the item 0
        int32_t o = v[0];
        v.erase(v.end());
        return o;
    }

    bool isEmpty()
    {
        return (v.size() == 0);
    }

    int size()
    {
        return v.size();
    }

    void reset()
    {
        v.clear();
    }
};

class Queue
{
public:
    vector<Point> v;

    Queue()
    {
        v = vector<Point>();
    }

    void add(Point p)
    {
        v.push_back(p);
    }

    Point get(uint32_t i)
    {
       return v[i];
    }

    Point retrieve()
    {
        // Removes the item 0
        Point o = v[0];
        v.erase(v.end());
        return o;
    }

    bool isEmpty()
    {
        return (v.size() == 0);
    }

    int size()
    {
        return v.size();
    }

    void reset()
    {
        v.clear();
    }
};

//---------------------------------------------------------------
// Begin of Debugging functions
//---------------------------------------------------------------
void dump_cv(char *name, Mat grayIn);
void dump8_cv(char *name, Mat grayIn);
void dump_matrix_8_cv(char *name, const uint idx, Mat grayIn);
void dump_matrix_32_cv(char *name, const uint idx, Mat grayIn);
void dump_matrix_f32_cv(char *name, const uint idx, Mat grayIn);
void dump_complex_cv(char *name, Mat in);
const string currentDateTime();
double diffclock(clock_t clock1,clock_t clock2);
bool selectRandomRegion(const Mat rgb_src, Mat &rgb_dst, const int h_0, const int y_0);
bool selectCenterRegion(const Mat rgb_src, Mat &rgb_dst, const int min_img_size);
//---------------------------------------------------------------
// End of Debugging functions
//---------------------------------------------------------------

#endif // TOOLS

