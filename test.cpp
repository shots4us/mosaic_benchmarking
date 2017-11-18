#include "test.h"
#include "tools.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <math.h>

#include <QCoreApplication>
#include <QDebug>

using namespace cv;
using namespace std;

Watershed::Watershed(Mat in, Mat markers, Mat g, unsigned int k, Mat &out)
{
    fictitious = Point(-1, -1);
    inputImage = in;
    out = out;

    if (TRACE)
    {
        for (int i=0;i<256;i++)
        {
            h[i]=0;
        }
    }
}

void Watershed::launch(Mat markers, Mat g, unsigned int k)
{
    Mat dist(inputImage.size(), CV_32S);

    // labelled watershed image
    Mat workOut(inputImage.size(), CV_8S);

    outputImage = Mat(inputImage.size(), CV_8U);

    // Initialise the workout image
    workOut.setTo(INIT);
    dist.setTo(0);
    int currentLabel = WSHED;
    Point p;

    TRACE = true;
    RGI = true;

    // pixel value distribution,

    // Chercher une methode pour ne pas boucler tout l'intervalle, mais s'arreter à la denière valeur trouvée
    vector<Point>distro[256];
    calculateDistro(inputImage, distro);

    if (TRACE)
    {
        //dump8_cv("imageIn", inputImage);
    }

    // start flooding
    for (int i = 0; i < 256; i++)
    {
        if (TRACE)
        {
            cout << "Flooding: " << i << endl;
        }

        // geodesic SKIZ of level i - 1 inside level i
        int size = distro[i].size();

        // Reads the list of pixels corresponding to values in the range [0..255]
        for (int j = 0; j < size; j++)
        {
            // Begin RGI Composition Addon
            // Floads only pixels having g[p] = k
            //if (g.at<uint8_t>(p.x, p.y) != k)
            // {
            //     continue;
            // }
            // End RGI Composition Addon

            p = distro[i][j];

            workOut.at<qint8>(p.x, p.y) = MASK;

            if (areThereLabelledNeighbours(workOut, p) == true)
            {
                // If any labelled neighbour is detected the pixel is isoltaed and will be discarded
                dist.at<int>(p.x, p.y) = 1;
                fifo.add(p);

                if (TRACE)
                {
                    cout << "adding p to FIFO: " << p.x << " " << p.y << endl;
                }
            }
        }

        if (TRACE && size > 0)
        {
            cout << "disti ITERA I " << i << "  currentLabel" << currentLabel;
        }

        int curDist = 1;
        fifo.add(fictitious);

        int sizeFiFO = fifo.size();

        // Process all the pixels at fleding level = i
        while(true)
        {
            p = fifo.retrieve();

            if (TRACE)
            {
                cout << "Retrieving from FIFO: " << p.x << " " << p.y << endl;
            }

            if (p.x == -1 && p.y == -1)
            {
                if (fifo.isEmpty() == true)
                {
                    break;
                }
                else
                {
                    fifo.add(fictitious);
                    curDist++;
                    p = fifo.retrieve();
                }
            }

            // labelling p by inspecting its neighbours
            for (int r = p.x - 1; r <= p.x + 1; r++)
            {
                for (int c = p.y - 1; c <= p.y + 1; c++)
                {
                    if (r < 0 || r >= inputImage.rows
                            || c < 0
                            || c >= inputImage.cols)
                        continue;

                    // if the pixel is already labelled
                    if (!(r == p.x && c == p.y) && (dist.at<int>(r, c) < curDist) && workOut.at<qint8>(r, c) > WSHED)
                    {
                        if (workOut.at<qint8>(r, c) > 0)
                        {
                            if (workOut.at<qint8>(p.x, p.y) == MASK || workOut.at<qint8>(p.x, p.y) == WSHED)
                            {
                                workOut.at<qint8>(p.x, p.y) = workOut.at<qint8>(r, c);
                            }
                            // Marker suspervised watersheding
                            else if (workOut.at<qint8>(p.x, p.y) != workOut.at<qint8>(r, c))
                            {
                                // Begin RGI Composition Addon
                                if (RGI)
                                {
                                    if (markers.at<qint8>(p.x, p.y) != 0)
                                    {
                                            workOut.at<qint8>(p.x, p.y) = WSHED;
                                    }
                                    else
                                    {
                                        // Overlapping the same value on different images
                                        // Different labels can be melted to one single CB
                                        workOut.at<qint8>(p.x, p.y) = workOut.at<qint8>(r, c);
                                    }
                                }
                                else
                                {
                                    // Original code
                                    workOut.at<qint8>(p.x, p.y) = WSHED;
                                }
                                // End RGI Composition Addon


                            }
                        }
                        else if ( workOut.at< char>(p.x, p.y) == MASK)
                        {
                            workOut.at<qint8>(p.x, p.y) = WSHED;
                        }
                    }
                    else if (workOut.at< char>(r, c) == MASK && dist.at<int>(r, c) == 0)
                    {
                        // if the neighbour is a plateau pixel
                        dist.at<int>(r, c) = curDist + 1;
                        fifo.add(Point(r, c));
                    }
                }
            }
        }

        // check for new minima
        size = distro[i].size();

        // detect and process new minima at level i
        for (int j = 0; j < size; j++)
        {
            p = distro[i][j];

            // reset distance to 0
            dist.at<int>(p.x, p.y) = 0;

            // if p is inside a new minimum
            if ( workOut.at<qint8>(p.x, p.y) == MASK)
            {
                // create a new label
                currentLabel++;
                fifo.add(p);

                if (RGI)
                {
                    // Begin RGI
                    workOut.at<qint8>(p.x, p.y)= markers.at<qint8>(p.x, p.y);
                }
                else
                {
                    workOut.at<qint8>(p.x, p.y) = currentLabel;
                }
                // End RGI
                int aaa = workOut.at<qint8>(p.x, p.y);

                if (TRACE)
                {
                  //  dump8_cv("workout", workOut);
                }

                while (fifo.isEmpty() == false)
                {
                    Point q = fifo.retrieve();

                    if (TRACE)
                    {
                        cout << "Retieving p from FIFO: " << p.x << " " << p.y << endl;
                    }

                    // for every pixel in the 8-neighbourhood of q
                    for (int r = q.x - 1; r <= q.x + 1; r++)
                    {
                        for (int c = q.y - 1; c <= q.y + 1; c++)
                        {
                            if (r < 0 || r >= inputImage.rows || c < 0 || c >= inputImage.cols)
                            {
                                continue;
                            }

                            if (!(c == q.y && r == q.x) && workOut.at<qint8>(r, c) == MASK)
                            {
                                fifo.add(Point(r, c));

                                if (RGI)
                                {
                                    // Begin RGI Modifications
                                    workOut.at<qint8>(r, c) = markers.at<qint8>(r, c);
                                    // End RGI
                                }
                                else
                                {
                                    // Original code
                                    workOut.at<qint8>(r, c) = currentLabel;
                                }
                            }
                        }
                    }
                }
            }
        }
        //        if (TRACE && size > 0)
        //        {
        //            cout << "mini ITERA I " << i << " << currentLabel" << currentLabel;
        //            cout << "fifo " << fifo.size();

        //            for (int ii=0; ii < fifo.size(); ii++)
        //            {
        //                cout << fifo.v[ii] << " ";
        //            }

        //            if (TRACE)
        //            {

        //            }
        //        }
    }
    //dump_matrix_8_cv("workout_end", 0, workOut);
}

void Watershed::calculateDistro(Mat& img, vector<Point> distro[])
{
    for (int r = 0; r < img.rows; r++)
    {
        for (int c = 0; c < img.cols; c++)
        {
            if (TRACE)
            {
                h[(uint8_t)img.at<unsigned char>(r, c)]++;
            }
            distro[(uint8_t)img.at<unsigned char>(r, c)].push_back(Point(r, c));
        }
    }

    if (TRACE)
    {
        for (int i=0; i<256; i++)
        {
            for (int j=0; j<distro[i].size(); j++)
            {
                cout << i << " (" << distro[i][j].x << ", " <<  distro[i][j].y << ")" << endl;
            }
            cout << i << " " << h[i] << endl;
        }
    }
}

bool Watershed::areThereLabelledNeighbours(Mat img, Point p0)
{
    for (int r = p0.x - 1; r <= p0.x + 1; r++)
    {
        if (r >= img.rows || r < 0)
        {
            continue;
        }

        for (int c = p0.y - 1; c <= p0.y + 1; c++)
        {
            if (c >= img.cols || c < 0)
            {
                continue;
            }

            if (!(r == p0.x && c == p0.y) && img.at<qint8>(r, c) >= WSHED)
            {
                return true;
            }
        }
    }
    return false;
}

