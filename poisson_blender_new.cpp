/*
#########################   Interactive Poisson Blending ############################

Copyright (C) 2012 Siddharth Kherada
Copyright (C) 2006-2012 Natural User Interface Group

Source: https://github.com/Siddharthk/CompPhoto-NUIGroup-GSoC-2012/blob/master/Interactive%20Poisson%20Blending/poisson.h

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details. "

#####################################################################################
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "math.h"
#include "poisson_new.h"
#include "poisson_blender_new.h"

using namespace std;
using namespace cv;

IplImage *img_0, *img_1, *img_2, *subimg, *result;
CvPoint point;
int drag = 0;
int destx, desty;

void drawImage(IplImage* target, IplImage* source, int x, int y)
{
    for (int ix=0; ix<source->width; ix++)
    {
        for (int iy=0; iy<source->height; iy++)
        {
            int r = cvGet2D(source, iy, ix).val[2];
            int g = cvGet2D(source, iy, ix).val[1];
            int b = cvGet2D(source, iy, ix).val[0];
            CvScalar bgr = cvScalar(b, g, r);
            cvSet2D(target, iy+y, ix+x, bgr);
        }
    }
}

//--------------------------------------------------------------------

void mouseHandler(int event, int x, int y, int flags, void* param)
{


    if (event == CV_EVENT_LBUTTONDOWN && !drag)
    {
        point = cvPoint(x, y);
        drag  = 1;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag)
    {
        img_1 = cvCloneImage(img_0);

        cvRectangle(img_1,point,cvPoint(x, y),CV_RGB(255, 0, 0),1, 8, 0);

        cvShowImage("Source", img_1);
    }

    if (event == CV_EVENT_LBUTTONUP && drag)
    {
        img_1 = cvCloneImage(img_0);

        cvSetImageROI(img_1,cvRect(point.x,point.y,x - point.x,y - point.y));

        subimg = cvCreateImage(cvGetSize(img_1), img_1->depth, img_1->nChannels);

        cvCopy(img_1, subimg, NULL);

        cvNamedWindow("ROI",1);
        cvShowImage("ROI", subimg);
        cvWaitKey(0);
        cvDestroyWindow("ROI");
        cvResetImageROI(img_1);
        cvShowImage("Source", img_1);
        drag = 0;
    }

    if (event == CV_EVENT_RBUTTONUP)
    {
        cvShowImage("Source", img_0);
        drag = 0;
    }
}

//--------------------------------------------------------------------

void mouseHandler1(int event, int x, int y, int flags, void* param)
{
    IplImage *im, *iimage_1;

    iimage_1 = cvCloneImage(img_2);

    if (event == CV_EVENT_LBUTTONDOWN)
    {
        point = cvPoint(x, y);

        cvRectangle(iimage_1,cvPoint(x, y),cvPoint(x+subimg->width,y+subimg->height),CV_RGB(255, 0, 0),1, 8, 0);

        destx = x;
        desty = y;

        cvShowImage("Destination", iimage_1);
    }
    if (event == CV_EVENT_RBUTTONUP)
    {
        if(destx+subimg->width > img_2->width || desty+subimg->height > img_2->height)
        {
            cout << "Index out of range" << endl;
            exit(0);
        }

        drawImage(iimage_1,subimg,destx,desty);
        int cnt = 0;
        result = poisson_blend(img_2,subimg,desty,destx);

        ////////// save blended result ////////////////////

        cvSaveImage("Output.jpg",result);
        cvSaveImage("cutpaste.jpg",iimage_1);

        cvNamedWindow("Image cloned",1);
        cvShowImage("Image cloned", result);
        cvWaitKey(0);
        cvDestroyWindow("Image cloned");
    }
}

//--------------------------------------------------------------------

int unit_test()
{
    const string path("D:/PRJ/Benchmark/OBJECTIVE_TEST/Test/");

    vector<Point>err(2);
    err[0].x = 0;
    err[0].y = 0;

    vector<Point>shift(2);
    shift[0].x = 0 - err[0].x;
    shift[0].y = 0 - err[0].y;

    shift[1].x = 0 - err[1].x;
    shift[1].y = 64 - err[1].y;

    // long start, end;
    //start = clock();

    // -------------------------------------------------
    // interactive_test
    // -------------------------------------------------
    // interactive_test : 0 - Batch Image Stitching
    // interactive_test : 1 - Batch Object embed
    // interactive_test : 2 - Interactive Object embed
    //--------------------------------------------------
    const int interactive_test = 0;

    if (interactive_test == 0)
    {
        //-----------------------------------------
        // Begin Batch Stitching Unit Test - RGI
        //----------------------------------------
        Mat image_1 = imread(path + string("D:/PRJ/Benchmark/OBJECTIVE_TEST/Image_0000/roi_1/roi_1_66_147_0_0.png")); //test_poisson_img2.png"));
        img_0 = new IplImage(image_1);

        Mat image_2 = imread(path + string("D:/PRJ/Benchmark/OBJECTIVE_TEST/Image_0000/roi_2/Luminance/10/roi_2_130_147.png")); //test_poisson_img1.png"));
        img_2 = new IplImage(image_2);

        const Point pos_min(min(shift[0].x, shift[1].x),
                min(shift[0].y, shift[1].y));

        const Point pos_max(max(shift[0].x + image_1.cols, shift[1].x + image_2.cols),
                max(shift[0].y + image_1.rows, shift[1].y + image_1.rows));

        const Size canv_size(pos_max.x - pos_min.x, pos_max.y - pos_min.y);

        // Creating target canvas
        // Size canvSize = Size(image_1.cols-4 + m2.cols, image_1.rows + m2.rows);

        Mat trg_img = Mat::ones(canv_size, CV_8UC3);
        trg_img.setTo(255);

        Mat mask_0(canv_size, CV_8UC3);
        mask_0 = trg_img(Rect(shift[0].x, shift[0].y, image_1.cols, image_1.rows));
        image_1.copyTo(mask_0);

        Mat mask_2(canv_size, CV_8UC3);
        mask_2 = trg_img(Rect(shift[1].x, shift[1].y, image_2.cols, image_2.rows));
        image_2.copyTo(mask_2);

        //                cvNamedWindow("Test", 1);
        //               IplImage aa(trg_img);
        //               cvShowImage("Test",&aa);
        //                cvWaitKey(0);
        //        return 0;

        img_2 = new IplImage(trg_img);

        CvPoint pt_1;
        pt_1.x = 0;
        pt_1.y = 0;

        CvPoint pt_2;
        pt_2.x = img_0->width - 1;
        pt_2.y = img_0->height - 1;

        // Target position of the selected subimg      
        img_1 = cvCloneImage(img_0);
        subimg = cvCreateImage(cvGetSize(img_1), img_1->depth, img_1->nChannels);
        cvCopy(img_1, subimg, NULL);
        cvSetImageROI(img_1,cvRect(pt_2.x, pt_2.y, pt_2.x - pt_1.x, pt_2.y - pt_1.y));

        if(shift[0].x + subimg->width > img_2->width || shift[1].y + subimg->height > img_2->height)
        {
            cout << "Index out of range" << endl;
            exit(0);
        }

       int cnt = 0;
       result = poisson_blend(img_2, subimg, shift[0].y, shift[0].x);
        imwrite(path + "poisson_batch_stitch.png", Mat(result));

        cvNamedWindow("Result", 1);
        cvShowImage("Result", result);
        cvWaitKey(0);
        //-----------------------------------------
        // End Batch Stitching Unit Test - RGI
        //----------------------------------------
    }
    else if (interactive_test == 1)
    {
        //-----------------------------------------
        // Begin Object Embed Batch Unit Test - RGI
        //----------------------------------------

        Mat image_1 = imread(path + string("eye1.jpg"));
        img_0 = new IplImage(image_1);

        Mat m2 = imread(path + string("hand1.jpg"));
        img_2 = new IplImage(m2);

        // Creating target canvas
       // Size canvSize = Size(image_1.rows + m2.rows, image_1.cols + m2.cols);

        CvPoint pt_1;
        pt_1.x = 0;
        pt_1.y = 0;

        CvPoint pt_2;
        pt_2.x = img_0->width - 1;
        pt_2.y = img_0->height - 1;

        // Target position of the selected subimg
        desty = 185; //img_2->height/2;
        destx = 160; //img_2->width/2;

        IplImage *im, *iimage_1;
        iimage_1 = cvCloneImage(img_2);

        img_1 = cvCloneImage(img_0);
        subimg = cvCreateImage(cvGetSize(img_1), img_1->depth, img_1->nChannels);
        cvCopy(img_1, subimg, NULL);
        cvRectangle(iimage_1,cvPoint(destx, desty),cvPoint(destx+subimg->width,desty+subimg->height),CV_RGB(255, 0, 0),1, 8, 0);
        cvSetImageROI(img_1,cvRect(pt_2.x,pt_2.y,pt_2.x - pt_1.x,pt_2.y - pt_1.y));

        if(destx+subimg->width > img_2->width || desty+subimg->height > img_2->height)
        {
            cout << "Index out of range" << endl;
            exit(0);
        }

        int cnt = 0;
        result = poisson_blend(img_2,subimg,desty,destx);

        imwrite(path + "poisson_batch_embed.jpg", Mat(result));

        cvNamedWindow("Result", 1);
        cvShowImage("Result",result);
        cvWaitKey(0);
        return 0;
        //-----------------------------------------
        // End Object Embed Batch Unit Test - RGI
        //----------------------------------------
    }
    else  // if (interactive_test == 0)
    {
        //-----------------------------------------
        // Begin Interactive Unit Test - RGI
        //----------------------------------------
        Mat image_1 = imread(path + string("eye1.jpg"));
        img_0 = new IplImage(image_1);

        Mat m2 = imread(path + string("hand1.jpg"));
        img_2 = new IplImage(m2);

        //////////// source image ///////////////////

        cvNamedWindow("Source", 1);
        cvSetMouseCallback("Source", mouseHandler, NULL);
        cvShowImage("Source", img_0);

        /////////// destination image ///////////////
        imwrite(path + "poisson_interactive_embed.jpg", Mat(img_2));

        cvNamedWindow("Destination", 1);
        cvSetMouseCallback("Destination", mouseHandler1, NULL);
        cvShowImage("Destination",img_2);

        cvWaitKey(0);
        cvDestroyWindow("Source");
        cvDestroyWindow("Destination");

        //-----------------------------------------
        // Begin Interactive Unit Test - RGI
        //----------------------------------------
    }
    // cvReleaseImage(&img_0);
    // cvReleaseImage(&img_1);
    // cvReleaseImage(&img_2);

    return 0;
}
