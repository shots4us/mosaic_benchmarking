#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <math.h>
#include <stdint.h>

#include "tools.h"
#include "remove_watersheds.h"

using namespace cv;
using namespace std;

/*
 * Copyright (c) 2011-2014, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef watershedSoille_H
#define watershedSoille_H


class watershedSoille
{
public:
    watershedSoille();
    ~watershedSoille();

    bool process(Mat &input , Mat &seeds , Mat g, int max_g);

    int cnt;

    // values of pixels belonging to the watershed
    const int WSHED = 0;

    // initial value of the labeled output image
    const int INIT = -1;

    // Initial value of a threshold level
    const int MASK = -2;

    // index of the marker pixel.  Fictitious
    const Point MARKER_PIXEL = Point(-1, -1);


    // histogram for sorting the image.  8-bits so 256 possible values
    // each element refers to a pixel in the input image

protected:
    // integer queue, items are added and extracted from the tail. Java : GrowQueue_I32
    Queue histogram[256];

    // Output image.  This is im_o in the paper.
    // The output image has a 1-pixel wide border which means that bound checks don't need
    // to happen when examining a pixel's neighbor.
    Mat output;
    Mat out_buffer[8];

    // storage for sub-image output
    Mat outputSubt32;

    // work image of distances. im_d in the paper
    // also has a 1 pixel border
    Mat distance;
    int currentDistance;

    // label of the region being marked
    int currentLabel;

    // FIFO circular queue
    Fifo fifo;

    // used to remove watersheds
    RemoveWatersheds removeWatersheds;
    bool removedWatersheds;

protected:
    /**
         * See if a neighbor has a label ( > 0 ) or has been assigned WSHED ( == 0 ).  If so
         * set distance of pixel index to 1 and add it to fifo.
         *
         * @param index Pixel whose neighbors are being examined
         */
    // virtual void assignNewToNeighbors(int index) = 0;

    /**
         * Check the neighbors to see if it should become a member or a watershed
         * @param index Index of the target pixel
         */
    // virtual void checkNeighborsAssign(int index) = 0;

    void handleNeighborAssign(Point &indexTarget, Point &indexNeighbor, Mat &g, const uint32_t k);

    /**

          * Checks neighbors of pixel 'index' to see if their region is MASK, if so they are assigned the
          * currentLabel and added to fifo.
          *
          * @param index Pixel whose neighbors are being examined.
          */
    // virtual void checkNeighborsMasks(int index);

    /**
          * Very fast histogram based sorting.  Index of each pixel is placed inside a list for its intensity level.
          */

    void checkMask(Point index);
    void sortPixels(Mat input, Mat g, const uint32_t k);

    Mat getOutput();
    Mat getOutputBorder();
    void removeTheWatersheds();
    int getTotalRegions();


    //    /**
    //     * Implementation which uses a 4-connect rule
    //     */
    //    void assignNewToNeighbors4s(int index) {
    //        if( output.data[index+1] >= 0 ) {                           // (x+1,y)
    //            distance.data[index] = 1;
    //            fifo.add(index);
    //        } else if( output.data[index-1] >= 0 ) {                    // (x-1,y)
    //            distance.data[index] = 1;
    //            fifo.add(index);
    //        } else if( output.data[index+output.step] >= 0 ) {        // (x,y+1)
    //            distance.data[index] = 1;
    //            fifo.add(index);
    //        } else if( output.data[index-output.step] >= 0 ) {        // (x,y-1)
    //            distance.data[index] = 1;
    //            fifo.add(index);
    //        }
    //    }

    //    void checkNeighborsAssign4(int index) {
    //        handleNeighborAssign(index, index + 1);
    //        handleNeighborAssign(index, index - 1);
    //        handleNeighborAssign(index, index + output.step);
    //        handleNeighborAssign(index, index - output.step);
    //    }

    //    void checkNeighborsMasks4(int index) {
    //        checkMask(index + 1);
    //        checkMask(index - 1);
    //        checkMask(index + output.step);
    //        checkMask(index - output.step);
    //    }

    /**
     * Implementation which uses a 8-connect rule
     */

    void assignNewToNeighbors8(Point index, Mat g, const uint8_t k) {
        // qDebug() << "Index x: "  << index.x << " Index y: " << index.y;
        if ( out_buffer[k].at<int8_t>(index.x+1, index.y) >= 0) { // && g.at<uint8_t>(index.x+1, index.y) == k) {                    // (x+1,y)
            distance.at<int32_t>(index.x, index.y) = 1;
            fifo.add(index);

        } else if( out_buffer[k].at<int8_t>(index.x-1, index.y) >= 0) {         // (x-1,y)
            distance.at<int32_t>(index.x, index.y) = 1;
            fifo.add(index);
        } else if( out_buffer[k].at<int8_t>(index.x, index.y+1) >= 0) {         // (x,y+1)
            distance.at<int32_t>(index.x, index.y) = 1;
            fifo.add(index);
        } else if( out_buffer[k].at<int8_t>(index.x, index.y-1) >= 0) {         // (x,y-1)
            distance.at<int32_t>(index.x, index.y) = 1;
            fifo.add(index);
        } else if( out_buffer[k].at<int8_t>(index.x+1, index.y+1) >= 0) {       // (x+1,y+1)
            distance.at<int32_t>(index.x, index.y) = 1;
            fifo.add(index);
        } else if( out_buffer[k].at<int8_t>(index.x-1, index.y+1) >= 0) {       // (x-1,y+1)
            distance.at<int32_t>(index.x, index.y) = 1;
            fifo.add(index);
        } else if( out_buffer[k].at<int8_t>(index.x+1, index.y-1) >= 0) {       // (x+1,y-1)
            distance.at<int32_t>(index.x, index.y) = 1;
            fifo.add(index);
        } else if( out_buffer[k].at<int8_t>(index.x-1, index.y-1) >= 0) {       // (x-1,y-1)
            distance.at<int32_t>(index.x, index.y) = 1;
            fifo.add(index);
        }
    }

    void checkNeighborsAssign8(Point &index, Mat &g, const uint32_t k) {
        Point pt1(index.x, index.y);
        Point pt2(index.x, index.y);
        handleNeighborAssign(pt1, pt2, g, k);

        pt1 = Point(index.x, index.y);
        pt2 = Point(index.x - 1, index.y);
        handleNeighborAssign(pt1, pt2, g, k);

        pt1 = Point(index.x, index.y);
        pt2 = Point(index.x, index.y + 1);
        handleNeighborAssign(pt1, pt2, g, k);

        pt1 = Point(index.x, index.y);
        pt2 = Point(index.x, index.y - 1);
        handleNeighborAssign(pt1, pt2, g, k);

        pt1 = Point(index.x, index.y);
        pt2 = Point(index.x + 1, index.y + 1);
        handleNeighborAssign(pt1, pt2, g, k);

        pt1 = Point(index.x, index.y);
        pt2 = Point(index.x - 1, index.y + 1);
        handleNeighborAssign(pt1, pt2, g, k);

        pt1 = Point(index.x, index.y);
        pt2 = Point(index.x + 1, index.y - 1);
        handleNeighborAssign(pt1, pt2, g, k);

        pt1 = Point(index.x, index.y);
        pt2 = Point(index.x - 1, index.y - 1);
        handleNeighborAssign(pt1, pt2, g, k);
    }
};
#endif // watershedSoille_H
