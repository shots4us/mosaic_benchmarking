#ifndef REMOVE_WATERSHED_H
#define REMOVE_WATERSHED_H

#include "global.h"
#include "tools.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <math.h>
#include <queue>
#include <stdint.h>

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

/**
 * Examines a segmented image created by {@link WatershedVincentSoille1991} and merged watershed pixels
 * into neighboring regions.  Since there is no good rule for which region the pixel should be
 * merged into, it is merged into the first valid one.
 *
 * @author Peter Abeles
 */

class RemoveWatersheds
{
public:
    RemoveWatersheds();
    ~RemoveWatersheds();

        // relative indexes of connected pixels
       // int connect[4];

        // list of watershed pixels which have yet to be merged.
        queue<Point> open;
       // Queue open2;

        /**
         * Removes watersheds from the segmented image.  The input image must be the entire original
         * segmented image and assumes the outside border is filled with values < 0.  To access
         * this image call {@link boofcv.alg.segmentation.watershed.WatershedVincentSoille1991#getOutputBorder()}.
         * Each watershed is assigned the value of an arbitrary neighbor.  4-connect rule is used for neighbors.
         * Doesn't matter if initial segmented was done using another connectivity rule.  The value of each region i
         * s reduced by one at the very end.
         *
         * @param segmented Entire segmented image (including border of -1 values) with watersheds
         */
        bool remove( Mat &segmented );
};

#endif // REMOVE_WATERSHED_H
