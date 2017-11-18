#include "watershed_soille.h"
#include "remove_watersheds.h"
#include "tools.h"

watershedSoille::watershedSoille()
{
    cnt=0;
}

watershedSoille::~watershedSoille()
{
    
}

/**
     * <p>
     * Segments the image using initial seeds for each region.  This is often done to avoid
     * over segmentation but requires additional preprocessing and/or knowledge on the image structure.  Initial
     * seeds are specified in the input image 'seeds'.  A seed is any pixel with a value > 0.  New new regions
     * will be created beyond those seeds.  The final segmented image is provided by {@link #getOutput()}.
     * </p>
     *
     * <p>
     * NOTE: If seeds are used then {@link #getTotalRegions()} will not return a correct solution.
     * </p>
     *
     * @param input (Input) Input image
     * @param seeds (Output) Segmented image containing seeds.  Note that all seeds should have a value > 0 and have a
     *              value <= numRegions.
     */

bool watershedSoille::process( Mat &input , Mat &seeds, Mat g, int max_g) {
    
    if (input.rows != seeds.rows || input.cols != seeds.cols)
    {
        return false;
    }
    
     // dump_matrix_8_cv("input_1", 0, input);
    //  dump_matrix_32_cv("markers_1", 0, seeds);
    
    removedWatersheds = false;
    output = Mat(input.rows,input.cols, CV_8SC1);
    distance = Mat(input.rows,input.cols, CV_32SC1);
    
    output.setTo(INIT);
    distance.setTo(0);
    
    fifo.reset();
    
    // copy the seeds into the output directory
    for( int r = 0; r < seeds.rows; r++ ) {
        for( int c = 0; c < seeds.cols; c++) {
            int v = seeds.at<int32_t>(r, c);
            if( v > 0 ) {
                output.at<int8_t>(r, c) = v;
            }
        }
    }
    
    for (uint k = 0; k < max_g - 1; k++)
    {
        out_buffer[k] = Mat(input.rows,input.cols, CV_8SC1);
        out_buffer[k].setTo(INIT);

        // copy the seeds into the output directory
        for( int r = 0; r < seeds.rows; r++ ) {
            for( int c = 0; c < seeds.cols; c++) {
                int v = seeds.at<int32_t>(r, c);
                if( v > 0 ) {
                    out_buffer[k].at<int8_t>(r, c) = v;
                }
            }
        }

        sortPixels(input, g, k + 2);
        // dump_matrix_8_cv("output_after_sort", 0, out_buffer[k]);
        
        // perform watershed
        for( int i = 0; i < histogram[0].size(); i++ ) {
         //   qDebug() <<"i: " << i << "  " << histogram[0].get(i).x << " " << histogram[0].get(i).y;
        }
        
        for( int i = 0; i < 256; i++ ) {
            Queue level = histogram[i];
            if( level.size() == 0 )
            {
                continue;
            }
            
            
            // Go through each pixel at this level and mark them according to their neighbors
            for(int j = 0; j < level.size(); j++)
            {
                //   qDebug() <<"Level i: " << 0 << "  " << level.get(0).x << " " << level.get(0).y;
                Point index = level.get(j);
                
                // If not has not already been labeled by a seed then try assigning it values
                // from its neighbors
                if ( out_buffer[k].at<int8_t>(index.x, index.y) == INIT)
                {
                    out_buffer[k].at<int8_t>(index.x , index.y) = MASK;
                    assignNewToNeighbors8(index, g, k);
                }
            }
            
            currentDistance = 1;
            fifo.add(MARKER_PIXEL);
            
            while( true ) {
                Point p = fifo.retrieve();
                
                // end of a cycle.  Exit the loop if it is done or increase the distance and continue processing
                if( p == MARKER_PIXEL) {
                    if( fifo.isEmpty() )
                        break;
                    else {
                        fifo.add(MARKER_PIXEL);
                        currentDistance++;
                        p = fifo.retrieve();
                    }
                }

                // look at its neighbors and see if they have been labeled or belong to a watershed
                // and update its distance
                checkNeighborsAssign8(p, g, k);
                //   dump_matrix_8_cv("marker", i, output );
            }
            
            // Ensure that all pixels have a distance of zero
            // Could probably do this a bit more intelligently...

            distance.setTo(0);
            
            
            string fileNameOut = ("D:/PRJ/LITTORALG/Docs/Morphological image compositing/Test/output_loop.png");
            imwrite(fileNameOut, out_buffer[k]);
        }
        // dump_matrix_8_cv("out_buffer", k, out_buffer[k]);
    }


    uint8_t score_table[8];
    int32_t label  = 0;

    // Applies Majority rule to resolve multilabeled pixels
    uint32_t max_score = 0;
    uint8_t max_label = 0;

    for( int r = 1; r < seeds.rows - 1; r++ )
    {
        for( int c = 1; c < seeds.cols - 1; c++ )
        {

            if (g.at<uint8_t>(r, c) > 1)
            {
                max_score = 0;
                max_label = 0;
                memset(score_table, 0, sizeof(uint8_t) * 4);

                for (uint k = 0; k < max_g - 1; k++)
                {
                    label = out_buffer[k].at<int8_t>(r, c);

                    if (label >= 0)
                    {
                        score_table[label]++;

                        if (score_table[label] >= max_score)
                        {
                            max_score = score_table[label];
                            max_label = label;
                        }

                        seeds.at<int32_t>(r, c) = max_label;
                    }
                }
            }
        }
    }
    seeds.convertTo(seeds, CV_8U);
    // dump_matrix_8_cv("seeds", 0, seeds );
    //   output.copyTo(seeds);
    return true;
}


void watershedSoille::handleNeighborAssign(Point &indexTarget, Point &indexNeighbor, Mat &g, const uint32_t k)
{
    int regionNeighbor = out_buffer[k].at<int8_t>(indexNeighbor.x, indexNeighbor.y);
    int distanceNeighbor = distance.at<int32_t>(indexNeighbor.x, indexNeighbor.y);

    // if neighbor has been assigned a region or is WSHED
    if( regionNeighbor >= 0 && distanceNeighbor < currentDistance )
    {
        int regionTarget = out_buffer[k].at<int8_t>(indexTarget.x, indexTarget.y);

        // see if the target belongs to an already labeled basin or watershed
        if( regionNeighbor > 0 )
        {
            if( regionTarget < 0 )
            {
                // if is MASK
                out_buffer[k].at<int8_t>(indexTarget.x, indexTarget.y) = regionNeighbor;
                //   qDebug() << cnt++ << "1. (" << indexTarget.x << ", " << indexTarget.y << "): " <<  output.at<int8_t>(indexTarget.x, indexTarget.y);

            }
            else if( regionTarget == 0 )
            {
                // if it is a watershed only assign to the neighbor value if it would be closer
                // this is a deviation from what's in the paper.  There might be a type-o there or I miss read it
                if( distanceNeighbor+1 < currentDistance  )
                {
                    out_buffer[k].at<int8_t>(indexTarget.x, indexTarget.y) = regionNeighbor;
                    // qDebug() << cnt++ << "2. (" << indexTarget.x << ", " << indexTarget.y << "): " <<  output.at<int8_t>(indexTarget.x, indexTarget.y);
                }
            }
            else if( regionTarget != regionNeighbor )
            {
                out_buffer[k].at<int8_t>(indexTarget.x, indexTarget.y) = WSHED;
                // qDebug() << cnt++  << "3. (" << indexTarget.x << ", " << indexTarget.y << "): " <<  output.at<int8_t>(indexTarget.x, indexTarget.y);
            }
        }
        else if( regionTarget == MASK )
        {
            out_buffer[k].at<int8_t>(indexTarget.x, indexTarget.y) = WSHED;
            //  qDebug() << cnt++ << "4. (" << indexTarget.x << ", " << indexTarget.y << "): " <<  output.at<int8_t>(indexTarget.x, indexTarget.y);
        }
    }
    else if( regionNeighbor == MASK && distanceNeighbor == 0)
    {
        distance.at<int32_t>(indexNeighbor.x, indexNeighbor.y) = currentDistance + 1;
        fifo.add(indexNeighbor);
    }
}

void watershedSoille::checkMask(Point index)
{
    if( output.at<int8_t>(index.y, index.x) == MASK )
    {
        output.at<int8_t>(index.y, index.x) = currentLabel;
        fifo.add(index);
    }
}

/**
     * Very fast histogram based sorting.  Index of each pixel is placed inside a list for its intensity level.
     */
void watershedSoille::sortPixels(Mat input, Mat g, const uint32_t k)
{
    // initialize histogram
    for( int i = 0; i < 256; i++ )
    {
        histogram[i].reset();
    }
    // sort by creating a histogram

    for( int r = 1; r < input.rows - 1; r++ )
    {
        for( int c = 1; c < input.cols - 1; c++ )
        {
            if (g.at<uint8_t>(r, c) == k)
            {
                int value = input.at<uint8_t>(r, c);
                Point indexOut(r, c);
                histogram[value].add(indexOut);
            }
        }
    }
}

/**
     * Segmented output image with watersheds.  This is a sub-image of {@link #getOutputBorder()} to remove
     * the outside border of -1 valued pixels.
     */
Mat watershedSoille::getOutput()
{
    Mat outputSub(1,1,output.cols-1,output.rows-1);
    output(cv::Rect(1,1,output.cols-1,output.rows-1)).copyTo(outputSub);
    return outputSub;
}

/**
     * The entire segmented image used internally.  This contains a 1-pixel border around the entire
     * image filled with pixels of value -1.
     */
Mat watershedSoille::getOutputBorder()
{
    return output;
}

/**
     * Removes watershed pixels from the output image by merging them into an arbitrary neighbor.
     */
void watershedSoille::removeTheWatersheds()
{
    removedWatersheds = true;
    removeWatersheds.remove(output);
}


/**
     * Returns the total number of regions labeled.  If watersheds have not
     * been removed then this will including the watershed.
     *
     * <p>THIS IS NOT VALID IF SEEDS ARE USED!!!</p>
     *
     * @return number of regions.
     */
int watershedSoille::getTotalRegions()
{
    return removedWatersheds ? currentLabel : currentLabel + 1;
}
