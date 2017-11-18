#include "remove_watersheds.h"

RemoveWatersheds::RemoveWatersheds()
{

}

RemoveWatersheds::~RemoveWatersheds()
{

}

bool RemoveWatersheds::remove(Mat &segmented)
{
    open.empty();

    // step through the inner pixels and find watershed pixels
    for (int r = 1; r < segmented.rows-1; r++)
    {
        for (int c = 1; c < segmented.cols-1; c++)
        {
            if (segmented.at<uint8_t>(r, c) == 0)
            {
                open.push(Point(r, c));
            }
        }
    }

    // assign region values to watersheds until they are all assigned
    while (open.size() != 0)
    {
        Point index = open.front();
        open.pop();
        // assign it to the first valid region it finds
        for (int d_x = -1; d_x < 1; d_x++)
        {
            for (int d_y = -1; d_y < 1; d_y++)
            {
                // the outside border in the enlarged segmented image will have -1 and watersheds are 0
                Point adj_index = index + Point(d_x, d_y);
                uint8_t adj_px = segmented.at<uint8_t>(adj_index.x, adj_index.y);

                if (adj_px != 0)
                {
                    segmented.at<uint8_t>(index.x, index.y) = adj_px;
                    break;
                }
            }
        }
    }

    // Statically assigns the corners
    segmented.at<uint8_t>(0, 0) = 2;
    segmented.at<uint8_t>(segmented.rows - 1, segmented.cols - 1) = 1;
    // dump_matrix_8_cv("segmented.txt", 0, segmented);
    return true;
}

