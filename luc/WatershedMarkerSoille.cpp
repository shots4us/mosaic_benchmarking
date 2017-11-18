#include "WatershedMarkerSoille.hpp"
//using namespace watershedSpace;


WatershedMarkerSoille::WatershedMarkerSoille(){

};




// void dumpMat(Mat input);



// #define DEBUG


void WatershedMarkerSoille::process(Mat & input, Mat &  seeds) {

#ifdef DEBUG
	printf("WatershedMarkerSoille::process\n");
#endif

	removedWatersheds = false;
	Size ns(input.cols+2,input.rows+2);

	output = new Mat(ns,CV_32S,cv::Scalar(INIT));
	distanceM =new Mat(ns,CV_32S,cv::Scalar(0));
	fifo =new queue<int>();

	// copy the seeds into the output directory

	for( int y = 0; y < seeds.rows; y++ ) {
		for( int x = 0; x < seeds.cols; x++   ) {
			int v = seeds.at<int>(y,x);
            if (v > 0)
				(*output).at<int>(y+1,x+1)= v ;
		}
	}

	// sort pixels


#ifdef DEBUG
	printf("sortPixels\n");
#endif

	sortPixels(input);



	// perform watershed
	for( unsigned i = 0; i < histogramSize; i++ ) {

#ifdef DEBUG
	printf("watershed color %u \n",i);
#endif
		vector<int> level = histogram[i];


		if( level.empty()  )
			continue;


		// Go through each pixel at this level and mark them according to their neighbors
		for( unsigned j = 0; j < level.size(); j++ ) {

			int index = level[j];
			int x=index % output->cols;
			int y=index / output->cols;
			// If not has not already been labeled by a seed then try assigning it values
			// from its neighbors


			if( (*output).at<int>(y,x)  == INIT ) {
				(*output).at<int>(y,x) = MASK;
				assignNewToNeighbors(index,x,y);
			}
		}

		currentDistance = 1;


		fifo->push(MARKER_PIXEL);



		while( true ) {
			int p = fifo->front();
			fifo->pop();
			// end of a cycle.  Exit the loop if it is done or increase the distance and continue processing
			if( p == MARKER_PIXEL) {
				if( fifo->empty() )
					break;
				else {

					fifo->push(MARKER_PIXEL);
					currentDistance++;
					p = fifo->front();
					fifo->pop();
				}
			}

			// look at its neighbors and see if they have been labeled or belong to a watershed
			// and update its distance
			int x=p % output->cols;
			int y=p / output->cols;
			checkNeighborsAssign(x,y);



		}



		// Ensure that all pixels have a distance of zero
		// Could probably do this a bit more intelligently...
		///ImageMiscOps.fill(distance, 0);
		for( int y = 0; y < (*distanceM).rows; y++ ) {

			for( int x = 0; x < (*distanceM).cols; x++   ) {

				(*distanceM).at<int>(y,x)= 0; ;

			}
		}
	}



	//copy output  seeds
	for( int y = 0; y < seeds.rows; y++ ) {
		for( int x = 0; x < seeds.cols; x++   ) {
			seeds.at<int>(y,x)= (*output).at<int>(y+1,x+1) ;

		}
	}

  delete(fifo);
  output->release( );
  distanceM->release();

};


/**
 * Very fast histogram based sorting.  Index of each pixel is placed inside a list for its intensity level.
 */


void WatershedMarkerSoille::sortPixels(Mat input) {

    // cout << "type " << input.type() << endl;

	// initialize histogram
	for( int i = 0; i < histogramSize; i++ ) {
		histogram[i].clear();
	}
	// sort by creating a histogram

	for (int y = 0; y < input.rows; y++  ){ //,index++ , indexOut++) {
		for( int x = 0; x < input.cols; x++ ) {
			int indexOut = (x +1 )+ (y+1)*output->cols;

			/**
			 * AVEC data

			unsigned char value=((unsigned char*)input.data)[y+x*input.step];

			 */

			unsigned char value = input.at<unsigned char>(y,x);// & 0xFF;

			histogram[value].push_back(indexOut);
		}


	}

}



/**
	 * Implementation which uses a 4-connect rule
	 */
void WatershedMarkerSoille::assignNewToNeighbors(int index,unsigned x, unsigned y) {



	if( (*output).at<int>(y,x+1) >= 0 ) {                           // (x+1,y)
		(*distanceM).at<int>(y,x) = 1;

		fifo->push(index);
	} else if( (*output).at<int>(y,x-1) >= 0 ) {                    // (x-1,y)
		(*distanceM).at<int>(y,x) = 1;

		fifo->push(index);
	} else if( (*output).at<int>(y+1,x) >= 0 ) {        // (x,y+1)
		(*distanceM).at<int>(y,x) = 1;

		fifo->push(index);
	} else if( (*output).at<int>(y-1,x) >= 0 ) {        // (x,y-1)
		(*distanceM).at<int>(y,x) = 1;

		fifo->push(index);
	}
}



//
//
// diffuse Neighbor
//
 void WatershedMarkerSoille::handleNeighborAssign(int xTarget,int yTarget ,int  xNeighbor,int yNeighbor) {
	int regionNeighbor = (*output).at<int>(yNeighbor,xNeighbor);
	int distanceNeighbor = (*distanceM).at<int>(yNeighbor,xNeighbor);

	// if neighbor has been assigned a region or is WSHED
	if( regionNeighbor >= 0 && distanceNeighbor < currentDistance ) {
		int regionTarget = (*output).at<int>(yTarget,xTarget);

		// see if the target belongs to an already labeled basin or watershed
		if( regionNeighbor > 0 ) {
			if( regionTarget < 0 ) {// if is MASK
				(*output).at<int>(yTarget,xTarget) = regionNeighbor;
			} else if( regionTarget == 0 ) {
				// if it is a watershed only assign to the neighbor value if it would be closer
				// this is a deviation from what's in the paper.  There might be a type-o there or I miss read it
				if( distanceNeighbor+1 < currentDistance  ) {
					(*output).at<int>(yTarget,xTarget) = regionNeighbor;
				}
			} else if( regionTarget != regionNeighbor ) {
				(*output).at<int>(yTarget,xTarget) = WSHED;
			}
		} else if( regionTarget == MASK ) {
			(*output).at<int>(yTarget,xTarget) = WSHED;
		}
	} else if( regionNeighbor == MASK && distanceNeighbor == 0) {
		(*distanceM).at<int>(yNeighbor,xNeighbor) = currentDistance + 1;

			int indexN=xNeighbor+output->cols*yNeighbor;

		fifo->push(indexN);
	}
}

 //
 // check Neighbors
 //
 void WatershedMarkerSoille::checkNeighborsAssign(int x,int y) {
		handleNeighborAssign(x,y, x+1,y );
		handleNeighborAssign(x,y, x-1,y );
		handleNeighborAssign(x,y, x,y+1 );
		handleNeighborAssign(x,y, x,y-1 );
	}


//		}


// void dumpMat(Mat input){
//	 printf("rows %d cols %d  step %d \n",input.rows,input.cols,input.step);

//	// cout << input << endl;
//	 for( int y = 0; y < input.rows; y++ ) {

//				 for( int x = 0; x < input.cols; x++   )
//					 printf("%2d ", input.at<int>(y,x));
//				 printf("\n");


//			 }


// }




 /*
  * Test Watershed
  *
  */
/*


 int main(int argc ,char * argv[]){



	 cv::Mat image = cv::imread(argv[1]);
	 //cout << image << endl;
	 cout << "IMAGE TYPE" << image.type() << endl;
	 cv::Mat blank(image.size(),CV_8U,cv::Scalar(0xFF));
	 cv::Mat dest;
	 imshow("originalimage", image);

	 // Create markers image
	 cv::Mat markers(image.size(),CV_8U,cv::Scalar(-1));

	 markers(Rect(10,0,image.cols -10, 1)) = Scalar::all(1);
	 //bottom rectangle
	 markers(Rect(0,image.rows-1,image.cols-10, 1)) = Scalar::all(2);
	 //left rectangle
	 markers(Rect(0,10,1,image.rows-10)) = Scalar::all(2);
	 //right rectangle
	 markers(Rect(image.cols-1,0,1,image.rows -10)) = Scalar::all(1);

	 markers.convertTo(markers,CV_BGR2GRAY);
	 imshow("markers", markers);

	 WatershedMarkerSoille   segmenter;

	 markers.convertTo(markers, CV_32S);
	 Mat gImg;
	 cvtColor(image, gImg, CV_RGB2GRAY);

	 segmenter.process(gImg,markers);

	 markers.convertTo(markers,CV_8U);
	 cv::Mat mask;
	 convertScaleAbs(markers, mask, 1, 0);
	 double thresh = threshold(mask, mask, 1, 255, THRESH_BINARY);


	 bitwise_and(image, image, dest, mask);
	 dest.convertTo(dest,CV_8U);

	 imshow("final_result", dest);


	 cv::waitKey(0);

	 return 0;



 }
*/

