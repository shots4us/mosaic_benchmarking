
//#include <opencv2/core/core.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/xfeatures2d/nonfree.hpp>
//#include <opencv2/imgproc.hpp>

//#include <stdio.h>
//#include <iostream>
//#include <vector>
//#include "WatershedMarkerSoille.hpp"


//using namespace cv;
//using namespace std;
//using namespace xfeatures2d;

//void readme();


//#define DEBUG

///** @function main */
//int main( int argc, char** argv )
//{
//	if( argc != 3 )
//	{ readme(); return -1; }

//	Mat img2 = imread( argv[1]);//, CV_LOAD_IMAGE_GRAYSCALE );
//	Mat img1 = imread( argv[2]);//, CV_LOAD_IMAGE_GRAYSCALE );

//	if( !img1.data || !img2.data )
//	{ std::cout<< " --(!) Error reading images " << std::endl; return -1; }

//	//-- Step 1: Detect the keypoints using SURF Detector
//	int minHessian = 400;

//	Ptr<SURF>  detector =SURF::create(minHessian );


//	Mat descriptors_object, descriptors_scene;


//	std::vector<KeyPoint> keypoints_object, keypoints_scene;

//	detector->detectAndCompute(img1, Mat(), keypoints_object, descriptors_object);

//	detector->detectAndCompute(img2, Mat(), keypoints_scene,  descriptors_scene);



//	//-- Step 3: Matching descriptor vectors using FLANN matcher
//	FlannBasedMatcher matcher;
//	std::vector< DMatch > matches;
//	matcher.match( descriptors_object, descriptors_scene, matches );

//	double max_dist = 0; double min_dist = 100;

//	//-- Quick calculation of max and min distances between keypoints
//	for( int i = 0; i < descriptors_object.rows; i++ ){
//		double dist = matches[i].distance;
//		if( dist < min_dist ) min_dist = dist;
//		if( dist > max_dist ) max_dist = dist;
//	}

//	printf("-- Max dist : %f \n", max_dist );
//	printf("-- Min dist : %f \n", min_dist );

//	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
//	std::vector< DMatch > good_matches;

//	for( int i = 0; i < descriptors_object.rows; i++ )
//	{ if( matches[i].distance < 3*min_dist )
//	{ good_matches.push_back( matches[i]); }
//	}

//	Mat img_matches;
//	drawMatches( img1, keypoints_object, img2, keypoints_scene,
//			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//			std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


//	//-- Localize the object
//	std::vector<Point2f> cimg1;
//	std::vector<Point2f> cimg2;

//	for( int i = 0; i < good_matches.size(); i++ )
//	{
//		//-- Get the keypoints from the good matches
//		cimg1.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
//		cimg2.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
//	}


//#ifdef TRACE
//	imshow( "Good Matches & Object detection", img_matches );
//#endif


//	Mat H_2to1 = findHomography( cimg2, cimg1, CV_RANSAC );

//	cimg1.clear();
//	cimg2.clear();


//	Mat H_1to2 = H_2to1.inv();



//	Size simg1=img1.size();
//	Mat H_1toW = (Mat_<double>(3,3) << 1.,0,img1.cols/2,0,1,img1.rows/2,0,0,1);
//	// Mat H_1toW = (1.,0,simg1.width()/4,0,1,simg1.height()/4,0,0,1);
//	Mat H_Wto1 = H_1toW.inv();



//	//  Mat H_2toW= H_1toW   +H_2to1 ;// non focntionne pas




//	//  cout << "H_2toW" << H_2toW <<endl;

//	//-- Get the corners from the image_1 ( the object to be "detected" )
//	std::vector<Point2f> img2_corners(4);
//	img2_corners[0] = cvPoint(0,0);
//	img2_corners[1] = cvPoint( img2.cols, 0 );
//	img2_corners[2] = cvPoint( img2.cols, img2.rows );
//	img2_corners[3] = cvPoint( 0, img2.rows );

//	std::vector<Point2f> img2New_corners(4);


//	perspectiveTransform( img2_corners, img2New_corners, H_2to1);

//	perspectiveTransform( img2New_corners, img2New_corners, H_1toW);


//	vector< vector<Point> >  co_ordinates;
//	co_ordinates.push_back(vector<Point>());

//	co_ordinates[0].push_back(img2New_corners[0]);
//	co_ordinates[0].push_back(img2New_corners[1]);
//	co_ordinates[0].push_back(img2New_corners[2]);
//	co_ordinates[0].push_back(img2New_corners[3]);


//	Size ns(img2.size().width*2,img2.size().height*2);
//	Mat maskI2(ns, CV_8UC1, cv::Scalar(0));
//	drawContours( maskI2,co_ordinates,0, Scalar(255),CV_FILLED, 8 );

//#ifdef DEBUG
//	imshow( "mask 2", maskI2 );
//#endif

//	Mat maskI1(ns, CV_8UC1, cv::Scalar(0));
//		rectangle( maskI1, Point(img1.cols/2,img1.rows/2),
//				  Point(img1.cols/2+img1.cols,img1.rows/2+img1.rows),
//				  Scalar(255),CV_FILLED, 8 ,0);
//#ifdef DEBUG
//	imshow( "mask 1 ", maskI1 );
//#endif

//	Mat inter;
//	bitwise_and(maskI2,maskI1,inter);
//#ifdef DEBUG
//	imshow( "inter 1 ", inter );
//#endif

///*
//	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
//	line( img_matches, img2_corners[0] + Point2f( img1.cols, 0), img2_corners[1] + Point2f( img1.cols, 0), Scalar(0, 255, 0), 4 );
//	line( img_matches, img2_corners[1] + Point2f( img1.cols, 0), img2_corners[2] + Point2f( img1.cols, 0), Scalar( 0, 255, 0), 4 );
//	line( img_matches, img2_corners[2] + Point2f( img1.cols, 0), img2_corners[3] + Point2f( img1.cols, 0), Scalar( 0, 255, 0), 4 );
//	line( img_matches, img2_corners[3] + Point2f( img1.cols, 0), img2_corners[0] + Point2f( img1.cols, 0), Scalar( 0, 255, 0), 4 );
//*/


//	Mat part1(ns,img2.type());



//	warpPerspective(img1,part1,H_1toW,part1.size());// , INTER_LINEAR , BORDER_CONSTANT   );

//#ifdef DEBUG
//	imshow( "translation img1 ", part1 );
//#endif
//	Mat imask;
//	bitwise_not(maskI2,imask);

//	Size ns2(img2.size().width*2,img2.size().height*2);
//	Mat part2(ns2,img2.type());

//	warpPerspective(img2,part2,H_2to1,part2.size() );//, INTER_LINEAR , BORDER_CONSTANT   );
//	warpPerspective(part2,part2,H_1toW,part2.size() );//, INTER_LINEAR , BORDER_CONSTANT   );


//#ifdef DEBUG

//	    imshow( "translation trans img2 ", part2 );
//#endif


//	Mat dstI2;

//	part2.copyTo(dstI2,maskI2);

//#ifdef DEBUG
//	imshow( "partie 2", dstI2 );
//#endif


//	Mat dstI1;
//	Mat tmp;
//	bitwise_not(maskI2,tmp);
//	part1.copyTo(dstI1,tmp);

//    //cvReleaseImage(&tmp);
//#ifdef DEBUG
//	imshow( "partie 1 complete", dstI1 );
//#endif

//	Mat result =   dstI2 + dstI1;

//#ifdef DEBUG
//	imshow( "result", result );
//#endif

//	// creation de la matrice des markers
//	// -1 all
//	//  1 image 1
//	//  2 image 2
//	 cv::Mat markers(ns,CV_8U,cv::Scalar(-1));


//	 // creation du mask de image 1 non commune a img 2

//	 Mat tm1,ttm1;
//	// imshow( "maskI1 ", maskI1 );
//	 bitwise_not(maskI1,tm1);
//	// imshow( "not1 ", tm1 );
//	 bitwise_or(tm1,inter,ttm1);
//	 bitwise_not(ttm1, ttm1);
//	 //Mat m1= maskI1 - inter;

//#ifdef DEBUG
//	 imshow( "m1 ", ttm1 );
//#endif

//	 //  marker 1
//	 Mat cosntante1(ns, CV_8U, Scalar(1));
//     cosntante1.copyTo(markers, ttm1);


//     // creation du mask de image 2 non commune a img 1
//     Mat tm2,ttm2;

//    	 bitwise_not(maskI2,tm2);

//    	 bitwise_or(tm2,inter,ttm2);
//    	 bitwise_not(ttm2, ttm2);

//#ifdef DEBUG
//    	 imshow( "m2 ", ttm2 );
//#endif

//     // ecrit le marker 2
//	 Mat cosntante2(ns, CV_8U, Scalar(2));
//	 cosntante2.copyTo(markers, ttm2);



//	 tm2.release();
//	 ttm2.release();

//	 // calul des gradiants
//	 int erosion_size=1;
//	 Mat element = getStructuringElement( MORPH_RECT,
//	                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
//	                                       Point( erosion_size, erosion_size ) );



//	 // gradiant morphololique img 1

//	 Mat eimg1,dimg1,img1NB;
//	 cvtColor(img2, img1NB, CV_RGB2GRAY);

//	 erode( img1NB, eimg1, element );
//     dilate( img1NB, dimg1, element );

//     Mat gradI1=dimg1-eimg1;
//     eimg1.release();
//     dimg1.release();

//     //  gradiant morphololique img 2


//     Mat eimg2,dimg2;
//     Mat img2NB ;
//     cvtColor(img2, img2NB, CV_RGB2GRAY);
//   	 erode( img2NB, eimg2, element );
//     dilate( img2NB, dimg2, element );
//     Mat gradI2=dimg2-eimg2;
//     eimg2.release();
//     dimg2.release();


//     // min des gradients
//      Mat minGrad=min(gradI2,gradI2);


//      // min dans le domaine
//      cv::Mat minGradD(ns,CV_8U,cv::Scalar(0));
//      warpPerspective(minGrad,minGradD,H_1toW,minGradD.size() );//, INTER_LINEAR , BORDER_CONSTANT   );


//#ifdef DEBUG
//      imshow("min ", minGrad);
//      imshow("min Domain", minGradD);
//#endif

//      // préparation pour le watershed
//      //
//	  markers.convertTo(markers,CV_BGR2GRAY);
//#ifdef DEBUG
//      imshow("markers", markers);
//#endif


//      WatershedMarkerSoille   segmenter;

//	  markers.convertTo(markers, CV_32S);
//	  Mat gImg=minGradD;
//	//  cvtColor(minGrad, gImg, CV_RGB2GRAY);
//	    printf("WatershedMarkerSoille ....\n");
//		 segmenter.process(gImg,markers);
//		 printf("WatershedMarkerSoille fin \n");



//		 // cout<< "RESULT MARKER " << markers << endl;
//		 markers.convertTo(markers,CV_8U);



//		 // creation de l'image
//		 cv::Mat rmask=markers;
//		// convertScaleAbs(markers, rmask, 1, 0);

//		//  cout<< "RESULT MARKER ABS " << rmask << endl;


//		 cv::Mat rmask1;

//		 threshold(rmask, rmask1, 0, 1, THRESH_BINARY);
//		// inRange(rmask,0.5,1.5,rmask1);
//#ifdef DEBUG
//		 imshow("mask result 1 ", rmask1);
//#endif

//		 Mat dest1;
//		 bitwise_and(part1, part1, dest1, rmask1);

//		 //dest1.convertTo(dest1,CV_8U);
//#ifdef DEBUG
//		 imshow("final_result img1", dest1);
//#endif
//		 Mat dest2;
//		 cv::Mat rmask2;
//		// bitwise_not(rmask1,rmask2);

//		 threshold(rmask, rmask2, 1, 255, THRESH_BINARY);
//		// inRange(rmask,1.5,2.5,rmask1);

//#ifdef DEBUG
//		 imshow("mask result 2 ", rmask2);
//#endif

//		 bitwise_and(part2, part2, dest2, rmask2);
//		 //dest1.convertTo(dest2,CV_8U);
//#ifdef DEBUG
//		 imshow("final_result img2", dest2);
//#endif

//		 Mat resultF(dest1);
//		 dest2.copyTo(resultF,rmask2);
//#ifdef DEBUG
//		 imshow("final RESULT", resultF);
//#endif

//         imwrite("/tmp/resultStichingCV.jpg",resultF);

//	waitKey(0);
//	return 0;
//}

///** @function readme */
//void readme()
//{ std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }
