//#include "robustmatcher.h"

//using namespace std;
//using namespace cv;

//// Source : https://github.com/zixuanwang/Research/blob/master/Coclustering/RobustMatcher.cpp

//RobustMatcher::RobustMatcher() :
//        ratio(0.8), distance(1.0) {
//    // SURF is the default feature
//    detector = new cv::SurfFeatureDetector();
//    extractor = new cv::SurfDescriptorExtractor();
//}

//RobustMatcher::~RobustMatcher() {
//    // TODO Auto-generated destructor stub
//}

//Mat RobustMatcher::match(const Mat& image1,
//                             const Mat& image2,
//                             vector<KeyPoint>* pKeypoint1,
//                             vector<KeyPoint>* pKeypoint2)
//{
//    pKeypoint1->clear();
//    pKeypoint2->clear();

//    if (image1.empty() || image2.empty())
//    {
//        return Mat();
//    }
//   // vector<DMatch> matchArray;
//    vector<KeyPoint> keypoint1;
//    vector<KeyPoint> keypoint2;
//    Mat imageDescriptor1;
//    Mat imageDescriptor2;
//    detector->detect(image1,keypoint1);
//    extractor->compute(image1,keypoint1,imageDescriptor1);
//    detector->detect(image2,keypoint2);
//    extractor->compute(image2,keypoint2,imageDescriptor2);
//    cv::BFMatcher matcher(NORM_L2);
//    // from image 1 to image 2
//    // based on k nearest neighbours (with k=2)
//    vector<vector<DMatch> > matches1;
//    matcher.knnMatch(imageDescriptor1, imageDescriptor2, matches1, 2); // vector of matches (up to 2 per entry)

//    // return 2 nearest neighbours
//    // from image 2 to image 1
//    // based on k nearest neighbours (with k=2)
//    vector<vector<DMatch> > matches2;
//    matcher.knnMatch(imageDescriptor2, imageDescriptor1, matches2, // vector of matches (up to 2 per entry)
//            2);
//    // return 2 nearest neighbours
//    // 3. Remove matches for which NN ratio is
//    // > than threshold
//    // clean image 1 -> image 2 matches
//    ratioTest(matches1);
//    // clean image 2 -> image 1 matches
//    ratioTest(matches2);
//    // 4. Remove non-symmetrical matches
//    vector<DMatch> symMatches;
//    symmetryTest(matches1, matches2, symMatches);
//    if (symMatches.size() < 10) {
//        return Mat();
//    }
//    // 5. Validate matches using RANSAC
//    Mat homography = ransacTest(symMatches, keypoint1, keypoint2,
//            matchArray);
//    for (size_t i = 0; i < matchArray.size(); ++i) {
//        pKeypoint1->push_back(keypoint1[matchArray[i].queryIdx]);
//        pKeypoint2->push_back(keypoint2[matchArray[i].trainIdx]);
//    }
//    // return the found homography matrix
//    return homography;
//}

//Mat RobustMatcher::patch(const Mat& image,
//        const vector<KeyPoint>& keypointArray) {
//    Mat points((int) keypointArray.size(), 1, CV_32SC2);
//    for (size_t i = 0; i < keypointArray.size(); ++i) {
//        int* ptr = points.ptr<int>((int) i);
//        ptr[0] = (int) keypointArray[i].pt.x;
//        ptr[1] = (int) keypointArray[i].pt.y;
//    }
//    cv::Rect bbox = cv::boundingRect(points);
//    return image(bbox).clone();
//}

//// Clear matches for which NN ratio is > than threshold
//// return the number of removed points
//// (corresponding entries being cleared,
//// i.e. size will be 0)
//int RobustMatcher::ratioTest(vector<vector<DMatch> > &matches) {
//    int removed = 0;
//    // for all matches
//    for (vector<vector<DMatch> >::iterator matchIterator =
//            matches.begin(); matchIterator != matches.end(); ++matchIterator) {
//        // if 2 NN has been identified
//        if (matchIterator->size() > 1) {
//            // check distance ratio
//            if ((*matchIterator)[0].distance / (*matchIterator)[1].distance
//                    > ratio) {
//                matchIterator->clear(); // remove match
//                removed++;
//            }
//        } else { // does not have 2 neighbours
//            matchIterator->clear(); // remove match
//            removed++;
//        }
//    }
//    return removed;
//}
//// Insert symmetrical matches in symMatches vector
//void RobustMatcher::symmetryTest(
//        const vector<vector<DMatch> >& matches1,
//        const vector<vector<DMatch> >& matches2,
//        vector<DMatch>& symMatches) {
//    // for all matches image 1 -> image 2
//    for (vector<vector<DMatch> >::const_iterator matchIterator1 =
//            matches1.begin(); matchIterator1 != matches1.end();
//            ++matchIterator1) {
//        // ignore deleted matches
//        if (matchIterator1->size() < 2)
//            continue;
//        // for all matches image 2 -> image 1
//        for (vector<vector<DMatch> >::const_iterator matchIterator2 =
//                matches2.begin(); matchIterator2 != matches2.end();
//                ++matchIterator2) {
//            // ignore deleted matches
//            if (matchIterator2->size() < 2)
//                continue;
//            // Match symmetry test
//            if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx
//                    && (*matchIterator2)[0].queryIdx
//                            == (*matchIterator1)[0].trainIdx) {
//                // add symmetrical match
//                symMatches.push_back(
//                        DMatch((*matchIterator1)[0].queryIdx,
//                                (*matchIterator1)[0].trainIdx,
//                                (*matchIterator1)[0].distance));
//                break; // next match in image 1 -> image 2
//            }
//        }
//    }
//}

//// Identify good matches using RANSAC
//// Return homography matrix
//Mat RobustMatcher::ransacTest(const vector<DMatch>& matches,
//        const vector<KeyPoint>& keypoints1,
//        const vector<KeyPoint>& keypoints2,
//        vector<DMatch>& outMatches) {
//    // Convert keypoints into Point2f
//    vector<cv::Point2f> points1, points2;
//    for (vector<DMatch>::const_iterator it = matches.begin();
//            it != matches.end(); ++it) {
//        // Get the position of left keypoints
//        float x = keypoints1[it->queryIdx].pt.x;
//        float y = keypoints1[it->queryIdx].pt.y;
//        points1.push_back(cv::Point2f(x, y));
//        // Get the position of right keypoints
//        x = keypoints2[it->trainIdx].pt.x;
//        y = keypoints2[it->trainIdx].pt.y;
//        points2.push_back(cv::Point2f(x, y));
//    }
//    vector<uchar> inliers(points1.size(), 0);
//    Mat homography = cv::findHomography(Mat(points1), Mat(points2),
//            inliers, CV_RANSAC, distance);
//    // extract the surviving (inliers) matches
//    vector<uchar>::const_iterator itIn = inliers.begin();
//    vector<DMatch>::const_iterator itM = matches.begin();
//    // for all matches
//    for (; itIn != inliers.end(); ++itIn, ++itM) {
//        if (*itIn) { // it is a valid match
//            outMatches.push_back(*itM);
//        }
//    }
//    return homography;
//}

//// Set the feature detector
//void RobustMatcher::setFeatureDetector(cv::Ptr<cv::FeatureDetector>& detect) {
//    detector = detect;
//}
//// Set the descriptor extractor
//void RobustMatcher::setDescriptorExtractor(
//        cv::Ptr<cv::DescriptorExtractor>& desc) {
//    extractor = desc;
//}

//void RobustMatcher::show(const Mat& image1, const Mat& image2, const vector<KeyPoint>& keypointArray1, const vector<KeyPoint>& keypointArray2, const string& outputPath){
//    int height=max(image1.rows,image2.rows);
//    int width=image1.cols+image2.cols;
//    Mat corres(height,width,CV_8UC3,cv::Scalar(0,0,0,0));
//    Mat imageTarget1=corres(cv::Rect(0,0,image1.cols,image1.rows));
//    image1.copyTo(imageTarget1);
//    Mat imageTarget2=corres(cv::Rect(image1.cols,0,image2.cols,image2.rows));
//    image2.copyTo(imageTarget2);
//    for(size_t i=0;i<keypointArray1.size()&&i<keypointArray2.size();++i){
//       // cv::line(corres,keypointArray1[i].pt,cv::Point(keypointArray2[i].pt.x+image1.cols,keypointArray2[i].pt.y),CV_RGB(255,0,0),1,CV_AA);
//    }
//    if(outputPath.empty()){
//        cv::namedWindow("corres");
//        cv::imshow("corres",corres);
//        cv::waitKey(0);
//    }else{
//        cv::imwrite(outputPath,corres);
//    }
//}
