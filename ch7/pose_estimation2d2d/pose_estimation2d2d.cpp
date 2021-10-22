#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/feature2d/feature2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void find_feature_matches(const Mat &img_1, 
                          const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch>  &matches);

void pose_estimation_2d2d(std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R, 
                          Mat &t);

Point2d pixel2cam(const Point2d &p, const Mat &K);


int main(int argc, char **argv)
{
    if(argc != 3){
        cout<<"usage: pose_estimation_2d2d img1 img2"<<endl;
        return 1;
    }

    // Read the image
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Can not load images");

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;

    // Calculate the orb features between 2 images
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout<<"Totally find "<<matches.size()<<"matches points"<<endl;
    
    // Estimate the motions between the 2 images
    Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

    // Evaluate E=t^R*scale
    Mat t_x = (Mat_<double>(3,3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
                                    t.at<double>(2, 0), 0, -t.at<double>(0, 0),
                                    -t.at<double>(1, 0), t.at<double>(0, 0), 0);

    cout << "t^R=" <<endl << t_x*R<<endl;

    // Evaluate Epipolar Geometry
    Mat K = (Mat_<double>(3,3)<<520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for(DMatch m: matches){
        Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Mat y1 = (Mat_<double>(3, 1)<<pt1.x, pt1.y, 1);
        Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        Mat y2 = (Mat_<double>(3, 1)<<pt2.x, pt2.y, 1);
        Mat d = y2.t()*t_x*R*y1;
        cout<<"epipolar constraint = "<<d<<endl;
    }

    return 0;
}


void find_feature_matches(const Mat &img_1, 
                          const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches){
    // Initialization
    Mat descriptors_1, descriptors_2;

    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create("ORB");
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create("ORB");
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

}
