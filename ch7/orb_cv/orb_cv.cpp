#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{   
    if(argc!=3){
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }
    // image reading
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data!=nullptr && img_2.data!=nullptr);

    // Initialization 
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // First step: Oriented FAST corner point position detection
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // Second step: according to the corner point position to calculate BRIEF descriptor
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"extract ORB cost = "<<time_used.count()<<" seconds. "<<endl;

    Mat outimag1;
    drawKeypoints(img_1, keypoints_1, outimag1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB features", outimag1);
    imwrite("orb_freatures.jpg", outimag1);
    // Third step: to do the matcher work of BRIEF descriptor by using Hamming distance
    vector<DMatch> matches;
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"match ORB cost ="<<time_used.count()<<" seconds. "<<endl;

    // Fouth Steps mathers selection and Calculate the smallest and biggest distance
    auto min_max = minmax_element(matches.begin(), matches.end(), [](const DMatch &m1, const DMatch &m2){return m1.distance<m2.distance;});
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("-- Max dist: %f \n", max_dist);
    printf("-- Min dist: %f \n", min_dist);

    // When the distnace between the descriptors is bigger than the 2 times of the smallest distance,
    // the two descriptors are regarded as mis-matched. But generally the 30 is set as the threshold to
    // avoid the too small distance values
    std::vector<DMatch> good_matches;
    for(int i = 0; i<descriptors_1.rows; i++){
        if(matches[i].distance <= max(2*min_dist, 30.0)){
            good_matches.push_back(matches[i]);}
    }
    
    // Fifth Steps to draw the matches results
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_2, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    imshow("all matches", img_match);
    imwrite("all_matches.jpg", img_match);
    imshow("good matches", img_goodmatch);
    imwrite("good_matches.jpg", img_goodmatch);
    waitKey(0);

    return 0;
}

