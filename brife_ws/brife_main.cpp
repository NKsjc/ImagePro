#include <stdio.h>

#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/features2d/features2d.hpp>

using namespace cv;

int main(int argc, char** argv) 
{ 
    Mat img_1 = imread("/home/jc/slambook/brife_ws/000001.jpg"); 
    Mat img_2 = imread("/home/jc/slambook/brife_ws/000003.jpg");

    // -- Step 1: Detect the keypoints using STAR Detector 
    std::vector<KeyPoint> keypoints_1,keypoints_2; 
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
    cv::Ptr<cv::ORB> orb= cv::ORB::create();
    //StarFeatureDetector detector; 
    orb->detect(img_1, keypoints_1); 
    orb->detect(img_2, keypoints_2);

    // -- Stpe 2: Calculate descriptors (feature vectors) 
    //Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name);
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptor_name);
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //BriefDescriptorExtractor brief; 
    Mat descriptors_1, descriptors_2; 
    descriptor->compute(img_1, keypoints_1, descriptors_1); 
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- Step 3: Matching descriptor vectors with a brute force matcher 
    //BFMatcher matcher(NORM_HAMMING); 
    std::vector<DMatch> mathces,good_matches; 
    matcher->match(descriptors_1, descriptors_2, mathces); 
    double min_d=1000000,max_d=0;
    for(int i=0;i<descriptors_1.rows;i++)
    {
      double dis=mathces[i].distance;
      if(dis<min_d)
	min_d=dis;
      if(dis>max_d)
	max_d=dis;
      
    }
    
    printf("max %f    min%f",max_d,min_d);
    for(int i=0;i<descriptors_1.rows;i++)
    {
      if(mathces[i].distance<=max(2*min_d,20.0))
	good_matches.push_back(mathces[i]);
    }
    // -- dwaw matches 
    Mat img_mathes; 
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_mathes); 
    // -- show 
    imshow("Mathces", img_mathes);

    waitKey(0); 
    cv::destroyWindow("Mathces");
    return 0; 
}
