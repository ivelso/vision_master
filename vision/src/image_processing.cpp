
#include <iostream>
#include <bits/stdc++.h> 
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/features2d.hpp"
//#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
//#include <visp/vpFeaturePoint.h>
/**
 *This class is responsible for finding the feature points in the image.
 *
 *
 *
 *
 *  pages used as examples for the code
 * https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
 **/

using namespace cv;
using namespace cv::xfeatures2d;

namespace vision
{

    class ImageNode
    {
    public:
      

        ImageNode(void)
        {
            // get the image that is the referance. 
            //img1 = imread("/home/student/catkin_ws/src/vision_master/vision/qr2.png", IMREAD_GRAYSCALE);
            //findFeaturePoints(img1); // find the features to track 
            setTargetPoints();
            this->subscriber_colour_image = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw/", 1, &ImageNode::callback_colour_image, this);




        }

        void callback_colour_image(sensor_msgs::ImageConstPtr const &colour_image)
        {


            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            cv::Mat frame = cv_bridge::toCvShare(colour_image, sensor_msgs::image_encodings::BGR8)->image;

            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            FindBloobs(frame);
            //cv::imshow("cv_img", frame);

            //findFeaturePoints(frame);

        }

        /**
         * set the target position for the features.
        */
        void setTargetPoints()
        {
            //150,150 
            //150,300 
            //set up the target 
            targetKeypoints.push_back(cv::KeyPoint(150, 150, 10));
            targetKeypoints.push_back(cv::KeyPoint(300, 150, 10));
            targetKeypoints.push_back(cv::KeyPoint(150, 300, 10));
            targetKeypoints.push_back(cv::KeyPoint(300, 300, 10));


        }

        void getFeaturePoints(std::vector<KeyPoint>&keypoints)
        {
            keypoints = imageKeypoints;
        }

        /**
         *Find round blobs.
         **/

        void FindBloobs(cv::Mat &frame) {
            std::vector<KeyPoint> keyPoints;
            std::vector<KeyPoint> GoodkeyPoints;
            Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();
            float averageSize= 0;
            // Detect blobs
            detector->detect(frame, keyPoints);


            for (int i =0;i < keyPoints.size(); i++) {
                averageSize += keyPoints[i].size;
                //std::cout<<" number"<<keyPoints[i].pt<< "size"<<keyPoints[i].size<< std::endl;
            }
            averageSize = averageSize/ keyPoints.size();
            // find the average size of the keypoints to filter out other keypoints that is not the target. 
            for (int i =0;i < keyPoints.size(); i++) {
                if (std::abs(keyPoints[i].size-averageSize)<(averageSize/8)) {
                    GoodkeyPoints.push_back(keyPoints[i]);
                }
            }

            cv::Mat keypointsMat;
            drawKeypoints(frame, GoodkeyPoints, keypointsMat, cv::Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            drawKeypoints(keypointsMat, targetKeypoints, keypointsMat, cv::Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::imshow("keypoints", keypointsMat);
            waitKey(2);

        }

        void getTargetPoint(std::vector<KeyPoint>&keypoints)
        {

            keypoints = targetKeypoints;

        }

        /**
         * surf keypoints. find keypoints in the image
         **/
        void findFeaturePoints(cv::Mat &frame) {
            int minHessian = 400;
            Ptr<SURF> detector = SURF::create(minHessian);

            std::vector<KeyPoint> keypoints;
            cv::Mat descriptor;
            detector->detectAndCompute(frame, noArray(), keypoints, descriptor);

            if (firstRound == true) {
                targetKeypoints = keypoints;
                targetDescriptor = descriptor;
                targetFrame = frame;
                firstRound= false;

            }
            else {

                //Ptr<DescriptorMatcher> matcher = BFMatcher::create();
                //std::vector<DMatch> matches;
                //matcher -> knnMatch(descriptor, targetDescriptor, matches,10);
                //printf("image1:%zd keypoints are found.\n", matches.size();

                Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
                std::vector< std::vector<DMatch> > knn_matches;

                if (targetDescriptor.empty() != true && descriptor.empty() != true) {
                    //cvError(0, "MatchFinder", "1st descriptor empty", __FILE__, __LINE__);
                //if (descriptor.empty())
                  ///  cvError(0, "MatchFinder", "2nd descriptor empty", __FILE__, __LINE__);
                    matcher->knnMatch(targetDescriptor, descriptor, knn_matches, 2);

                    //-- Filter matches using the Lowe's ratio test
                    const float ratio_thresh = 0.7f;
                    std::vector<DMatch> good_matches;

                    for (size_t i = 0; i < knn_matches.size(); i++)
                    {
                        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                        {
                            good_matches.push_back(knn_matches[i][0]);

                        }
                    }

                    if (good_matches.size()>0) {
                        //-- Draw matches
                        Mat img_matches;
                        drawMatches(targetFrame, targetKeypoints, frame, keypoints, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                        //-- Show detected matches

                        imshow("Good Matches", img_matches);
                    }
                }
                else {
                    ROS_INFO("missing descriptors from the image stream ");
                }

                cv::waitKey(2);
            }
        }

    private:
        ros::Subscriber subscriber_colour_image, subscriber_depth;
        ros::NodeHandle nh;
        std::vector<KeyPoint> targetKeypoints, imageKeypoints;
        cv::Mat targetDescriptor;
        cv::Mat targetFrame;
        cv::Mat img1;

        bool firstRound = true;

        // Function for calculating median 
        double findMedian(int a[], int n)
        {
            // First we sort the array 
            std::sort(a, a + n);

            // check for even case 
            if (n % 2 != 0)
                return (double)a[n / 2];

            return (double)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
        }

    };
} // namespace vision


int main(int argc, char **argv)
{
    ROS_INFO("Starting ROS Beacon Detector module");
    ros::init(argc, argv, "image_processing");
    vision::ImageNode visionNode;

    ros::spin();
    ROS_INFO("Shutting down ROS Beacon Detector module");
    //visionNode.setTargetPoints();
    //ImageNode::setTargetPoints();
    return 0;
}

//#endif