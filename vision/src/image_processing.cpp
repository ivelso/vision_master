
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/features2d.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
/**
 *This class is responsible for finding the feature points in the image.
 *
 *
 *
 *
 **/

using namespace cv;
using namespace cv::xfeatures2d;

namespace vision
{

    class ImageNode
    {
    public:
        /*
                ImageNode();
                void setTargetPoints();
                void getTargetPoint();
                void getFeaturePoints();
                */

        ImageNode(void)
        {
            this->subscriber_colour_image = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw/", 1, &ImageNode::callback_colour_image, this);

        }

        void callback_colour_image(sensor_msgs::ImageConstPtr const &colour_image)
        {


            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            cv::Mat frame = cv_bridge::toCvShare(colour_image, sensor_msgs::image_encodings::BGR8)->image;
            cv::Mat img_transformed;
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

            //cv::imshow("cv_img", frame);

            int minHessian = 400;
            Ptr<SURF> detector = SURF::create(minHessian);

            std::vector<KeyPoint> keypoints;
            cv::Mat descriptor;
            detector->detectAndCompute(frame, noArray(), keypoints, descriptor);

            if (firstRound == true) {
                TargetKeypoints = keypoints;
                targetDescriptor = descriptor;
                targetFrame = frame;
            }
            firstRound= false;
            //Ptr<DescriptorMatcher> matcher = BFMatcher::create();
            //std::vector<DMatch> matches;
            //matcher -> knnMatch(descriptor, targetDescriptor, matches,10);
            //printf("image1:%zd keypoints are found.\n", matches.size();

            Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
            std::vector< std::vector<DMatch> > knn_matches;
           
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
             
            //-- Draw matches
            Mat img_matches;
            drawMatches(targetFrame, TargetKeypoints, frame, keypoints, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            //-- Show detected matches
           
            imshow("Good Matches", img_matches);


            // show the key points
           // Mat img_keypoints;

            //drawKeypoints(frame, keypoints, img_keypoints);
             //-- Show detected (drawn) keypoints

             
            
           // imshow("SURF Keypoints", img_keypoints);
            //imshow("SURF ", targetFrame);
            cv::waitKey(2);
        }

        void setTargetPoints()
        {
        }

        void getFeaturePoints()
        {
        }

        void getTargetPoint()
        {
        }

    private:
        ros::Subscriber subscriber_colour_image, subscriber_depth;
        ros::NodeHandle nh;
        std::vector<KeyPoint> TargetKeypoints;
        cv::Mat targetDescriptor;
        cv::Mat targetFrame;
        bool firstRound = true;

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

#endif