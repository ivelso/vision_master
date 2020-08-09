
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
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

            //cv::imshow("cv_img", frame);

            int minHessian = 400;
            Ptr<SURF> detector = SURF::create(minHessian);
            std::vector<KeyPoint> keypoints;
            detector->detect(frame, keypoints);
            // show the key points
            Mat img_keypoints;
            drawKeypoints(src, keypoints, img_keypoints);
            //-- Show detected (drawn) keypoints
            imshow("SURF Keypoints", img_keypoints);
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