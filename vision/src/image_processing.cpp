
#include <iostream>
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include "opencv2/features2d.hpp"

// visp includes
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>

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
            // changed the rostopic to the simulated.
            //this->subscriber_colour_image = nh.subscribe<sensor_msgs::Image>("camera/rgb/image_raw/", 1, &ImageNode::callback_colour_image, this);
            //this->subscriber_depth = nh.subscribe<sensor_msgs::Image>("camera/depth/image_raw/", 1, &ImageNode::callback_depth, this);
            message_filters::Subscriber<sensor_msgs::Image> rgb_image_sub(nh, "camera/color/image_raw/", 1);
            message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, "/camera/depth/image_rect_raw", 1);
            // http://library.isr.ist.utl.pt/docs/roswiki/message_filters.html
            //message_filters::Subscriber<sensor_msgs::Image> rgb_image_sub(nh, "camera/rgb/image_raw/", 1);
            //message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, "camera/depth/image_raw/", 1);
            message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(rgb_image_sub, depth_image_sub, 10);
            //boost::bind(&ImageNode::callback_images, this,_1, _2);
            sync.registerCallback(boost::bind(&ImageNode::callback_images, this, _1, _2));
            ros::Rate loop_rate(10);
            vpServo task;
            task.setServo(vpServo::EYEINHAND_CAMERA);
            task.setInteractionMatrixType(vpServo::CURRENT);
            task.setLambda(0.5);
            task.addFeature(s[0],sd[0] );
            task.addFeature(s[1],sd[1] );
            task.addFeature(s[2],sd[2] );
            task.addFeature(s[3],sd[3] );
            while (ros::ok())
            {
                if (!firstRound){
                        vpColVector v = task.computeControlLaw();
                        std::cout<< v<<std::endl;
                }
            
                ros::spinOnce();
                loop_rate.sleep();
                // ROS_INFO("");
            }
            task.kill();
        }

        void callback_images(const sensor_msgs::ImageConstPtr &colour_image, const sensor_msgs::ImageConstPtr &depth_image)
        {
            //ROS_INFO("got image");

            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            cv::Mat frame = cv_bridge::toCvShare(colour_image, sensor_msgs::image_encodings::BGR8)->image;

            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            FindBloobs(frame);

            //ROS_INFO("callback_depth()");
            

            if (imageKeypoints.size() >= numberOfKeypoints)
            {
                // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
                cv::Mat depthframe = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image; // Note encoding

                // find the depth to each feature.
                for (int i = 0; i < numberOfKeypoints; i++)
                {
                    double centre_depth = DEPTH_SCALE * static_cast<double>(depthframe.at<uint16_t>(imageKeypoints[i].pt));
                    //ROS_INFO("centre depth: %.4f", centre_depth);
                    if (centre_depth > 0)
                    {
                        s[i].buildFrom(imageKeypoints[i].pt.x, imageKeypoints[i].pt.y, centre_depth);
                    }
                    else
                    {
                        s[i].buildFrom(imageKeypoints[i].pt.x, imageKeypoints[i].pt.y, 0.30);

                    }
                    firstRound= false;
                    // s[i].print();
                }
                // ROS_INFO("thats the keypoints ");

            }
        }
        //example
        void callback_colour_image(sensor_msgs::ImageConstPtr const &colour_image)
        {

            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            cv::Mat frame = cv_bridge::toCvShare(colour_image, sensor_msgs::image_encodings::BGR8)->image;

            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            FindBloobs(frame);
            //cv::imshow("cv_img", frame);

            //findFeaturePoints(frame);
        }
        // example
        void callback_depth(sensor_msgs::ImageConstPtr const &depth_image)
        {
            ROS_INFO("callback_depth()");

            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            cv::Mat frame = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image; // Note encoding

            // Find the depth of the centre pixel in metres
            cv::Point2i centre_point(frame.cols / 2, frame.rows / 2);
            double centre_depth = DEPTH_SCALE * static_cast<double>(frame.at<uint16_t>(centre_point));
            ROS_INFO("centre depth: %.4f", centre_depth);

            //cv::imshow("cv_depth", frame);
            //cv::waitKey(2);
        }

        void getObjectPosFromCam()
        {
        }

        /**
         * set the target position for the features.
        */
        void setTargetPoints()
        {
            //150,150
            //150,300
            //set up the target
            //numberOfKeypoints = 4;
            // used to se the setpoint.
            targetKeypoints.push_back(cv::KeyPoint(150, 150, 20));
            targetKeypoints.push_back(cv::KeyPoint(300, 150, 20));
            targetKeypoints.push_back(cv::KeyPoint(150, 300, 20));
            targetKeypoints.push_back(cv::KeyPoint(300, 300, 20));

            sd[0].buildFrom(150, 150, 0.1);
            sd[1].buildFrom(300, 150, 0.1);
            sd[2].buildFrom(150, 300, 0.1);
            sd[3].buildFrom(300, 300, 0.1);
        }

        void getFeaturePoints(std::vector<KeyPoint> &keypoints)
        {
            keypoints = imageKeypoints;
        }

        /**
         *Find round blobs.
         **/

        void FindBloobs(cv::Mat &frame)
        {
            std::vector<KeyPoint> keyPoints;
            std::vector<KeyPoint> GoodkeyPoints;
            Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();
            float averageSize = 0;
            // Detect blobs
            detector->detect(frame, keyPoints);

            for (int i = 0; i < keyPoints.size(); i++)
            {
                averageSize += keyPoints[i].size;

                std::cout << " number" << keyPoints[i].pt << "size" << keyPoints[i].size << std::endl;
            }
            averageSize = averageSize / keyPoints.size();
            // find the average size of the keypoints to filter out other keypoints that is not the target.
            float sumX = 0, sumY = 0, minX = 0, minY = 0;
            for (int i = 0; i < keyPoints.size(); i++)
            {
                if (std::abs(keyPoints[i].size - averageSize) < (averageSize / 8) && GoodkeyPoints.size() < numberOfKeypoints)
                {
                    GoodkeyPoints.push_back(keyPoints[i]);
                    // find the center of grevety of the features.
                    sumX = sumX + keyPoints[i].pt.x;
                    sumY = sumY + keyPoints[i].pt.y;
                    if (minX == 0 || minX > keyPoints[i].pt.x)
                    {
                        minX = keyPoints[i].pt.x;
                    }
                    if (minY == 0 || minY > keyPoints[i].pt.y)
                    {
                        minY = keyPoints[i].pt.y;
                    }
                }
            }
            KeyPoint sortedKeypoints[4];
            float centerX = sumX / numberOfKeypoints;
            float centerY = sumY / numberOfKeypoints;
            for (int i = 0; i < GoodkeyPoints.size(); i++)
            {
                if (GoodkeyPoints[i].pt.x < centerX && GoodkeyPoints[i].pt.y < centerY)
                {
                    sortedKeypoints[0] = GoodkeyPoints[i];
                }
                if (GoodkeyPoints[i].pt.x > centerX && GoodkeyPoints[i].pt.y < centerY)
                {
                    sortedKeypoints[1] = GoodkeyPoints[i];
                }
                if (GoodkeyPoints[i].pt.x < centerX && GoodkeyPoints[i].pt.y > centerY)
                {
                    sortedKeypoints[2] = GoodkeyPoints[i];
                }
                if (GoodkeyPoints[i].pt.x > centerX && GoodkeyPoints[i].pt.y > centerY)
                {
                    sortedKeypoints[3] = GoodkeyPoints[i];
                }
            }

            /*
            for (int i = 0; i < GoodkeyPoints.size(); i++){
                if (sortedKeypoints.size()==0){
                    sortedKeypoints.push_back(GoodkeyPoints[0]);
                }
                if (sortedKeypoints[sortedKeypoints.size-1].pt.x>GoodkeyPoints[i].pt.x){
                    sortedKeypoints[sortedKeypoints.size-1] = 
                }

                
            }
            */
            imageKeypoints.clear();
            for (int i = 0; i < numberOfKeypoints; i++)
            {
                imageKeypoints.push_back(sortedKeypoints[i]);
            }
            //imageKeypoints(std::begin(sortedKeypoints), std::end(sortedKeypoints));

            //sort(imageKeypoints.begin(), imageKeypoints.end());
            cv::Mat keypointsMat;
            drawKeypoints(frame, imageKeypoints, keypointsMat, cv::Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            drawKeypoints(keypointsMat, targetKeypoints, keypointsMat, cv::Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::imshow("keypoints", keypointsMat);
            waitKey(2);
        }

        void getTargetPoint(std::vector<KeyPoint> &keypoints)
        {

            keypoints = targetKeypoints;
        }

        /**
         * surf keypoints. find keypoints in the image
         **/
        void findFeaturePoints(cv::Mat &frame)
        {
            int minHessian = 400;
            Ptr<SURF> detector = SURF::create(minHessian);

            std::vector<KeyPoint> keypoints;
            cv::Mat descriptor;
            detector->detectAndCompute(frame, noArray(), keypoints, descriptor);

            if (firstRound == true)
            {
                targetKeypoints = keypoints;
                targetDescriptor = descriptor;
                targetFrame = frame;
                firstRound = false;
            }
            else
            {

                //Ptr<DescriptorMatcher> matcher = BFMatcher::create();
                //std::vector<DMatch> matches;
                //matcher -> knnMatch(descriptor, targetDescriptor, matches,10);
                //printf("image1:%zd keypoints are found.\n", matches.size();

                Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
                std::vector<std::vector<DMatch>> knn_matches;

                if (targetDescriptor.empty() != true && descriptor.empty() != true)
                {
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

                    if (good_matches.size() > 0)
                    {
                        //-- Draw matches
                        Mat img_matches;
                        drawMatches(targetFrame, targetKeypoints, frame, keypoints, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                        //-- Show detected matches

                        imshow("Good Matches", img_matches);
                    }
                }
                else
                {
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
        static constexpr double DEPTH_SCALE = 0.001;
        static const int numberOfKeypoints = 4;
        vpFeaturePoint sd[numberOfKeypoints];
        vpFeaturePoint s[numberOfKeypoints];
    

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
    ROS_INFO("Starting ROS image processing node ");
    ros::init(argc, argv, "image_processing");
    vision::ImageNode visionNode;
    ROS_INFO("ros spin ");
    ros::spin();
    ROS_INFO("Shutting down ROS Beacon Detector module");
    //visionNode.setTargetPoints();
    //ImageNode::setTargetPoints();
    return 0;
}

//#endif