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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <thread>
#include <chrono>
#include <ros/callback_queue.h>

// visp includes
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpServo.h>
//display image
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpColor.h>

#include <visp3/gui/vpPlot.h>

#include "turtleBot_pan.h"
//#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
//#include <visp/vpFeaturePoint.h>
/**
 *This class is responsible for finding the feature points in the image.
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
    private:
        vpCameraParameters cam;
        uint64_t lastPubTime;
        uint64_t Wait4Image;
        bool newFeatures = false;
        //static ros::SingleThreadedSpinner spinner_a;
        //static std::thread spinner_thread_a;
        ros::NodeHandle n_a;
        //static ros::CallbackQueue *callback_queue_cam;

    public:
        ImageNode()
        {
            lastPubTime = ros::Time::now().toSec();
            Wait4Image = ros::Time::now().toSec();
            cam.initPersProjWithoutDistortion(617.0617065429688, 616.9425659179688, 337.3551940917969, 238.88201904296875); //617.0617065429688, 616.9425659179688
            //K=[617.0617065429688, 0.0, 337.3551940917969, 0.0, 616.9425659179688, 238.88201904296875, 0.0, 0.0, 1.0])
            setTargetPoints(cam);

            //std::this_thread::sleep_for(std::chrono::seconds(3)); // let the system start up first.
            // ros::AsyncSpinner spinner(1); // async spinner due to the moveit library

            // Make one separate thread for the images. this have a separate queue

            // spinner.start();
            // std::thread testthread(&ImageNode::loop, this);
            // testthread.join();
            // kill the threads.
            // ros::waitForShutdown();

            //spinner.stop();
        }

        ~ImageNode()
        {
            // testthread.join();
            // kill the threads.
            //ros::waitForShutdown();

            //spinner_thread_a.join();

            // spinner.stop();
        }
        void initiate()
        {
            ros::CallbackQueue callback_queue_cam;
            n_a.setCallbackQueue(&callback_queue_cam);

            // changed the rostopic to the simulated.
            // this->subscriber_colour_image = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw/", 5, &ImageNode::callback_colour_image, this);
            // this->subscriber_depth = nh.subscribe<sensor_msgs::Image>("camera/aligned_depth_to_color/image_raw/", 5, &ImageNode::callback_depth, this);
            message_filters::Subscriber<sensor_msgs::Image> rgb_image_sub(n_a, "camera/color/image_raw/", 1);
            message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(n_a, "camera/aligned_depth_to_color/image_raw/", 1);
            // http://library.isr.ist.utl.pt/docs/roswiki/message_filters.html
            // message_filters::Subscriber<sensor_msgs::Image> rgb_image_sub(nh, "camera/rgb/image_raw/", 1);//CompressedImage
            // message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, "camera/depth/image_raw/", 1);
            // ensure that the image is with same time stamp.
            message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(rgb_image_sub, depth_image_sub, 10);
            boost::bind(&ImageNode::callback_images, this, _1, _2);
            sync.registerCallback(boost::bind(&ImageNode::callback_images, this, _1, _2));

            std::thread spinner_thread_a([&callback_queue_cam]() {
                ros::SingleThreadedSpinner spinner_a;
                spinner_a.spin(&callback_queue_cam);
            });
            std::cout << "Image processing initiated" << std::endl;
            //spinner.start();
            ros::waitForShutdown();
            spinner_thread_a.join();
        }
        void loop()
        {

            if (imageRecived) // && (ros::Time::now().toSec() - Wait4Image) > 2
            {
                std::cout << ros::Time::now().toSec() - Wait4Image << "time " << std::endl;
                Wait4Image = ros::Time::now().toSec();
                imageRecived = false;
                cv::Mat newImg = currentImg;
                cv::Mat depthframe = currentDepth;
                FindBloobs(newImg); //change to make a different detection method
                // ensure that all feature is found in the image.
                if (imageKeypoints.size() >= numberOfKeypoints)
                {

                    // find the depth to each feature.
                    for (int i = 0; i < numberOfKeypoints; i++)
                    {
                        // for choosing the keypoints.
                        std::cout << "keypoint: " << i << " " << imageKeypoints[i].pt << "target: " << targetKeypoints[i].pt << std::endl;
                        double centre_depth = DEPTH_SCALE * static_cast<double>(depthframe.at<uint16_t>(imageKeypoints[i].pt));
                        //ROS_INFO("centre depth: %.4f", centre_depth);
                        vpImagePoint point(imageKeypoints[i].pt.x, imageKeypoints[i].pt.y);
                        vpFeatureBuilder::create(s[i], cam, point);
                        if (centre_depth > 0)
                        {
                            s[i].set_Z(centre_depth);
                        }
                        else
                        {
                            s[i].set_Z(0.15);
                        }
                        firstRound = false;
                        newFeatures = true;
                    }

                    s_Z.buildFrom(s[1].get_x(), s[1].get_y(), s[1].get_Z(), log(s[1].get_Z() / s_Zd.get_Z()));
                    // print the error in the image in cartesian coordinate.
                    std::cout << s[0].get_x() << " " << s[1].get_x() << " " << s[2].get_x() << " " << s[3].get_x() << std::endl;
                    std::cout << s[0].get_y() << " " << s[1].get_y() << " " << s[2].get_y() << " " << s[3].get_y() << std::endl;
                }
            }
        }

        void getsd(vpFeaturePoint *_sd)
        {

            _sd[0] = sd[0];
            _sd[1] = sd[1];
            _sd[2] = sd[2];
            _sd[3] = sd[3];
        }
        bool gets(vpFeaturePoint *_s)
        {
            if (newFeatures)
            {
                _s[0] = s[0];
                _s[1] = s[1];
                _s[2] = s[2];
                _s[3] = s[3];
                newFeatures = false;
                return true;
            }
            else
            {
                return false;
            }
        }

        /**
         * function that plots the variables to a plot. plots the keypoints in pixel coord, in xy coord error in z distance and the velocity for the joints. 
         * 
         * */
        void ploter(const vpColVector &v, const vpColVector &error)
        {
            static bool initiate = true;
            static vpPlot plotter;
            static unsigned int iter = 0;
            if (initiate)
            {
                initiate = false;
                plotter.init(4, 250 * 4, 1500, 100, 200, "Real time curves plotter");
                plotter.setTitle(0, "Visual features error");
                plotter.setTitle(1, "Camera velocities");
                plotter.setTitle(2, "Distance to target");
                plotter.setTitle(3, "Pixel error");

                plotter.initGraph(0, 8);
                plotter.initGraph(1, 3);
                plotter.initGraph(2, 1);
                plotter.initGraph(3, 8);
                plotter.setLegend(0, 0, "x1");
                plotter.setLegend(0, 1, "y1");
                plotter.setLegend(0, 2, "x2");
                plotter.setLegend(0, 3, "y2");
                plotter.setLegend(0, 4, "x3");
                plotter.setLegend(0, 5, "y3");
                plotter.setLegend(0, 6, "x4");
                plotter.setLegend(0, 7, "y4");

                plotter.setLegend(1, 0, "v_x");
                plotter.setLegend(1, 1, "v_w");
                plotter.setLegend(1, 2, "v_q1");

                plotter.setLegend(3, 0, "x1");
                plotter.setLegend(3, 1, "y1");
                plotter.setLegend(3, 2, "x2");
                plotter.setLegend(3, 3, "y2");
                plotter.setLegend(3, 4, "x3");
                plotter.setLegend(3, 5, "y3");
                plotter.setLegend(3, 6, "x4");
                plotter.setLegend(3, 7, "y4");

                plotter.setLegend(2, 0, "z");
            }
            vpColVector z(1);
            z[0] = (s[1].get_Z() / sd[1].get_Z()) - 1;
            vpColVector keypointsVector(8);
            // find the error for the keypoints
            keypointsVector[0] = imageKeypoints[0].pt.x - targetKeypoints[0].pt.x;
            keypointsVector[1] = imageKeypoints[0].pt.y - targetKeypoints[0].pt.y;
            keypointsVector[2] = imageKeypoints[1].pt.x - targetKeypoints[1].pt.x;
            keypointsVector[3] = imageKeypoints[1].pt.y - targetKeypoints[1].pt.y;
            keypointsVector[4] = imageKeypoints[2].pt.x - targetKeypoints[2].pt.x;
            keypointsVector[5] = imageKeypoints[2].pt.y - targetKeypoints[2].pt.y;
            keypointsVector[6] = imageKeypoints[3].pt.x - targetKeypoints[3].pt.x;
            keypointsVector[7] = imageKeypoints[3].pt.y - targetKeypoints[3].pt.y;

            plotter.plot(0, iter, error);
            plotter.plot(1, iter, v);
            plotter.plot(2, iter, z);
            plotter.plot(3, iter, keypointsVector);
            iter++;
        }

        /**
        * Callback for the depth and rgb image. Stores the images. 
        **/
        void callback_images(const sensor_msgs::ImageConstPtr &colour_image, const sensor_msgs::ImageConstPtr &depth_image)
        {
            //ROS_INFO("got image");

            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            currentImg = cv_bridge::toCvShare(colour_image, sensor_msgs::image_encodings::BGR8)->image; //sensor_msgs::image_encodings::BGR8
            currentDepth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            imageRecived = true;

            //currentDepth
            //cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            //std::cout<<"printing the image "<<std::endl;
            //cv::imshow("keypoints", frame);
            // waitKey(2);
        }

        /**
         * set the target position for the features.
        */
        void setTargetPoints(vpCameraParameters &cam)
        {

            // set point in pixel coord this is also showed in the image
            targetKeypoints.push_back(cv::KeyPoint(317.6557007, 177.0446625, 40));
            targetKeypoints.push_back(cv::KeyPoint(391.3921814, 171.9816589, 40));
            targetKeypoints.push_back(cv::KeyPoint(323.1187439, 250.4288635, 40));
            targetKeypoints.push_back(cv::KeyPoint(396.3076172, 245.7163239, 40));

            vpImagePoint point(targetKeypoints[0].pt.x, targetKeypoints[0].pt.y);
            vpFeatureBuilder::create(sd[0], cam, point);
            sd[0].set_Z(0.251);

            point.set_ij(targetKeypoints[1].pt.x, targetKeypoints[1].pt.y);
            vpFeatureBuilder::create(sd[1], cam, point);
            sd[1].set_Z(0.251);

            point.set_ij(targetKeypoints[2].pt.x, targetKeypoints[2].pt.y);
            vpFeatureBuilder::create(sd[2], cam, point);
            sd[2].set_Z(0.251);

            point.set_ij(targetKeypoints[3].pt.x, targetKeypoints[3].pt.y);
            vpFeatureBuilder::create(sd[3], cam, point);
            sd[3].set_Z(0.251);

            s_Zd.buildFrom(sd[1].get_x(), sd[1].get_y(), sd[1].get_Z(), 0);
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
            SimpleBlobDetector::Params params;
            params.filterByInertia = false;
            params.filterByCircularity = true;

            params.minCircularity = 0.73;

            params.filterByColor = true;
            params.blobColor = 0; //filter only black
            params.filterByArea = true;

            // Filter by Area.
            params.filterByArea = true;
            params.minArea = 20;
            params.maxArea = 3.14159 * 250.0f * 250.0f; // calculate the area of circle

            std::vector<KeyPoint> keyPoints;
            std::vector<KeyPoint> GoodkeyPoints;
            Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
            float averageSize = 0;
            // Detect blobs
            detector->detect(frame, keyPoints);

            for (int i = 0; i < keyPoints.size(); i++)
            {
                averageSize += keyPoints[i].size;

                //std::cout << " number" << keyPoints[i].pt << "size" << keyPoints[i].size << std::endl;
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
            if (GoodkeyPoints.size() < 4)
            {
                missingKeypoint = true;
            }
            else
            {
                missingKeypoint = false;
            }

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
            //remove the old keypoints
            imageKeypoints.clear();
            for (int i = 0; i < numberOfKeypoints; i++)
            {
                imageKeypoints.push_back(sortedKeypoints[i]);
            }

            if (missingKeypoint || debug)
            {
                cv::Mat keypointsMat;
                drawKeypoints(frame, imageKeypoints, keypointsMat, cv::Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                drawKeypoints(keypointsMat, targetKeypoints, keypointsMat, cv::Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                cv::imshow("keypoints", keypointsMat);
                waitKey(2);
            }
        }

        /**
        * get the target points in a opencv keypoint vector. 
        **/
        void getTargetPoint(std::vector<KeyPoint> &keypoints)
        {

            keypoints = targetKeypoints;
        }

    private:
        //ros::Subscriber subscriber_colour_image, subscriber_depth;
        ros::NodeHandle nh;

        std::vector<KeyPoint> targetKeypoints, imageKeypoints;
        cv::Mat currentImg, currentDepth;

        static constexpr double DEPTH_SCALE = 0.001;
        static const int numberOfKeypoints = 4;
        //features for the control
        vpFeaturePoint sd[numberOfKeypoints];
        vpFeaturePoint s[numberOfKeypoints];
        vpFeatureDepth s_Z, s_Zd;
        //variables for the running
        bool imageRecived = false;
        bool firstRound = true;
        bool debug = true;
        bool missingKeypoint = true;
    };
} // namespace vision
  /**
int main(int argc, char **argv)
{
    ROS_INFO("Starting ROS image processing node");
    ros::init(argc, argv, "image_processing");
    vision::ImageNode visionNode;

    return 0;
}
**/
  //#endif