/**
    visiontest.cpp

    A ROS node that searching in colour images for any beacons and tries to 
    find its location relative to the robot.

    Subscribed: camera/colour/image_raw/compressed
    Publishes: 

    Created: 2020/02/06
    Author: Brendan Halloran
**/

// Standard Library
#include <string>
#include <vector>

// External Libraries
#include <XmlRpcException.h>
#include <XmlRpcValue.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <visp_bridge/image.h>
#include "visp/vpDisplayX.h"
#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp_ros/vpROSGrabber.h>
#include <iostream>
// #include <visp_ros/include/visp_ros/vpROSGrabber.h>

// Local Headers

namespace vision
{
   

    class visionTestNode
    {
    public:
        // Constants
        static constexpr double DEPTH_SCALE = 0.001;
        vpDisplay* disp;
        bool firstRun = true;
        bool init_done = false;
        vpDot2 blob;
        vpImagePoint germ;
        visionTestNode()
        {
          
            std::cout << "here " << std::endl;
            this->subscriber_colour_image = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw/", 1, &visionTestNode::callback_colour_image, this);
            this->subscriber_depth = nh.subscribe<sensor_msgs::Image>("camera/aligned_depth_to_color/image_raw/", 1, &visionTestNode::callback_depth, this);

        }

        void callback_colour_image(sensor_msgs::ImageConstPtr const& colour_image )//sensor_msgs::ImageConstPtr const& colour_image 
        {
          
            vpImage<vpRGBa> vispImage;
           
            vispImage = visp_bridge::toVispImageRGBa(*colour_image);
            vpImage<unsigned char> greyImage;
            vpImageConvert::convert(vispImage, greyImage);

            if (firstRun){
                //disp = new vpDisplayX();
                disp = new vpDisplayX(greyImage, 0, 0, "camera view");
                disp->init(greyImage);
               // //disp->setTitle("Image processing initialisation interface");
                firstRun= false;
                std::cout << "first run " << std::endl;
                //vpDisplay::display(greyImage);
                //vpDisplay::flush(greyImage);
            }
            // vpDisplay::display(greyImage);
            
            /**
            try{
              if (!init_done) {
                  std::cout << "initdone  " << init_done << std::endl;
            disp->displayText(greyImage, vpImagePoint(10, 10), "Click in the blob to initialize the tracker", vpColor::red);
            vpDisplay::flush(greyImage);
            while (!init_done ){
            std::cout << "first run " << std::endl;
            if (vpDisplay::getClick(greyImage, germ, false)) {
               std::cout << "init done  " << std::endl;
                blob.setGraphics(true);
                blob.setGraphicsThickness(2);
                blob.initTracking(greyImage, germ);
                init_done = true;
                }}}

             else {
                blob.track(greyImage,germ);
                std::cout << "blob  " << std::endl;
            }
            }
            catch (...){
                init_done=false;
            }
            disp->flush(greyImage);
            std::cout << "flush  " << std::endl;

            **/
          
           
            disp->display(greyImage);
            disp->flushDisplay();
            
             

            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            //cv::Mat frame = cv_bridge::toCvShare(colour_image, sensor_msgs::image_encodings::BGR8)->image;
            // vpImage<vpRGBa> vispImage;
            /**
           
            #ifdef VISP_HAVE_X11
            vpDisplayX d(vispImage);
            #else
             std::cout << "No image viewer is available..." << std::endl;
            #endif
            // vpImageConvert::convert(&frame,vispImage);
         **/
            std::cout << "Image size: " << vispImage.getWidth() << " " << vispImage.getHeight() << std::endl;
            //vpDisplay::setTitle(vispImage,"image");
            //vpDisplay::display(vispImage);
            // vpDisplay::flush(vispImage);
            // cv::imshow("cv_img", frame);
           // cv::waitKey(2);
        }

        void callback_depth(sensor_msgs::ImageConstPtr const& depth_image)
        {
            ROS_INFO("callback_depth()");

            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            cv::Mat frame = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image; // Note encoding

            // Find the depth of the centre pixel in metres
            cv::Point2i centre_point(frame.cols / 2, frame.rows / 2);
            double centre_depth = DEPTH_SCALE * static_cast<double>(frame.at<uint16_t>(centre_point));
            ROS_INFO("centre depth: %.4f", centre_depth);

            // cv::imshow("cv_depth", frame);
            cv::waitKey(2);
        }

    private:
        ros::Subscriber subscriber_colour_image, subscriber_depth;
        ros::NodeHandle nh;
   
    };
} // namespace ecte477
/**
int main(int argc, char** argv)
{
    ROS_INFO("Starting ROS Beacon Detector module");
    ros::init(argc, argv, "visiontest_node");
   
    vision::visionTestNode bd;
    /**
    vpImage<vpRGBa> vispImage;
    #ifdef VISP_HAVE_X11
    std::cout << "image" << std::endl;
    ROS_INFO("Starting ROS");
    vpDisplayX d(vispImage);
    ROS_INFO("Starting ROS Beacon  module");
    #else
    ROS_INFO("Starting ROS  Detector module");
    std::cout << "No image viewer is available..." << std::endl;
    #endif
    **/
/***
    ros::spin();
    ROS_INFO("Shutting down ROS Beacon Detector module");

    return 0;
}
**/



int main(int argc, const char** argv)
{
  try {
    bool opt_use_camera_info = false;
    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--use-camera-info")
        opt_use_camera_info = true;
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0]
                  << " [--use-camera-info] [--help]"
                  << std::endl;
        return 0;
      }
    }
    std::cout << "Use camera info: " << ((opt_use_camera_info == true) ? "yes" : "no") << std::endl;

    //vpImage<unsigned char> I; // Create a gray level image container
    vpImage<vpRGBa> I; // Create a color image container
    vpROSGrabber g; // Create a grabber based on ROS

    g.setImageTopic("/camera/color/image_raw");
    if (opt_use_camera_info) {
      g.setCameraInfoTopic("/camera/color/camera_info");
      g.setRectify(true);
    }

    g.open(I);
    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;
         bool firstRun = true;
        bool init_done = false;
        vpDot2 blob;
        vpImagePoint germ;
#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
   vpImage<unsigned char> greyImage;
    while(1) {
      g.acquire(I);
      vpImageConvert::convert(I, greyImage);
      vpDisplay::display(greyImage);
      vpDisplay::displayText(greyImage, 20, 20, "A click to quit...", vpColor::red);
      vpDisplay::flush(greyImage);
        


            try{
              if (!init_done) {
            std::cout << "initdone  " << init_done << std::endl;
            vpDisplay::displayText(greyImage, vpImagePoint(10, 10), "Click in the blob to initialize the tracker", vpColor::red);
            vpDisplay::flush(greyImage);
            while (!init_done ){
            std::cout << "first run " << std::endl;
            if (vpDisplay::getClick(greyImage, germ, false)) {
               std::cout << "init done  " << std::endl;
                blob.setGraphics(true);
                blob.setGraphicsThickness(2);
                blob.initTracking(greyImage, germ);
                init_done = true;
                }}}

             else {
                blob.track(greyImage,germ);
                std::cout << "blob  " << std::endl;
            }
            }
            catch (...){
                init_done=false;
            }



      if (vpDisplay::getClick(I, false))
        break;
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}