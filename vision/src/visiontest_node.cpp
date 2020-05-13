/**
    visiontest.cpp

  http://wiki.ros.org/visp_ros
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
#include <visp3/core/vpImage.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/vision/vpPose.h>
#include <visp_ros/vpROSRobot.h>

// #include <visp_ros/include/visp_ros/vpROSGrabber.h>

// Local Headers

namespace vision
{

  class visionTestNode
  {
  public:
    // Constants
    static constexpr double DEPTH_SCALE = 0.001;
    vpDisplay *disp;
    bool firstRun = true;
    bool init_done = false;
    vpDot2 blob;
    vpImagePoint germ;
    visionTestNode()
    {

      std::cout << "here " << std::endl;
      //this->subscriber_colour_image = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw/", 1, &visionTestNode::callback_colour_image, this);
      //this->subscriber_depth = nh.subscribe<sensor_msgs::Image>("camera/aligned_depth_to_color/image_raw/", 1, &visionTestNode::callback_depth, this);
    }

    void callback_colour_image(sensor_msgs::ImageConstPtr const &colour_image) //sensor_msgs::ImageConstPtr const& colour_image
    {

      vpImage<vpRGBa> vispImage;

      vispImage = visp_bridge::toVispImageRGBa(*colour_image);
      vpImage<unsigned char> greyImage;
      vpImageConvert::convert(vispImage, greyImage);

      if (firstRun)
      {
        //disp = new vpDisplayX();
        disp = new vpDisplayX(greyImage, 0, 0, "camera view");
        disp->init(greyImage);
        // //disp->setTitle("Image processing initialisation interface");
        firstRun = false;
        std::cout << "first run " << std::endl;
        //vpDisplay::display(greyImage);
        //vpDisplay::flush(greyImage);
      }
      // vpDisplay::display(greyImage);
    }
  };
} // namespace vision

int main(int argc, const char **argv)
{
  try
  {
    bool opt_use_camera_info = false;
    for (int i = 0; i < argc; i++)
    {
      if (std::string(argv[i]) == "--use-camera-info")
        opt_use_camera_info = true;
      else if (std::string(argv[i]) == "--help")
      {
        std::cout << "Usage: " << argv[0]
                  << " [--use-camera-info] [--help]"
                  << std::endl;
        return 0;
      }
    }
    std::cout << "Use camera info: " << ((opt_use_camera_info == true) ? "yes" : "no") << std::endl;

    vpImage<unsigned char> I; // Create a gray level image container
    //vpImage<vpRGBa> I; // Create a color image container
    vpROSGrabber g; // Create a grabber based on ROS
    vpDot2 blobDetector;
    blobDetector.setGraphics(true);
    blobDetector.setGraphicsThickness(2);
    vpImagePoint detectedPoints;
    bool init_done = false;

    g.setImageTopic("/camera/color/image_raw/compressed");
    g.setImageTransport("jpeg");
    if (opt_use_camera_info)
    {
      g.setCameraInfoTopic("/camera/color/camera_info");
      g.setRectify(true);
    }

    g.open(I);
    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while (1)
    {
      try
      {
        g.acquire(I);
        vpDisplay::display(I);
        if (!init_done)
        {
          std::cout << "click the blob" << std::endl;
          vpDisplay::displayText(I, vpImagePoint(10, 10), "click the blob", vpColor::red);
          if (vpDisplay::getClick(I, detectedPoints, false))
          {
            std::cout << "should not be her" << std::endl;
            blobDetector.initTracking(I, detectedPoints);
            init_done = true;
          }
        }
        else
        {
          std::cout << "trackiing " << std::endl;
          blobDetector.track(I);
        }
        vpDisplay::displayText(I, 20, 20, "A click to quit...", vpColor::red);
        vpDisplay::flush(I);
        if (vpDisplay::getClick(I, false))
          break;
      }

      catch (...)
      {
        init_done = false;
      }
    }
  }
  catch (vpException e)
  {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
