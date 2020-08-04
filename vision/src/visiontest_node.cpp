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
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vs/vpServo.h>

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

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;
  double x = 0, y = 0;
  for (unsigned int i = 0; i < point.size(); i++)
  {
    vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }
  if (init == true)
  {
    vpHomogeneousMatrix cMo_dem;
    vpHomogeneousMatrix cMo_lag;
    pose.computePose(vpPose::DEMENTHON, cMo_dem);
    pose.computePose(vpPose::LAGRANGE, cMo_lag);
    double residual_dem = pose.computeResidual(cMo_dem);
    double residual_lag = pose.computeResidual(cMo_lag);
    if (residual_dem < residual_lag)
      cMo = cMo_dem;
    else
      cMo = cMo_lag;
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);
}

int main(int argc, const char **argv)
{
  try
  {
    bool opt_use_camera_info = true;
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
    bool init_done = true;

    g.setImageTopic("/camera/color/image_raw/compressed");
    g.setImageTransport("jpeg");
    if (opt_use_camera_info)
    {
      g.setCameraInfoTopic("/camera/color/camera_info");
      g.setRectify(true);
    }

    // 3D model of the QRcode: here we consider a 12cm by 12cm QRcode
    std::vector<vpPoint> point;
    point.push_back(vpPoint(-0.06, -0.06, 0)); // QRcode point 0 3D coordinates in plane Z=0
    point.push_back(vpPoint(0.06, -0.06, 0));  // QRcode point 1 3D coordinates in plane Z=0
    point.push_back(vpPoint(0.06, 0.06, 0));   // QRcode point 2 3D coordinates in plane Z=0
    point.push_back(vpPoint(-0.06, 0.06, 0));  // QRcode point 3 3D coordinates in plane Z=0

    vpHomogeneousMatrix cMo;                       //current pose of point inimage frame.
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0); //setpoint
    vpDetectorQRCode detector;

    // ibvs parameters
    vpHomogeneousMatrix wMo; // world to object
    //robot position world to camera
    vpHomogeneousMatrix wMc(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);// changed to give out the velocity 
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);
    vpFeaturePoint pd[4], fp[4]; // desired points
    bool init_tracker = false;
    g.open(I);
    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;
    vpCameraParameters cam;
    bool camerainfoRecived = false;
    //get the camera info from the topic
    while (!camerainfoRecived)
    {
      camerainfoRecived = g.getCameraInfo(cam);
    }

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while (1)
    {

      g.acquire(I);
      bool status = detector.detect(I);
      vpDisplay::display(I);

      std::ostringstream legend;
      legend << detector.getNbObjects() << " bar code detected";
      vpDisplay::displayText(I, (int)I.getHeight() - 30, 10, legend.str(), vpColor::red);

      vpDisplay::displayText(I, (int)I.getHeight() - 30, 10, legend.str(), vpColor::red);
      if (status)
      { // true if at least one QRcode is detected
        for (size_t i = 0; i < detector.getNbObjects(); i++)
        {
          std::vector<vpImagePoint> p = detector.getPolygon(i); // get the four corners location in the image
                                                                // vpFeaturePoint fP
    

          for (size_t j = 0; j < p.size(); j++)
          {
            vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
            std::ostringstream number;
            number << j;
            vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
          }
          // compute the pose for where the camera is in comparison to the qr code.
          computePose(point, p, cam, init_done, cMo); // resulting pose is available in cMo var
          for (unsigned int i = 0; i < 4; i++)
          {
            if (!init_tracker)
            {
              std::cout<<i;
              point[i].track(cdMo);
              vpFeatureBuilder::create(pd[i], point[i]);
              point[i].track(cMo);
              vpFeatureBuilder::create(fp[i], point[i]);
              task.addFeature(fp[i], pd[i]);
              init_tracker=true;
            }
              point[i].track(cMo);
              vpFeatureBuilder::create(fp[i], point[i]);

       
          }
               vpColVector v = task.computeControlLaw();
               
            std::cout << v << "vector"<< std::endl;
          std::cout << "Pose translation (meter): " << cMo.getTranslationVector().t() << std::endl
                    << "Pose rotation (quaternion): " << vpQuaternionVector(cMo.getRotationMatrix()).t() << std::endl;
          vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
        }
      }
      vpDisplay::displayText(I, 20, 20, "A click to quit...", vpColor::red);
      vpDisplay::flush(I);
      // compute the velocity for the robot.

      if (vpDisplay::getClick(I, false))
        break;
    }
    task.kill();
  }
  catch (vpException e)
  {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
