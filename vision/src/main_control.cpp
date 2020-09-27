#include "image_processing.cpp"
#include "visual_control.cpp"
#include "robot_controller.cpp"
#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <visp/vpColVector.h>
int main(int argc, char **argv)
{
    ROS_INFO("Starting ROS main control node");
    ros::init(argc, argv, "main_control");

    static ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;
    vision::ImageNode image_processing;
    //create a thread for the image stream.
    std::thread spinner_thread_a(&vision::ImageNode::initiate, &image_processing);

    vision::VisualControl visualControl;

    vision::RobotController robotController(&nh);

    robotController.setStartPos();

    static const int numberOfKeypoints = 4;
    //features for the control
    vpFeaturePoint sd[numberOfKeypoints];
    vpFeaturePoint s[numberOfKeypoints];
    image_processing.getsd(sd);
    std::vector<double> robotPos;
    robotController.setVelocityBase(0, 0);
    visualControl.setFeatures(s, sd);
    bool driving = true;

    uint64_t Wait4Image = ros::Time::now().toSec();
    while (ros::ok())
    {
        image_processing.loop();

        if (image_processing.gets(s))
        {
            ROS_INFO("getRobot Pos");
            robotController.getRobotPos(robotPos);
            ROS_INFO("visula loop ");
            vpColVector v = visualControl.loop(robotPos);
            ROS_INFO("set joint Pos");

            robotController.setPositonJoints(v[2]);
            ROS_INFO("move base ");
            robotController.setVelocityBase(v[0], v[1]);
            driving = true;
            Wait4Image = ros::Time::now().toSec();

            //  ros::spin();
        }
        else if (driving && ros::Time::now().toSec() - Wait4Image > 0.5)
        {
            robotController.setVelocityBase(0, 0);
            driving = false;
          
        }
    }
    robotController.setVelocityBase(0, 0);
    spinner_thread_a.join();
    spinner.stop();
    // get image features s,sd.
    //calculate the control, return v, given s,sd
    //control the robot v
    //based on th error chose control.

    return 0;
}