#include "image_processing.cpp"
#include "visual_control.cpp"
#include "robot_controller.cpp"
#include <ros/ros.h>
#include <iostream>
#include<fstream>
#include <thread>
#include <visp/vpColVector.h>

/**
 * This main is resposible for controlling a robot using image based visual servoing. 
 * 
 *  Author: Sondre Iveland 
 * */
int main(int argc, char **argv)
{
    ROS_INFO("Starting ROS main control node");
    ros::init(argc, argv, "main_control");

    static ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;
    vision::ImageNode image_processing;
    //create a thread for the image stream.

    vision::VisualControl visualControl;

    vision::RobotController robotController(&nh);

    //robotController.setStartPos();

    static const int numberOfKeypoints = 4;
    //features for the control
    vpFeaturePoint sd[numberOfKeypoints];
    vpFeaturePoint s[numberOfKeypoints];
    image_processing.getsd(sd);
    std::vector<double> robotPos;
    robotController.setVelocityBase(0, 0);
    visualControl.setFeatures(s, sd);
    bool driving = true;

    uint64_t wait4Image = ros::Time::now().toSec();
     uint64_t lastPubTime = ros::Time::now().toSec();
      std::ofstream myfile;
      myfile.open ("visualLog.csv");
      myfile<<"v1,v2,v3"<<std::endl; 


    /**
    for (int i = 0; i < 5; i++)
    {
       robotController.moveXYZcoord(0.05, 0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(2300));
       robotController.moveXYZcoord(-0.05, -0.0, -0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }*/
    bool armControl = false;

    while (ros::ok())
    {
        //image_processing.loop();

        if (image_processing.gets(s) &&  (ros::Time::now().toSec() - lastPubTime > 3))
        {

            ROS_INFO("getRobot Pos");
            robotController.getRobotPos(robotPos);
            ROS_INFO("visual loop ");
            if (s[0].get_Z() < 0.20)
            {
                //swith the control method to arm control
                if (!armControl)
                {
                    visualControl.setTaskMode(1);
                    armControl = true;
                }
                vpColVector v = visualControl.loop(robotPos);

                ROS_INFO("set joint Pos");
                robotController.moveXYZcoord(v[1], v[2], -v[0]);
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            else
            {
                //swith the control method to base control
                if (armControl)
                {
                    visualControl.setTaskMode(0);
                    armControl = false;
                }
                vpColVector v = visualControl.loop(robotPos);

                ROS_INFO("set joint Pos");
                myfile<<v[0]<<","<<v[1]<<","<<v[2]<<","<<std::endl; 

                robotController.setPositonJoints(v[2]);
                ROS_INFO("move base ");
                robotController.setVelocityBase(v[0], v[1]);
                driving = true;
                wait4Image = ros::Time::now().toSec();
                
            }
            lastPubTime = ros::Time::now().toSec();

            //  ros::spin();
        }
        else if (driving && ros::Time::now().toSec() - wait4Image > 0.5)
        {
            robotController.setVelocityBase(0, 0);
            driving = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
       // std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    robotController.setVelocityBase(0, 0);
    robotController.~RobotController();
    visualControl.~VisualControl();
    image_processing.~ImageNode();
    spinner.stop();
     myfile.close();

    return 0;
}