#include "image_processing.cpp"
#include "visual_control.cpp"
#include "robot_controller.cpp"
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <visp/vpColVector.h>

/**
 * This main is resposible for controlling a robot using image based visual servoing. 
 * 
 *  Author: Sondre Iveland 
 * */
int main(int argc, char **argv)
{
    ROS_INFO("Starting ROS visual control node");
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
    std::ofstream baseLog, armLog, visualFeatureLog;
    baseLog.open("LogVelocityBase.csv");
    baseLog << "v1,v2,v3" << std::endl;
    armLog.open("LogVelocityArm.csv");
    armLog << "v1,v2,v3" << std::endl;
    visualFeatureLog.open("LogVisualError.csv");
    visualFeatureLog << "x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4," << std::endl;
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

        if (image_processing.gets(s) && (ros::Time::now().toSec() - lastPubTime > 2)) //3
        {
            for (int i = 0; i < numberOfKeypoints; i++)
            {
                visualFeatureLog << s[i].get_x() - sd[i].get_x() << "," << s[i].get_y() - sd[i].get_y() << "," << s[i].get_Z() - sd[i].get_Z() << ",";
            }
            visualFeatureLog << "\n";
            ROS_INFO("getRobot Pos");
            robotController.getRobotPos(robotPos);
            ROS_INFO("visual loop ");
            if (s[0].get_Z() < 0.20)
            {
                //swith the control method to arm control
                if (!armControl)
                {
                    visualControl.setControlMode(1);
                    armControl = true;
                }
                vpColVector v = visualControl.getVelocity(robotPos);
                armLog << v[1] << "," << v[2] << "," << -v[0] << ",\n";

                ROS_INFO("set joint Pos");
                // robotController.moveXYZcoord(v[1], v[2], -v[0]);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            else
            {
                double center = 0;
                for (int i = 0; i < 4; i++)
                {
                    // std::cout << "keypoint coord " << s[i].get_y() << std::endl;
                    center += s[i].get_y();
                }
                //std::cout << "center error " << center / 4 << std::endl;

                if (center > 0.01)
                {
                    center = 0.01;
                }
                else if (center < -0.01)
                {
                    center = -0.01;
                }
                robotController.setPositonJoints(robotPos[0] - center);

                //swith the control method to base control
                if (armControl)
                {
                    visualControl.setControlMode(0);
                    armControl = false;
                }
                if (abs(center) < 0.01)
                {
                    vpColVector v = visualControl.getVelocity(robotPos);

                    ROS_INFO("set joint Pos");
                    baseLog << v[0] << "," << v[1] << "," << v[2] << ",\n";

                    //robotController.setPositonJoints(v[2]);
                    ROS_INFO("move base ");
                    //robotController.setVelocityBase(v[0], v[1]);

                    driving = true;
                }
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

    baseLog << std::flush;
    armLog << std::flush;
    visualFeatureLog << std::flush;
    baseLog.close();
    armLog.close();
    visualFeatureLog.close();
    robotController.setVelocityBase(0, 0);
    robotController.~RobotController();
    visualControl.~VisualControl();
    image_processing.~ImageNode();
    spinner.stop();

    return 0;
}