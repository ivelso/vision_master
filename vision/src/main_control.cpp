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
    ros::Time::useSystemTime();
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
    //ros::Rate loop_rate(30);
    while (ros::ok())
    {
        //image_processing.loop();

        if (image_processing.gets(s) && (ros::Time::now().toSec() - lastPubTime > 3))
        {
            for (int i = 0; i < numberOfKeypoints; i++)
            {
                visualFeatureLog << (s[i].get_x() - sd[i].get_x()) << "," << (s[i].get_y() - sd[i].get_y()) << "," << (s[i].get_Z() - sd[i].get_Z()) << ",";
            }
            visualFeatureLog << "\n";
            ROS_INFO("getRobot Pos");
            robotController.getRobotPos(robotPos);
            ROS_INFO("visual loop ");
            if (s[0].get_Z() < 0.20 )
            {

                //swith the control method to arm control
                if (!armControl)
                {
                    visualControl.setTaskMode(1);
                    armControl = true;
                }

                vpColVector velocity = visualControl.loop(robotPos);
                armLog << velocity[2] << "," << velocity[1] << "," << -velocity[0] << ",\n";
                robotController.moveXYZcoord(velocity[2], -velocity[1], -velocity[0]);
                ROS_INFO("set joint Pos");

                /**
                 // un comment this to use the own algortihm for the arm. The arm will then move 0.05m to ward the target and try to get the features in the centre of the image. 
                 
                std::vector<double> velocity;
                visualControl.getVelocityToGetPointsInCentre(velocity);
                armLog << velocity[2] << "," << velocity[1] << "," << -velocity[0] << ",\n";
                std::cout << -velocity[0] << " center x " << -velocity[1] << std::endl;
                robotController.moveXYZcoord(velocity[2], -velocity[1], -velocity[0]);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                */
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
                baseLog << v[0] << "," << v[1] << "," << v[2] << ",\n";

                robotController.setVelocityJoint(v[2]);
                ROS_INFO("move base ");
                robotController.setVelocityBase(v[0], v[1]);
                driving = true;
                ros::spinOnce();
                //ros::Time::sleepUntil(ros::Time(0, 500));
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                robotController.setVelocityBase(0, 0);
                ros::spinOnce();
                //driving = false;

                wait4Image = ros::Time::now().toSec();
            }
            lastPubTime = ros::Time::now().toSec();

            //  ros::spin();
        }
        else if (driving && ros::Time::now().toSec() - wait4Image > 0.5)
        {
            robotController.setVelocityBase(0, 0);
            driving = false;
            image_processing.gets(s);
            // std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        //ros::spinOnce();
        //loop_rate.sleep();
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        baseLog << std::flush;
        armLog << std::flush;
        visualFeatureLog << std::flush;
        // ros::Time::;
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