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
/**
 * This class is responsible for the control given the featurepoints the desired velocites for the robot is calculated. 
 * the class just calculates the velocities and does not perform the comunication. 
 * 
 *  Author: Sondre Iveland 
 * */
namespace vision
{

    class VisualControl
    {
    public:
        VisualControl()
        {
            task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
            task.setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE);
            lambda.initStandard(0.7, 0.1, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
            task.setLambda(lambda);

            //std::thread testthread(&VisualControl::loop, this);
            // testthread.join();
        }
        ~VisualControl()
        {
            //task.kill();
        }

    private:
        vpAdaptiveGain lambda;
        vpServo task;
        vpVelocityTwistMatrix cVe;
        vpMatrix eJe;
        vpTurtlebotPan robot;
        bool firstRound = true;

    public:
    /**
     * set the features for the control given in a array of 4 features. 
     * This is only performed once because the address is used so the features will be updated. 
     * 
     * */
        void setFeatures(vpFeaturePoint *s, vpFeaturePoint *sd)
        {

            //add the features to the control
            task.addFeature(s[0], sd[0]); //, vpFeaturePoint::selectX()  vpFeaturePoint::selectY()
            task.addFeature(s[1], sd[1]);
            //task.addFeature(s_Z, s_Zd);
            task.addFeature(s[2], sd[2]);
            task.addFeature(s[3], sd[3]);
        }
        /**
        * The loop of the control. 
        **/
        vpColVector loop(std::vector<double> &joint_group_pos)
        {

            //task.setInteractionMatrixType(vpServo::CURRENT);

            //flag to make the robot stop, because the control is discrete
            //bool robotDiving = true;

            // task.setLambda(0.2);

            // vpMatrix k()

            //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            // finished moving

            // the loop as loong as ros is not in shut down.
            robot.set_eJe(joint_group_pos[0]);
            cVe = robot.get_cVe();
            task.set_cVe(cVe);
            eJe = robot.get_eJe();
            task.set_eJe(eJe);
            //std::cout << eJe << std::endl;
            //std::cout << "cVe " << std::endl;
            //std::cout << cVe << std::endl;
            
            vpColVector v = task.computeControlLaw();
            task.print();
            //for (int i = 0; i < v.size(); i++)
            //  std::cout << "velocity " << i << ": " << v[i] << std::endl;
            //ploter(v, task.getError());
            //if ((ros::Time::now().toSec() - lastPubTime) > 1)
            //{
            //std::cout << "time " << ros::Time::now().toSec() - lastPubTime << std::endl;
            //lastPubTime = ros::Time::now().toSec();

            float velocity = v[2];
            std::cout << "velocity joint " << velocity << std::endl;
            float limit = 0.01;

            if (velocity > limit)
            {
                velocity = limit;
            }
            if (velocity < -limit)
            {
                velocity = -limit;
            }
            if (velocity > 0.0005 || velocity < -0.0005)
            // the joint q is defined opposite direction in the turtlebot pan file.
            {
                v[2] = joint_group_pos[0] - velocity;
                //std::cout << "Joint angle" << current_joint_group_pos[0] - velocity << std::endl;
                //  move_group.setJointValueTarget(joint_group_positions); // set joint position for the arm
                //move_group.move();                                     // make the arm move
            }
            else
            {
                v[2] = 0;
            }
            // }
            return v;

        }
        //task.kill();

    }; 
} // namespace vision
