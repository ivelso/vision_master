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
            task.setServo(vpServo::EYEINHAND_CAMERA);
            identityMatcVe = task.get_cVe();
            identityMateJe = task.get_eJe();
            task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
            task.setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE);
            lambda.initStandard(2.25, 0.005, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
            task.setLambda(0.001);                //0.02 worked good for the Y coord                 //0.04 worked for all coord
        }
        ~VisualControl()
        {
            task.kill();
        }

        /**
     * set the features for the control given in a array of 4 features. 
     * This is only performed once because the address is used so the features will be updated. 
     * 
     * */
        void setFeatures(vpFeaturePoint *s, vpFeaturePoint *sd)
        {
            sptr = s;
            //add the features to the control
            task.addFeature(s[0], sd[0], vpFeaturePoint::selectY()); //, vpFeaturePoint::selectX()  vpFeaturePoint::selectY()
            task.addFeature(s[1], sd[1], vpFeaturePoint::selectY());
            //task.addFeature(s_Z, s_Zd);
            task.addFeature(s[2], sd[2], vpFeaturePoint::selectY());
            task.addFeature(s[3], sd[3], vpFeaturePoint::selectY());
        }
        void setTaskMode(int mode)
        {
            if (mode == 1)
            {
                task.setServo(vpServo::EYEINHAND_CAMERA);
                task.set_cVe(identityMatcVe);
                task.set_eJe(identityMateJe);
                //lambda.initStandard(0.5, 0.05, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
                task.setLambda(0.05);
                cameraVelocity = true;
            }
            if (mode == 0) // control of base
            {
                task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
                lambda.initStandard(0.7, 0.1, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
                task.setLambda(0.04);              //0.04
                cameraVelocity = false;
            }
            //task.setInteractionMatrixType(vpServo::CURRENT);
        }
        /**
        * The loop of the control. 
        **/
        vpColVector loop(std::vector<double> &joint_group_pos)
        {

            if (!cameraVelocity)
            {
                robot.set_eJe(joint_group_pos[0]);
                cVe = robot.get_cVe();
                task.set_cVe(cVe);
                eJe = robot.get_eJe();
                task.set_eJe(eJe);
            }

            //std::cout << task.get_eJe() << std::endl;
            //std::cout << "cVe " << std::endl;
            //std::cout << task.get_cVe() << std::endl;

            vpColVector v = task.computeControlLaw();
            // v[0] = v[0]*1.5 ; // because the x direction is found to be slow.
            //v[2] = v[2]*1.8 ;
            //  v[2] = -v[2];
            task.print();
            double vmax = 0.30;
            // make the velocity not go above the max and change the range for the velocity if it is above the range.
            for (int i = 0; i < v.size(); i++)
            {
                if (abs(v[i]) > vmax)
                {
                    for (int nmb = 0; nmb < v.size(); nmb++)
                    {
                        v[nmb] = (vmax * v[nmb]) / abs(v[i]);
                    }
                }
                std::cout << "velocity " << i << ": " << v[i] << std::endl;
            }

            /**
            if (task.getError().sumSquare() < 0.0001  && !errorSmallUseArm)
            {
                errorSmallUseArm = true;
                setTaskMode(1); 
            }
            else if (errorSmallUseArm == true)
            {
                errorSmallUseArm = false;
                setTaskMode(0); 
            }
            */
            //ploter(v, task.getError());
            //if ((ros::Time::now().toSec() - lastPubTime) > 1)
            //{
            //std::cout << "time " << ros::Time::now().toSec() - lastPubTime << std::endl;
            //lastPubTime = ros::Time::now().toSec();

            // }
            return v;
        }
        //task.kill();

        /***
         * return the velocity to control the arm with only gettting the target in the centre. 
         * simple p control where the error is the only thing used. 
         * */
        void getVelocityToGetPointsInCentre(std::vector<double> &velocity)
        {

            double centerX = 0;
            double centerY = 0;
            double centerZ = 0.005;
            double vmax = 0.005;
            for (int i = 0; i < 4; i++)
            {

                centerY += sptr[i].get_y();
                centerX += sptr[i].get_x();
            }
            centerY = centerY * 0.01;
            centerX = centerX * 0.01;
            velocity.clear();

            velocity.push_back(centerX);
            velocity.push_back(centerY);
            velocity.push_back(centerZ);
            // scale the output so max is the highest velocity
            for (int i = 0; i < velocity.size(); i++)
            {
                if (abs(velocity[i]) > vmax)
                {
                    for (int nmb = 0; nmb < velocity.size(); nmb++)
                    {
                        velocity[nmb] = (vmax * velocity[nmb]) / abs(velocity[i]);
                    }
                }
                std::cout << "velocity " << i << ": " << velocity[i] << std::endl;
            }
        }

        bool controlArm()
        {
            return errorSmallUseArm;
        }

    private:
        vpAdaptiveGain lambda;
        vpServo task;
        vpVelocityTwistMatrix cVe;
        vpMatrix eJe;
        vpTurtlebotPan robot;
        bool firstRound = true;
        bool cameraVelocity = false;
        bool errorSmallUseArm = false;
        vpFeaturePoint *sptr;

        vpVelocityTwistMatrix identityMatcVe;
        vpMatrix identityMateJe;
    }; // namespace vision
} // namespace vision
