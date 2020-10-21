#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

/**
 * This class is responsible for the control of the robot. 
 * It contains all the comunication with the robot for both the arm and the base 
 *  Author: Sondre Iveland 
 * */
namespace vision
{

    class RobotController
    {
    private:
        static constexpr const char *PLANNING_GROUP = "arm";
        moveit::planning_interface::MoveGroupInterface *move_group;
        ros::NodeHandle nh;
        ros::Publisher publisher_state;
        const robot_state::JointModelGroup *joint_model_group;
        geometry_msgs::PoseStamped newPose;
        bool jointsMoved;

    public:
        RobotController(ros::NodeHandle *nodehandle) : nh(*nodehandle) //ros::NodeHandle nh
        {
            // async spinner due to the moveit library

            // Make one separate thread for the images. this have a separate queue
            //spinner = new  ros::AsyncSpinner(2);
            //  spinner->start();
            // std::thread testthread(&ImageNode::loop, this);
            // testthread.join();
            // kill the threads.
            // ros::waitForShutdown();

            //spinner.stop();

            //initiate the robot arm
            this->publisher_state = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP); //PLANNING_GROUP
            std::this_thread::sleep_for(std::chrono::seconds(3));
            joint_model_group =
                move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP); //PLANNING_GROUP
            ROS_INFO("Robot controller running ");
            setStartPos();
            std::this_thread::sleep_for(std::chrono::seconds(3));
            newPose = move_group->getCurrentPose();
            jointsMoved = false;
            //moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

            //current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
            //robot.set_eJe(joint_group_positions[0]);

            // make the robot go to start position.

            //current_state = move_group.getCurrentState();
            //std::vector<double> current_joint_group_pos;
            //current_state->copyJointGroupPositions(joint_model_group, current_joint_group_pos);
            //robot.set_eJe(current_joint_group_pos[0]);
        }
        ~RobotController()
        {
            delete move_group;
            //spinner->stop();
        }

        void getRobotPos(std::vector<double> &joint_group_pos)
        {
            joint_group_pos.clear();
            moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

            current_state->copyJointGroupPositions(joint_model_group, joint_group_pos);
        }
        void setStartPos()
        {

            std::vector<double> joint_group_positions;
            // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
            joint_group_positions.push_back(0.0);    // radians 0
            joint_group_positions.push_back(-1.300); // radians -0.1
            joint_group_positions.push_back(0.834);  // radians -0.05
            joint_group_positions.push_back(0.460);  // radians 0

            move_group->setJointValueTarget(joint_group_positions);

            //success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            move_group->move();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        void setPositonJoints(float q1, float q2, float q3, float q4)
        {
            std::vector<double> joint_group_positions;
            joint_group_positions.push_back(q1); // radians
            joint_group_positions.push_back(q2); // radians
            joint_group_positions.push_back(q3); // radians
            joint_group_positions.push_back(q4); // radians

            move_group->setJointValueTarget(joint_group_positions);

            //success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group->asyncMove();
            jointsMoved = true;
            //  move_group->move();
        }
        void setPositonJoints(float q1)
        {

            std::vector<double> joint_group_positions;
            joint_group_positions.push_back(q1);     // radians
            joint_group_positions.push_back(-1.300); // radians -0.1
            joint_group_positions.push_back(0.834);  // radians -0.05
            joint_group_positions.push_back(0.460);  // radians 0

            move_group->setJointValueTarget(joint_group_positions);
            ROS_INFO("robot control: setting joint q1");

            //success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group->asyncMove();
            jointsMoved = true;
            //  (*move_group).move();
        }
        void moveXYZcoord(double x, double y, double z)
        {
            if (jointsMoved)
            {
                newPose = move_group->getCurrentPose();
                jointsMoved = false;
            }
            // geometry_msgs::PoseStamped curPose = move_group->getCurrentPose();
            // std::cout << "x:" << curPose.pose.position.x << std::endl;
            // std::cout << curPose.pose.position.y << std::endl;
            // std::cout << curPose.pose.position.z << std::endl;

            //c::Point newPoint;
            // geometry_msgs::Quaternion newQuert;
            //newPoint.x;
            //newQuert.x;
            newPose.pose.position.x = newPose.pose.position.x + x;
            newPose.pose.position.y = newPose.pose.position.y + y;
            newPose.pose.position.z = newPose.pose.position.z + z;
            //(*move_group).setPoseTarget(newPose);
            //success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            (*move_group).setPositionTarget(newPose.pose.position.x, newPose.pose.position.y, newPose.pose.position.z);
            move_group->asyncMove();
            ROS_INFO("robot control: move to x= %.4f y= %.4f , z= %.4f", newPose.pose.position.x, newPose.pose.position.y, newPose.pose.position.z);
            //(*move_group).move();
        }

        // set the speed of the robot in x and W direction where ohmega is around z axis.
        void setVelocityBase(float x = 0, float ohmega = 0)
        {

            float MaxVelocity = 0.3;
            if (x > MaxVelocity)
            {
                x = MaxVelocity;
            }
            if (x < -MaxVelocity)
            {
                x = -MaxVelocity;
            }
            if (ohmega > MaxVelocity)
            {
                ohmega = MaxVelocity;
            }
            if (ohmega < -MaxVelocity)
            {
                ohmega = -MaxVelocity;
            }
            geometry_msgs::Vector3 linMove, rotMove;
            linMove.x = x;
            rotMove.z = ohmega;

            geometry_msgs::Twist movemsg;
            movemsg.linear = linMove;
            movemsg.angular = rotMove;

            ROS_INFO("robot control: setting wheel speed x= %.4f ohmega= %.4f", x, ohmega);
            // publish to the topic given in the initiation function.

            this->publisher_state.publish(movemsg);
        }
    };
} // namespace vision
