
int main(int argc, char **argv)
{
    ROS_INFO("Starting ROS main control node");
    ros::init(argc, argv, "main_control");
    vision::ImageNode visionNode;

    return 0;
}