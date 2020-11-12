# vision_master
Code for the advanced project ecte940 while pursuing a Master of Engineering.

The code implements image based visual control for Turtlebot3 waffle with OpenMANIPULATOR-X. 
The camera is placed on the manipulator and the current version works with using blob detection. 
The camera used is a intel d435i realsense camera. 
This must publish the RGB image to the topic "camera/color/image_raw/" and the depth image to camera/aligned_depth_to_color/image_raw/".
The images need tobe recived with the same timestamp in order to be valied for the control. 

To run the program:
Separate computer 
$roscore
On the robot computer
$roslaunch turtlebot3_bringup turtlebot3_robot.launch 
Separat computer 
$roslaunch vision vision.launch 

The library OpenCV, VISP and VISp_ROS must be installed on the separate computer.  
The communication to the arm is thru MoveIT library
