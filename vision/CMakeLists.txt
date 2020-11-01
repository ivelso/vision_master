cmake_minimum_required(VERSION 2.8.3)
project(vision)


find_package(catkin REQUIRED COMPONENTS 
  roscpp rviz
  message_runtime std_msgs sensor_msgs geometry_msgs
  image_transport
  cv_bridge 
  visp_bridge
  visp_ros
  message_filters
  moveit_ros_planning_interface
  moveit_msgs
  moveit_commander moveit_core
  moveit_ros_move_group

 
)
find_package(OpenCV REQUIRED NO_MODULE)
find_package(VISP)



include_directories(SYSTEM
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )



catkin_package(INCLUDE_DIRS include)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Macro builds each node, assuming they all have the same requirements
macro(build_node name)
    add_executable(${name} src/${name}.cpp)
    target_include_directories(${name} PUBLIC ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)
    target_link_libraries(${name} PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
    target_compile_options(${name} PUBLIC ${PRIVATE_BUILD_FLAGS})
endmacro()





build_node(main_control)




