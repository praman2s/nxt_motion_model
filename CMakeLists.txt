cmake_minimum_required(VERSION 2.8.3)
project(nxt_motion_model)

find_package(catkin REQUIRED COMPONENTS cv_bridge dynamic_reconfigure image_transport roscpp rospy sensor_msgs std_msgs std_srvs)
find_package(OpenCV REQUIRED)

catkin_package(

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imageconverter src/nxt_motion_model/imageconverter.cpp)
target_link_libraries(nxt_motion_model_node
   ${catkin_LIBRARIES}
)
target_link_libraries(imageconverter
   ${catkin_LIBRARIES}
)


