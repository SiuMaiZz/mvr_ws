cmake_minimum_required(VERSION 3.0.2)
project(mvr_bot_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  gazebo_ros_control
  joint_state_controller
  robot_state_publisher
  roscpp
  rospy
  sensor_msgs
  std_msgs
  urdf
  xacro
  message_generation  # [修改1] 添加这行，用于生成消息代码
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## [修改2] 声明需要编译的消息文件
add_message_files(
  FILES
  ActionData.msg
  ObserveData.msg
  MotorData.msg
)

## [修改3] 生成消息，依赖 std_msgs (因为你的消息里用了 Header)
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mvr_bot_gazebo
  ## [修改4] 添加 message_runtime，这样 Python 运行时才能引用消息
  CATKIN_DEPENDS controller_manager gazebo_ros gazebo_ros_control joint_state_controller robot_state_publisher roscpp rospy sensor_msgs std_msgs urdf xacro message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## (以下部分暂时保持默认注释状态即可，除非你有 C++ 节点需要编译)