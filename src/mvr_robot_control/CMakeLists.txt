cmake_minimum_required(VERSION 3.0.2)
project(mvr_robot_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(app/thirdparty)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  controller_manager
  joint_limits_interface
)

find_package(Eigen3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED)

add_message_files(
  FILES
  MotorData.msg
  ActionData.msg
  ObserveData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS 
 include
 app
 CATKIN_DEPENDS
 roscpp
 message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_NAME}
  app
  app/thirdparty/soem/soem
  app/thirdparty/soem/osal
  app/thirdparty/soem/osal/linux
  app/thirdparty/soem/oshw/linux
  app/thirdparty/cpp-readline/src
)

set(SOURCES 
  app/motor_control.c
  app/motor_control.h
  app/math_ops.c
  app/math_ops.h
  app/transmit.cpp
  app/transmit.h
  app/config.h
  app/queue.h
  app/command.cpp
  app/command.h)

add_library(motor ${SOURCES})
target_link_libraries(motor soem)

add_executable(mvr_robot_control  
  src/BotHW.cpp
  src/BotControlLoop.cpp
)
target_link_libraries(mvr_robot_control
  soem motor ${catkin_LIBRARIES}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

