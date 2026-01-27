cmake_minimum_required(VERSION 3.0.2)
project(mvr_robot_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1. 保留原本的第三方库路径
add_subdirectory(app/thirdparty)

# 2. 查找所有依赖 (合并了原来的 C++ 依赖和新的仿真依赖)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs          # 新增：用于 Bridge 读取 IMU
  message_generation
  controller_manager
  joint_limits_interface
  gazebo_ros           # 新增
  gazebo_ros_control   # 新增
  joint_state_controller # 新增
  robot_state_publisher  # 新增
  urdf                 # 新增
)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)

# 3. 编译自定义消息 (保持不变)
add_message_files(
  FILES
  MotorData.msg
  ActionData.msg
  ObserveData.msg
  TestData.msg
  ActionData_10dof.msg
  ObserveData_10dof.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# 4. 配置 Catkin 包信息 (增加 sensor_msgs)
catkin_package(
 INCLUDE_DIRS 
 include
 app
 CATKIN_DEPENDS
 roscpp
 message_runtime
 sensor_msgs
)

# 5. 包含头文件路径 (保持不变)
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

# 6. 定义源文件 (保持不变)
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

# 7. 编译 motor 库 (保持不变)
add_library(motor ${SOURCES})
target_link_libraries(motor soem)

# add_executable(test_node 
#   src/AnkleTest.cpp 
#   src/MechanismSolver.cpp)
# target_link_libraries(test_node motor soem ${catkin_LIBRARIES})

# add_executable(cmd_node 
#   src/AnkleCmdPub.cpp)
# target_link_libraries(cmd_node motor soem ${catkin_LIBRARIES})

# 8. 编译 mvr_robot_control 可执行文件 (保持不变)
add_executable(mvr_robot_control  
  src/BotHW.cpp
  src/BotControlLoop.cpp
  src/MechanismSolver.cpp
)
# 注意：这里添加了 ${catkin_LIBRARIES} 以确保链接到 ROS 库
target_link_libraries(mvr_robot_control
  soem motor ${catkin_LIBRARIES}
)

add_executable(mvr_robot_control_10dof
  src/BotHW_10dof.cpp
  src/BotControlLoop_10dof.cpp
  src/MechanismSolver.cpp
)
target_link_libraries(mvr_robot_control_10dof
  soem motor ${catkin_LIBRARIES}
)

add_executable(hunter_robot_control  
  src/HunterHW.cpp
  src/HunterControlLoop.cpp
)
target_link_libraries(hunter_robot_control
  soem motor ${catkin_LIBRARIES}
)

# 9. 安装规则 (保持不变)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING 
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)