cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelSpeeds.msg"
  "msg/PIDGains.msg"
  "msg/YaguraMechanism.msg"
  "msg/RingMechanism.msg"
  "msg/WheelMessage.msg"
  "msg/HandMessage.msg"
)

ament_package()
