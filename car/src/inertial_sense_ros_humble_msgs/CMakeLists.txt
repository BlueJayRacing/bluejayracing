cmake_minimum_required(VERSION 3.8)
project(inertial_sense_ros_humble_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(rosidl_typesupport_cpp REQUIRED)
# find_package(rosidl_typesupport_introspection_cpp REQUIRED)
# find_package(rosidl_typesupport_c REQUIRED)
# find_package(rosidl_typesupport_introspection_c REQUIRED)
# find_package(rosidl_generator_cpp REQUIRED)
# find_package(rosidl_generator_c REQUIRED)


set(msg_files
  "msg/GTime.msg"
  "msg/SatInfo.msg"
  "msg/GPS.msg"
  "msg/GPSInfo.msg"
  "msg/PreIntIMU.msg"
  "msg/RTKInfo.msg"
  "msg/RTKRel.msg"
  "msg/GlonassEphemeris.msg"
  "msg/GNSSEphemeris.msg"
  "msg/GNSSObservation.msg"
  "msg/GNSSObsVec.msg"
  "msg/INL2States.msg"
  "msg/DIDINS2.msg"
  "msg/DIDINS1.msg"
  "msg/DIDINS4.msg"
)

set(srv_files
  "srv/Firmwareupdate.srv"
  "srv/Refllaupdate.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target( cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp ) 

ament_export_dependencies(rosidl_default_runtime)

ament_package()