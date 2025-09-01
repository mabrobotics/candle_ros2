cmake_minimum_required(VERSION 3.5)
project(candle_ros)
set(node_name md_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/ImpedanceCmd.msg"
  "msg/MotionCmd.msg"
  "msg/Pid.msg"
  "msg/VelocityPidCmd.msg"
  "msg/PositionPidCmd.msg"
)

set(srv_files
  "srv/AddMds.srv"
  "srv/Generic.srv"
  "srv/SetLimits.srv"
  "srv/SetMode.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(${node_name} src/md_node.cpp)

set(CMAKE_SKIP_INSTALL_RULES ON)
add_subdirectory(CANdle-SDK EXCLUDE_FROM_ALL)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${node_name} ${cpp_typesupport_target} candle)

target_include_directories(${node_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${node_name} PUBLIC c_std_99 cxx_std_20)

ament_target_dependencies(
  ${node_name}
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

set(CMAKE_SKIP_INSTALL_RULES OFF)
install(TARGETS ${node_name}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
