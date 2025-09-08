cmake_minimum_required(VERSION 3.5)
project(candle_ros2)

set(md_node_name md_node)
set(pds_node_name pds_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/ImpedanceCmd.msg"
  "msg/MotionCmd.msg"
  "msg/Pid.msg"
  "msg/VelocityPidCmd.msg"
  "msg/PositionPidCmd.msg"
  "msg/BrakeResistorData.msg"
  "msg/ControlModuleData.msg"
  "msg/IsolatedConverterData.msg"
  "msg/PowerStageData.msg"  
)

set(srv_files
  "srv/AddDevices.srv"
  "srv/Generic.srv"
  "srv/GenericPds.srv"
  "srv/SetLimits.srv"
  "srv/SetMode.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

file(GLOB_RECURSE sources "src/pds_modules/*.cpp")

add_executable(${md_node_name} src/md_node.cpp)
add_executable(${pds_node_name} src/pds_node.cpp ${sources})

set(CMAKE_SKIP_INSTALL_RULES ON)
add_subdirectory(CANdle-SDK EXCLUDE_FROM_ALL)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${md_node_name} ${cpp_typesupport_target} candle rclcpp_components::component)
target_include_directories(${md_node_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${md_node_name} PUBLIC c_std_99 cxx_std_20)

ament_target_dependencies(
  ${md_node_name}
  "rclcpp"
  "sensor_msgs"
)

target_link_libraries(${pds_node_name} ${cpp_typesupport_target} candle rclcpp_components::component)
target_include_directories(${pds_node_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${pds_node_name} PUBLIC c_std_99 cxx_std_20)

ament_target_dependencies(
  ${pds_node_name}
  "rclcpp"
  "sensor_msgs"
)

set(CMAKE_SKIP_INSTALL_RULES OFF)
install(TARGETS ${md_node_name} ${pds_node_name}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
