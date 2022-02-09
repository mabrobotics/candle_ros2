cmake_minimum_required(VERSION 3.5)
project(ros2_mab_md80)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(ros2_mab_md80
  "msg/ImpedanceCommand.msg"
  "msg/MotionCommand.msg"
  "msg/Pid.msg"
  "msg/VelocityPidCommand.msg"
  "msg/PositionPidCommand.msg"
  "srv/AddMd80s.srv"
  "srv/GenericMd80Msg.srv"
  "srv/SetModeMd80s.srv"
  "srv/SetLimitsMd80.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)
set(CMAKE_INSTALL_RPATH $ORIGIN)  # sets proper runtime paths for exec to link .so properly

add_executable(md80_node src/md80_node.cpp)
target_link_libraries(md80_node "${CMAKE_SOURCE_DIR}/lib/libcandle.so")
target_include_directories(md80_node PUBLIC ${CMAKE_SOURCE_DIR}/include)

rosidl_target_interfaces(md80_node ${PROJECT_NAME} "rosidl_typesupport_cpp")  # Links msgs from this very package
ament_target_dependencies(md80_node rclcpp sensor_msgs)

install(TARGETS md80_node 
  DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
install(FILES lib/libcandle.so 
  DESTINATION lib/${PROJECT_NAME}/
  )
ament_package()
