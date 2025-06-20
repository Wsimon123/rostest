cmake_minimum_required(VERSION 3.5)
project(my_srv_msg)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp geometry_msgs)         # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp geometry_msgs)     # CHANGE

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp geometry_msgs)         # CHANGE

add_executable(service src/add_two_ints_server.cpp)
ament_target_dependencies(service rclcpp geometry_msgs)     # CHANGE

install(TARGETS
  talker
  listener
  client
  service
  DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(listener
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
