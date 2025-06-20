# rostest
## cerate package
ros2 pkg create <my_package> --build-type ament_cmake --dependencies rclcpp 

## build select package
colcon build --packages-select my_package
