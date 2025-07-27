from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = '/home/wxm123/Desktop/rostest/install/my_simulate/share/my_simulate/urdf/first_robot.urdf'
    
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # 基础坐标系发布
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            output='screen'
        ),
        
        # 机器人状态发布
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        
        # 关节状态发布
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        
        # RViz2配置
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(os.path.dirname(urdf_path), '../rviz/display.rviz')],
            parameters=[{'use_sim_time': False}],
            output='screen'
        )
    ])