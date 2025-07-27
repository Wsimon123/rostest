from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # 获取包共享目录路径
    pkg_share = FindPackageShare('my_simulate').find('my_simulate')
    
    # 构建XACRO文件路径（立即求值）
    xacro_path = os.path.join(pkg_share, 'urdf', 'first_robot.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    
    # 验证文件存在
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"XACRO file not found at: {xacro_path}")
    
    # 使用xacro命令行工具将XACRO转换为URDF字符串
    xacro_cmd = f"xacro {xacro_path}"
    robot_description = Command(['xacro ', xacro_path])

    return LaunchDescription([
        # 基础坐标系发布
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            output='screen'
        ),
        
        # 机器人状态发布（使用xacro解析结果）
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
            name='joint_state_publisher',
            output='screen'
        ),
        
        # RViz2配置
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': False}],
            output='screen'
        )
    ])