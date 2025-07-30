from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    urdf_xacro_file = os.path.join(
        FindPackageShare('rehab_arm_robot').find('rehab_arm_robot'),
        'urdf',
        'rehab_arm_robot.urdf.xacro'
    )

    controller_config = os.path.join(
        FindPackageShare('rehab_arm_robot_moveit_config').find('rehab_arm_robot_moveit_config'),
        'config',
        'ros2_controllers.yaml'
    )
    robot_description = {'robot_description': Command(['xacro ', urdf_xacro_file])}

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_config],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['odrive_arm_controller'],
            output='screen'
        )
    ])
