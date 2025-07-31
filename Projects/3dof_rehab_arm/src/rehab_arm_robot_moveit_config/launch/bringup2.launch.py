import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

def generate_launch_description():

    # Tham số: file vị trí ban đầu của khớp
    initial_positions_file_arg = DeclareLaunchArgument(
        "initial_positions_file",
        default_value=os.path.join(
            get_package_share_directory("rehab_arm_robot_moveit_config"),
            "config",
            "initial_positions.yaml"
        ),
        description="Path to the initial joint positions YAML file"
    )

    # Build moveit_config từ MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("rehab_arm_robot", package_name="rehab_arm_robot_moveit_config")
        .robot_description(
            file_path="config/rehab_arm_robot.urdf.xacro",
            mappings={"initial_positions_file": LaunchConfiguration("initial_positions_file")}
        )
        .robot_description_semantic(file_path="config/rehab_arm_robot.srdf")
        .trajectory_execution(file_path="config/ros2_controllers.yaml")
        # .moveit_controller(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    #--- giai nen file moveit_controllers.yaml
    # Đọc file moveit_controllers.yaml và lấy dict move_group
    moveit_controller_yaml_file = os.path.join(
        get_package_share_directory("rehab_arm_robot_moveit_config"),
        "config",
        "moveit_controllers.yaml"
    )

    with open(moveit_controller_yaml_file, 'r') as f:
        # moveit_controller_yaml = yaml.safe_load(f)
        full_yaml = yaml.safe_load(f)
        moveit_controller_yaml = full_yaml.get("move_group", {})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_controller_yaml  # ✅ truyền dict chứ không truyền string path
        ]
    )
    #---

    rviz_config = os.path.join(
        get_package_share_directory("rehab_arm_robot_moveit_config"),
        "config",
        "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics
        ]
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description]
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("rehab_arm_robot_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
        arguments=["--ros-args", "--log-level", "debug"]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    rehab_arm_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rehab_arm_moveit_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        initial_positions_file_arg,
        move_group_node,
        rviz_node,
        static_tf_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        rehab_arm_moveit_controller_spawner
    ])
