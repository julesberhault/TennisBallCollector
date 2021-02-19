from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share1 = FindPackageShare('robot_tennis').find('robot_tennis')
    pkg_share2 = FindPackageShare('gazebo_ros').find('gazebo_ros')
    #pkg_share3 = FindPackageShare('robot1_description').find('robot1_description')
    model_file = os.path.join(pkg_share1, 'urdf', "robot_tennis.urdf")
    #path = os.path.join(pkg_share2, 'launch', "gazebo.launch.py")
    #rviz_config_file = os.path.join(pkg_share3, 'config', "gazebo.rviz")
    
    robot_state_publisher_node=Node(
        package="robot_state_publisher", node_executable="robot_state_publisher",
        arguments=[model_file]
        )
    rqt_robot_steering_node=Node(package="rqt_robot_steering", node_executable="rqt_robot_steering")
    #rviz_node=Node(package="rviz2", node_executable="rviz2",
    #    arguments=["-d", rviz_config_file])

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py', arguments=['-entity', 'robot_tennis', '-topic', '/robot_description', '-x', '5', '-y', '5', '-z', '0'])

    robot_controller_node=Node(
        package="robot_tennis_controller", node_executable="controller")

    navigation_node = Node(
        package="robot_tennis_controller", node_executable="navigation"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        #rviz_node,
        spawn_entity,
        rqt_robot_steering_node,
        robot_controller_node
    ])