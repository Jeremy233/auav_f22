import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_ros_ign_gazebo_demos = get_package_share_directory('ros_ign_gazebo_demos')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': '-v 4 -r trial_1.world --gui-config /home/docker/.ignition/gazebo/6/gui.config'
        }.items(),
    )

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msgs/PointCloud2@ignition.msgs.PointCloudPacked',
            '/rover/cmd_vel@geometry_msgs/Twist@ignition.msgs.Twist',
            '/rover/odom@nav_msgs/Odometry@ignition.msgs.Odometry'
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ros_ign_gazebo_demos, 'rviz', 'rgbd_camera_bridge.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        ign_gazebo,
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        bridge,
        rviz,
    ])
