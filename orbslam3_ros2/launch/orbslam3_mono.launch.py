from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='orbslam3_mono',
            description='ROS node name (set a unique one if launching multiple instances).',
        ),
        DeclareLaunchArgument(
            'vocabulary_file',
            default_value='',
            description='Path to ORBvoc.txt',
        ),
        DeclareLaunchArgument(
            'settings_file',
            default_value='',
            description='Path to ORB-SLAM3 YAML settings',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Monocular image topic',
        ),
        DeclareLaunchArgument(
            'use_viewer',
            default_value='true',
            description='Enable Pangolin viewer',
        ),
        DeclareLaunchArgument(
            'publish_pose',
            default_value='false',
            description='Publish camera_pose and camera_path topics',
        ),
        Node(
            package='orbslam3_ros2',
            executable='orbslam3_mono_node',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[{
                'vocabulary_file': LaunchConfiguration('vocabulary_file'),
                'settings_file': LaunchConfiguration('settings_file'),
                'image_topic': LaunchConfiguration('image_topic'),
                'use_viewer': LaunchConfiguration('use_viewer'),
                'publish_pose': LaunchConfiguration('publish_pose'),
            }],
        ),
    ])
