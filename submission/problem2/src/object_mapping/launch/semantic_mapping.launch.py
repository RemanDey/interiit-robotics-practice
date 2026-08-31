from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name',
            default_value='yolov8n-seg.pt',
            description='YOLO model name or path'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/oakd/rgb/preview/image_raw',
            description='RGB image topic'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/oakd/rgb/preview/depth',
            description='Aligned depth image topic'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/oakd/rgb/preview/camera_info',
            description='Camera info topic'
        ),
        DeclareLaunchArgument(
            'json_output_path',
            default_value='semantic_map.json',
            description='Output JSON file path'
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='map',
            description='Target frame for TF transformation (map or odom)'
        ),
        DeclareLaunchArgument(
            'dedup_threshold',
            default_value='0.5',
            description='Euclidean distance threshold for deduplication (meters)'
        ),
        DeclareLaunchArgument(
            'max_missed',
            default_value='30',
            description='Max frames before removing stale tracker'
        ),
        DeclareLaunchArgument(
            'benchmark_enabled',
            default_value='true',
            description='Enable benchmark logging'
        ),

        Node(
            package='object_mapping',
            executable='semantic_mapping',
            name='perception_segmentor_node',
            output='screen',
            parameters=[{
                'model_name': LaunchConfiguration('model_name'),
                'image_topic': LaunchConfiguration('image_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'json_output_path': LaunchConfiguration('json_output_path'),
                'target_frame': LaunchConfiguration('target_frame'),
                'dedup_threshold': LaunchConfiguration('dedup_threshold'),
                'max_missed': LaunchConfiguration('max_missed'),
                'benchmark_enabled': LaunchConfiguration('benchmark_enabled'),
            }]
        )
    ])