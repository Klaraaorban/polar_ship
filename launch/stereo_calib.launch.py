from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='stereo_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    remappings=[
                        ('left/image_rect', '/stereo/left/image_raw'),
                        ('right/image_rect', '/stereo/right/image_raw'),
                        ('left/camera_info', '/stereo/left/camera_info'),
                        ('right/camera_info', '/stereo/right/camera_info'),
                    ]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    remappings=[
                        ('disparity', '/disparity'),
                        ('left/camera_info', '/stereo/left/camera_info'),
                        ('right/camera_info', '/stereo/right/camera_info'),
                    ]
                )
            ],
            output='screen',
        )
    ])
