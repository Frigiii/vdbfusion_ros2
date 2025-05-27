import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('vdbfusion_ros2'),
        'config',
        'default.yaml'
    )
    
    return LaunchDescription([
        ComposableNodeContainer(
            name='vdbfusion_pipeline',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # # Depth to PointCloud node
                # ComposableNode(
                #     package='depth_image_proc',
                #     plugin='depth_image_proc::PointCloudXyzNode',
                #     name='depth_to_pointcloud',
                #     remappings=[
                #         ('image_rect', '/camera/depth/image_raw'),
                #         ('camera_info', '/camera/depth/camera_info'),
                #         ('points', '/input/pointcloud')  # Remap output to match your subscriber
                #     ]
                # ),  
                # Your custom node
                ComposableNode(
                    package='vdbfusion_ros2',
                    plugin='vdbfusion::vdbfusion_node',
                    name='vdbfusion_node',
                    parameters=[config_file],
                    remappings=[
                        # ('/trimesh_filter_node/lidar_boom', '/input/pointcloud'),  # Remap input to match your publisher
                        ('output/pointcloud', '/output/pointcloud')  # Remap output to match your subscriber
                    ]
                )
            ],
            output='screen',
        ),
    ])
