from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wave_filters',
            executable='data_generator_node',
            name='data_generator_node',
            output='screen',
            parameters=[{
                'frequency': 0.5,
                'amplitude': 2.0,
                'noise_std_dev': 0.3
            }]
        ),
        Node(
            package='wave_filters',
            executable='median_filter_node',
            name='median_filter_node',
            output='screen',
            parameters=[{'window_size': 5}]
        ),
        Node(
            package='wave_filters',
            executable='lowpass_filter_node',
            name='lowpass_filter_node',
            output='screen',
            parameters=[{'alpha': 0.2}]
        ),
    ])
