import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パッケージのパスを取得
    package_dir = get_package_share_directory('whill_visualization_example')

    # ノードの設定
    polygon_publisher = Node(
        package='whill_visualization_example',
        executable='polygon_array_publisher',
        name='polygon_publisher',
        parameters=[{
            'topic_name': '/polygon_array',
            'frame_id': 'map',
            'publish_rate': 10.0
        }],
        output='screen'
    )

    # RVizの設定
    rviz_config = os.path.join(package_dir, 'config', 'polygon_array_demo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        polygon_publisher,
        rviz_node
    ])
