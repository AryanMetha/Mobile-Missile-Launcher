from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('car_path_simulation'),
        'rviz',
        'path_display.rviz'
    )

    # Start Path Publisher immediately
    path_publisher = Node(
        package='car_path_simulation',
        executable='path_publisher',
        name='path_publisher',
        output='screen'
    )

    # Delay RViz launch by 2 seconds to ensure topics are ready
    rviz = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )]
    )

    return LaunchDescription([
        path_publisher,
        rviz
    ])
