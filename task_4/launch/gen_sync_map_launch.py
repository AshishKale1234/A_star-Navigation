from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    turtlebot4_navigation_dir = get_package_share_directory('turtlebot4_navigation')
    turtlebot4_viz_dir = get_package_share_directory('turtlebot4_viz')

    # Define launch arguments
    namespace = LaunchConfiguration('namespace', default='/robot')

    # Include SLAM launch file with namespace
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_navigation_dir, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'namespace': namespace
        }.items()
    )

    # Include RViz visualization launch file with namespace
    view_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_viz_dir, 'launch', 'view_robot.launch.py')
        ),
        launch_arguments={
            'namespace': namespace
        }.items()
    )

    return LaunchDescription([
        slam_launch,
        view_robot_launch
    ])d
