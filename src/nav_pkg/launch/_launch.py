import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='nav_pkg',
            executable='basic_navigation',
            name='basic_navigation'),
  ])