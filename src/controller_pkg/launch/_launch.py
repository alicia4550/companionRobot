import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='controller_pkg',
            executable='controller',
            name='controller',
            output= 'screen'),
        launch_ros.actions.Node(
            package='controller_pkg',
            executable='undock',
            name='undock',
            output= 'screen'),
  ])