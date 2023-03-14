import launch
import launch_ros.actions

def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='roam_pkg',
            executable='roam',
            name='roam',
            output= 'screen'),
        launch_ros.actions.Node(
            package='roam_pkg',
            executable='undock',
            name='undock',
            output= 'screen')
  ])
