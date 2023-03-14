import launch
import launch_ros.actions

def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='serial_test_pkg',
            executable='serial',
            name='serial',
             output= 'screen')
  ])
