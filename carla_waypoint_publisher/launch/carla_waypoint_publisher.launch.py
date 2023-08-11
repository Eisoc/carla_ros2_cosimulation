import launch
import launch_ros.actions
import os




def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='2'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='nbr_vehicles',
            default_value='30'
        ),
        launch.actions.DeclareLaunchArgument(
            name='nbr_walkers',
            default_value='0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='radius',
            default_value='150'
        ),
        launch_ros.actions.Node(
            package='carla_waypoint_publisher',
            executable='carla_waypoint_publisher',
            name='carla_waypoint_publisher',
            output='screen',
            emulate_tty='True',
            parameters=[
                {
                    'host': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
                {
                    "nbr_vehicles":launch.substitutions.LaunchConfiguration('nbr_vehicles')
                },
                {
                    "nbr_walkers":launch.substitutions.LaunchConfiguration('nbr_walkers')
                },
                {
                    "nbr_frame":launch.substitutions.LaunchConfiguration('nbr_frame')
                },
                {
                    "radius":launch.substitutions.LaunchConfiguration('radius')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
