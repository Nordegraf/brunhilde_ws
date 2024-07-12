import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_brunhilde_control = get_package_share_directory('brunhilde_control')

    pkg_brunhilde_gz_sim = get_package_share_directory('brunhilde_gz_sim')
    world = os.path.join(pkg_brunhilde_gz_sim, 'worlds', 'flat.sdf')

    model = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                    os.path.join(get_package_prefix('brunhilde_description'), 'share')
            )

    pkg_brunhilde_bringup = get_package_share_directory('brunhilde_bringup')

    bringup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_brunhilde_bringup, 'launch', 'bringup.launch.py')
                ),
                launch_arguments={'use_sim': 'true'}.items()
            )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': f'-r {world}'}.items()
            )

    controllers = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_brunhilde_control, 'launch', 'sim_control.launch.py')
                )
            )

    spawner = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'Brunhilde',
                    '-topic', 'robot_description',
                    '-z', '0.4',
                    '-x', '0',
                    '-y', '0',
                ],
                output='screen',
            )

    bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{
                    'config_file': os.path.join(pkg_brunhilde_gz_sim, 'config','gz_bridge.yaml'),
                }],
                output='screen'
            )

    # controllers must be started seperately
    # everything we tried to start them in the same launch file failed

    return LaunchDescription([
        model,
        bringup,
        gazebo,
        spawner,
        bridge
    ])
