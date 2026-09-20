import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_g2bot_gazebo = get_package_share_directory('g2bot_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_g2bot_gazebo, 'worlds', 'sim_env.world')
    models_path = os.path.join(pkg_g2bot_gazebo, 'models')

    # 新版 Gazebo 用 GZ_SIM_RESOURCE_PATH，Humble/Fortress 用 IGN 前缀，两个都设置
    resource_path = models_path
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        resource_path += os.pathsep + os.environ['IGN_GAZEBO_RESOURCE_PATH']

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': world_file,
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        gz_sim,
    ])
