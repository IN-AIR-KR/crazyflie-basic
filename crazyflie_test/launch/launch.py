import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

PACKAGE_NAME = 'crazyflie_test'


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='opticalflow',
        description='opticalflow(기본), opticalflow_multi, mocap',
    )
    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='cflib',
        description='cflib, cpp, sim 중 하나',
    )

    mode = LaunchConfiguration('mode')
    backend = LaunchConfiguration('backend')

    config_dir = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config')

    # mode 값에 따라 crazyflies_<mode>.yaml 선택
    crazyflies_yaml_file = PythonExpression(
        ["'", config_dir, "/crazyflies_' + '", mode, "' + '.yaml'"])
    motion_capture_yaml_file = os.path.join(config_dir, 'motion_capture.yaml')

    # mocap 모드일 때만 motion_capture_tracking 노드를 켠다
    use_mocap = PythonExpression(
        ["'True' if '", mode, "' == 'mocap' else 'False'"])

    crazyflie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('crazyflie'), 'launch', 'launch.py')
        ),
        launch_arguments={
            'crazyflies_yaml_file': crazyflies_yaml_file,
            'motion_capture_yaml_file': motion_capture_yaml_file,
            'mocap': use_mocap,
            'backend': backend,
        }.items(),
    )

    return LaunchDescription([
        mode_arg,
        backend_arg,
        crazyflie_launch,
    ])
