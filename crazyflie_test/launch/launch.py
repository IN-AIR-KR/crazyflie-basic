import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import yaml

PACKAGE_NAME = 'crazyflie_test'
# ros_gz_sim's bundled empty.sdf uses this world name.
WORLD_NAME = 'empty'


def gazebo_models(context):
    """Spawn one visual model for every enabled Crazyflie."""
    mode = LaunchConfiguration('mode').perform(context)
    config_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'config', f'crazyflies_{mode}.yaml')
    with open(config_file, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    mesh = os.path.join(
        get_package_share_directory('crazyflie_description'),
        'urdf', 'cf2_assembly_with_props.dae')
    condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('backend'), "' == 'sim' and '",
        LaunchConfiguration('gazebo'), "'.lower() == 'true'",
    ]))
    actions = []
    robot_names = []
    for name, robot in config['robots'].items():
        if not robot.get('enabled', False):
            continue
        robot_names.append(name)
        x, y, z = robot.get('initial_position', [0.0, 0.0, 0.0])
        sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="body">
      <visual name="crazyflie_visual">
        <geometry><mesh><uri>file://{mesh}</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>'''
        actions.append(Node(
            package='ros_gz_sim', executable='create',
            name=f'spawn_{name}', output='screen', condition=condition,
            arguments=['-world', WORLD_NAME, '-name', name, '-string', sdf,
                       '-x', str(x), '-y', str(y), '-z', str(z)],
        ))

    if robot_names:
        actions.append(Node(
            package='crazyflie_test', executable='gazebo_pose_bridge',
            name='gazebo_pose_bridge', output='screen', condition=condition,
            parameters=[{
                'robot_names': robot_names,
                'reference_frame': config.get('all', {}).get(
                    'reference_frame', 'world'),
                'world_name': WORLD_NAME,
            }],
        ))
    return actions


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
    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='True',
        description='backend:=sim 일 때 Gazebo Sim 시각화 실행',
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r -v 2 empty.sdf',
            'on_exit_shutdown': 'True',
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", backend, "' == 'sim' and '",
            LaunchConfiguration('gazebo'), "'.lower() == 'true'",
        ])),
    )

    set_pose_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gazebo_set_pose_service_bridge', output='screen',
        arguments=[
            f'/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        condition=IfCondition(PythonExpression([
            "'", backend, "' == 'sim' and '",
            LaunchConfiguration('gazebo'), "'.lower() == 'true'",
        ])),
    )

    return LaunchDescription([
        mode_arg,
        backend_arg,
        gazebo_arg,
        gazebo,
        set_pose_bridge,
        OpaqueFunction(function=gazebo_models),
        crazyflie_launch,
    ])
