"""FMS 스택 기동 — rosbridge + FMS 코어 + UI 정적 서버.

crazyflie_server 는 여기서 띄우지 않는다. 기존 `docker/launch.sh` 로 편대를 먼저 올리고,
FMS 는 거기에 "붙는" 구조다. 이래야 시뮬레이션을 재시작하지 않고 UI 만 껐다 켤 수 있다.

    터미널 1:  ./docker/launch.sh mode:=opticalflow_multi backend:=sim
    터미널 2:  ros2 launch crazyflie_fms fms.launch.py mode:=opticalflow_multi
    브라우저:  http://localhost:8080
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PACKAGE_NAME = 'crazyflie_fms'


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='opticalflow_multi',
        description='crazyflie_test 의 config/crazyflies_<mode>.yaml 를 고른다',
    )
    web_port_arg = DeclareLaunchArgument('web_port', default_value='8080')
    bridge_port_arg = DeclareLaunchArgument('bridge_port', default_value='9090')
    # fms_node 와 crazyflie_test 의 drone_main 은 둘 다 /fms/command 를 구독한다.
    # 동시에 띄우면 한 기체에 두 곳이 명령을 내리므로, 스크립트로 몰 때는 false 로 끄고
    # UI(rosbridge + web_server)만 목표 입력기로 쓴다.
    fms_node_arg = DeclareLaunchArgument(
        'with_fms_node', default_value='true',
        description='false 면 UI 만 띄운다 (drone_main 으로 몰 때)')

    mode = LaunchConfiguration('mode')
    web_port = LaunchConfiguration('web_port')
    bridge_port = LaunchConfiguration('bridge_port')
    with_fms_node = LaunchConfiguration('with_fms_node')

    # 기체 목록의 단일 출처는 crazyflie_test 의 config — 편대를 띄운 그 파일 그대로를
    # FMS 도 읽어야 UI 와 실제 서버가 같은 기체를 본다.
    test_config_dir = os.path.join(
        get_package_share_directory('crazyflie_test'), 'config')
    crazyflies_yaml_file = PythonExpression(
        ["'", test_config_dir, "/crazyflies_' + '", mode, "' + '.yaml'"])

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': bridge_port}],
        output='screen',
    )

    fms = Node(
        package=PACKAGE_NAME,
        executable='fms_node',
        name='fms_node',
        parameters=[{'crazyflies_yaml_file': crazyflies_yaml_file}],
        condition=IfCondition(with_fms_node),
        output='screen',
    )

    web = Node(
        package=PACKAGE_NAME,
        executable='web_server',
        name='fms_web_server',
        additional_env={'FMS_WEB_PORT': web_port},
        output='screen',
    )

    return LaunchDescription([
        mode_arg,
        web_port_arg,
        bridge_port_arg,
        fms_node_arg,
        rosbridge,
        fms,
        web,
    ])
