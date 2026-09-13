"""FMS 코어 노드 — 편대 텔레메트리 수집 + 임무 상태머신 + 웹 UI 게이트웨이.

브라우저(rosbridge 경유)와는 JSON 문자열 두 개로만 대화한다. 전용 .msg 패키지를 두면
ament_cmake + rosidl 로 패키지가 하나 더 늘어나는데, v1 의 정보량에는 과하다.

    /fms/fleet_state  (std_msgs/String, 발행)  편대 전체 스냅샷 — UI 가 이것만 그리면 된다
    /fms/command      (std_msgs/String, 구독)  UI 버튼/클릭이 내려오는 통로

⚠️ 텔레메트리 소스가 백엔드마다 다르다:
   - backend:=sim  은 /tf 만 쏜다 (crazyflie_sim 의 visualization/rviz.py). pose/status 토픽 없음.
   - backend:=cflib|cpp 는 firmware_logging.default_topics 설정에 따라 /<name>/pose,
     /<name>/status 를 쏜다.
   그래서 셋 다 구독해서 합친다. 어느 쪽이 들어오든 UI 는 같은 모양을 본다.
"""
from __future__ import annotations

import json
import math
import os

from crazyflie_interfaces.msg import Status
from crazyflie_interfaces.srv import GoTo, Land, Takeoff
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
import yaml

# 임무 단계
IDLE = 'idle'
TAKEOFF = 'takeoff'
ENROUTE = 'enroute'
HOLD = 'hold'
LANDING = 'landing'

POSE_TIMEOUT = 1.5      # s — 이 시간 안에 위치가 안 들어오면 연결 끊김으로 본다
SETTLE = 0.5            # s — 각 구간 뒤 여유
# goTo 의 명령 시간이 끝나도 기체는 아직 목표로 수렴하는 중이다. 그 시간만큼만 기다리면
# arrival_radius 안에 들어오기 전에 도착 처리돼 진행률이 실제보다 낙관적으로 보인다.
ENROUTE_GRACE = 2.5     # s — 이동 구간에만 붙이는 추가 수렴 대기


class RobotState:
    """기체 한 대의 실시간 상태 + 임무 진행도."""

    def __init__(self, name, initial_position, cf_type):
        self.name = name
        self.initial_position = list(initial_position)
        self.type = cf_type
        # 위치가 한 번도 안 들어왔을 때는 config 의 이륙 지점을 임시로 보여준다.
        self.pos = list(initial_position)
        self.last_pose_time = None
        self.battery = None
        self.pm_state = None
        self.rssi = None
        self.flying = None
        self.goal = None            # [x, y, z] or None
        self.phase = IDLE
        self.progress = 0.0
        self._leg_start = None      # 현재 구간 시작 위치 (진행률 계산용)
        self._leg_dist = 0.0

    def connected(self, now):
        if self.last_pose_time is None:
            return False
        return (now - self.last_pose_time) < POSE_TIMEOUT

    def dist_to_goal(self):
        if self.goal is None:
            return None
        return math.dist(self.pos, self.goal)

    def begin_leg(self):
        """현재 위치를 구간 시작점으로 박아둔다 — 진행률의 분모가 된다."""
        self._leg_start = list(self.pos)
        self._leg_dist = math.dist(self.pos, self.goal) if self.goal else 0.0

    def leg_progress(self):
        """남은 거리 기반 진행률. 구간 거리가 0 에 가까우면 시간 기반으로 넘긴다."""
        if self.goal is None or self._leg_dist < 1e-3:
            return None
        remaining = math.dist(self.pos, self.goal)
        return max(0.0, min(1.0, 1.0 - remaining / self._leg_dist))

    def snapshot(self, now):
        return {
            'name': self.name,
            'type': self.type,
            'initial_position': self.initial_position,
            'pos': [round(v, 3) for v in self.pos],
            'connected': self.connected(now),
            'battery': self.battery,
            'pm_state': self.pm_state,
            'rssi': self.rssi,
            'flying': self.flying,
            'goal': self.goal,
            'phase': self.phase,
            'progress': round(self.progress, 3),
            'dist_to_goal': (round(self.dist_to_goal(), 3)
                             if self.dist_to_goal() is not None else None),
        }


class FmsNode(Node):

    def __init__(self):
        super().__init__('fms_node')

        self.declare_parameter('crazyflies_yaml_file', '')
        self.declare_parameter('takeoff_height', 1.0)
        self.declare_parameter('takeoff_duration', 2.5)
        self.declare_parameter('land_duration', 2.5)
        self.declare_parameter('cruise_speed', 0.5)     # m/s — goTo duration 산출용
        self.declare_parameter('min_leg_duration', 2.0)
        self.declare_parameter('arrival_radius', 0.15)  # m
        self.declare_parameter('publish_rate', 10.0)    # Hz

        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.takeoff_duration = self.get_parameter('takeoff_duration').value
        self.land_duration = self.get_parameter('land_duration').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.min_leg_duration = self.get_parameter('min_leg_duration').value
        self.arrival_radius = self.get_parameter('arrival_radius').value
        rate = self.get_parameter('publish_rate').value

        self.robots = self._load_robots()
        if not self.robots:
            self.get_logger().error(
                'crazyflies yaml 에서 활성 기체를 하나도 못 읽었다. '
                'crazyflies_yaml_file 파라미터를 확인할 것.')

        self.mission = IDLE
        self._phase_t0 = None
        self._phase_duration = 0.0

        # --- 텔레메트리 구독 ---------------------------------------------------
        # sim 경로: /tf 의 child_frame_id 가 기체 이름이다.
        self.create_subscription(TFMessage, '/tf', self._on_tf, 10)
        # 실기 경로: 기체별 pose/status.
        for name in self.robots:
            self.create_subscription(
                PoseStamped, f'/{name}/pose',
                lambda msg, n=name: self._on_pose(n, msg), 10)
            self.create_subscription(
                Status, f'/{name}/status',
                lambda msg, n=name: self._on_status(n, msg), 10)

        # --- 명령 클라이언트 ---------------------------------------------------
        self.cli = {}
        for name in self.robots:
            self.cli[name] = {
                'takeoff': self.create_client(Takeoff, f'/{name}/takeoff'),
                'land': self.create_client(Land, f'/{name}/land'),
                'go_to': self.create_client(GoTo, f'/{name}/go_to'),
            }
        self.emergency_cli = self.create_client(Empty, '/all/emergency')

        # --- UI 연결 -----------------------------------------------------------
        self.state_pub = self.create_publisher(String, '/fms/fleet_state', 10)
        self.create_subscription(String, '/fms/command', self._on_command, 10)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'FMS 기동 — 기체 {len(self.robots)}대: {", ".join(self.robots)}')

    # ------------------------------------------------------------------ config
    def _load_robots(self):
        """crazyflies yaml 의 robots: 에서 enabled 기체만 읽는다.

        기체 목록의 출처를 토픽 스캔이 아니라 config 로 잡은 이유: 아직 연결 안 된
        기체도 UI 에 "연결 끊김" 으로 떠 있어야 편대 관리가 된다.
        """
        path = self.get_parameter('crazyflies_yaml_file').value
        if not path or not os.path.exists(path):
            self.get_logger().error(f'crazyflies yaml 을 못 찾음: {path!r}')
            return {}

        with open(path) as f:
            cfg = yaml.safe_load(f)

        robots = {}
        for name, spec in (cfg.get('robots') or {}).items():
            if not spec.get('enabled', False):
                continue
            robots[name] = RobotState(
                name,
                spec.get('initial_position', [0.0, 0.0, 0.0]),
                spec.get('type', 'default'),
            )
        return robots

    # -------------------------------------------------------------- telemetry
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tf(self, msg: TFMessage):
        # sim 의 tf 타임스탬프는 0 에서 시작하는 시뮬레이션 시간이라 벽시계와 비교하면
        # 항상 "오래됨" 으로 나온다. 신선도는 수신 시각으로 판단한다.
        now = self._now()
        for tr in msg.transforms:
            r = self.robots.get(tr.child_frame_id)
            if r is None:
                continue
            t = tr.transform.translation
            r.pos = [t.x, t.y, t.z]
            r.last_pose_time = now

    def _on_pose(self, name, msg: PoseStamped):
        r = self.robots[name]
        p = msg.pose.position
        r.pos = [p.x, p.y, p.z]
        r.last_pose_time = self._now()

    def _on_status(self, name, msg: Status):
        r = self.robots[name]
        r.battery = round(msg.battery_voltage, 2)
        r.pm_state = int(msg.pm_state)
        r.rssi = int(msg.rssi)
        r.flying = bool(msg.supervisor_info & Status.SUPERVISOR_INFO_IS_FLYING)

    # ---------------------------------------------------------------- commands
    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'명령 JSON 파싱 실패: {msg.data!r}')
            return

        op = cmd.get('op')
        handler = {
            'set_goal': self._cmd_set_goal,
            'clear_goals': self._cmd_clear_goals,
            'start': self._cmd_start,
            'land': self._cmd_land,
            'estop': self._cmd_estop,
        }.get(op)

        if handler is None:
            self.get_logger().warn(f'모르는 명령: {op!r}')
            return
        handler(cmd)

    def _cmd_set_goal(self, cmd):
        name = cmd.get('robot')
        r = self.robots.get(name)
        if r is None:
            self.get_logger().warn(f'모르는 기체: {name!r}')
            return
        goal = cmd.get('goal')
        if goal is None:
            r.goal = None
        else:
            z = goal[2] if len(goal) > 2 else self.takeoff_height
            r.goal = [float(goal[0]), float(goal[1]), float(z)]
        self.get_logger().info(f'[{name}] 목표 = {r.goal}')

    def _cmd_clear_goals(self, cmd):
        for r in self.robots.values():
            r.goal = None
        self.get_logger().info('전체 목표 해제')

    def _cmd_start(self, cmd):
        if self.mission != IDLE:
            self.get_logger().warn(f'이미 임무 진행 중({self.mission}) — start 무시')
            return
        if not any(r.goal for r in self.robots.values()):
            self.get_logger().warn('목표가 하나도 없다 — start 무시')
            return
        self._enter_takeoff()

    def _cmd_land(self, cmd):
        if self.mission in (IDLE, LANDING):
            return
        self._enter_landing()

    def _cmd_estop(self, cmd):
        """비상 정지 — 모터를 즉시 끈다. 기체는 떨어진다."""
        if self.emergency_cli.service_is_ready():
            self.emergency_cli.call_async(Empty.Request())
        self.mission = IDLE
        for r in self.robots.values():
            r.phase = IDLE
            r.progress = 0.0
        self.get_logger().warn('*** 비상 정지 ***')

    # ---------------------------------------------------------- mission phases
    def _active(self):
        """이번 임무에 참여하는 기체 = 목표가 찍힌 기체."""
        return [r for r in self.robots.values() if r.goal is not None]

    def _begin_phase(self, phase, duration):
        self.mission = phase
        self._phase_t0 = self._now()
        self._phase_duration = duration

    def _call(self, name, srv, req):
        cli = self.cli[name][srv]
        if not cli.service_is_ready():
            self.get_logger().warn(f'[{name}] {srv} 서비스가 아직 없다 — 건너뜀')
            return
        cli.call_async(req)

    @staticmethod
    def _duration_msg(seconds):
        req_sec = int(seconds)
        return req_sec, int((seconds - req_sec) * 1e9)

    def _enter_takeoff(self):
        for r in self._active():
            req = Takeoff.Request()
            req.group_mask = 0
            req.height = float(self.takeoff_height)
            sec, nsec = self._duration_msg(self.takeoff_duration)
            req.duration.sec, req.duration.nanosec = sec, nsec
            self._call(r.name, 'takeoff', req)
            r.phase = TAKEOFF
            r.progress = 0.0
        self._begin_phase(TAKEOFF, self.takeoff_duration + SETTLE)
        self.get_logger().info(f'이륙 — {len(self._active())}대')

    def _enter_enroute(self):
        longest = self.min_leg_duration
        for r in self._active():
            r.begin_leg()
            dur = max(self.min_leg_duration, r._leg_dist / max(self.cruise_speed, 1e-6))
            longest = max(longest, dur)

            req = GoTo.Request()
            req.group_mask = 0
            req.relative = False
            req.goal.x, req.goal.y, req.goal.z = [float(v) for v in r.goal]
            req.yaw = 0.0
            sec, nsec = self._duration_msg(dur)
            req.duration.sec, req.duration.nanosec = sec, nsec
            self._call(r.name, 'go_to', req)
            r.phase = ENROUTE
            r.progress = 0.0
        self._begin_phase(ENROUTE, longest + ENROUTE_GRACE)
        self.get_logger().info(f'목표 이동 개시 — 최장 {longest:.1f}s')

    def _enter_landing(self):
        for r in self._active():
            req = Land.Request()
            req.group_mask = 0
            req.height = 0.04
            sec, nsec = self._duration_msg(self.land_duration)
            req.duration.sec, req.duration.nanosec = sec, nsec
            self._call(r.name, 'land', req)
            r.phase = LANDING
            r.progress = 0.0
        self._begin_phase(LANDING, self.land_duration + SETTLE)
        self.get_logger().info('착륙')

    def _phase_elapsed_frac(self):
        if self._phase_duration <= 0:
            return 1.0
        return min(1.0, (self._now() - self._phase_t0) / self._phase_duration)

    def _step_mission(self):
        if self.mission == IDLE:
            return

        frac = self._phase_elapsed_frac()

        if self.mission == TAKEOFF:
            for r in self._active():
                r.progress = frac
            if frac >= 1.0:
                self._enter_enroute()

        elif self.mission == ENROUTE:
            for r in self._active():
                # 거리 기반이 실제 진행에 가깝고, 위치가 안 들어오면 시간으로 대체한다.
                p = r.leg_progress()
                r.progress = frac if p is None else p
                d = r.dist_to_goal()
                if d is not None and d <= self.arrival_radius:
                    r.phase = HOLD
                    r.progress = 1.0
            arrived = all(r.phase == HOLD for r in self._active())
            if arrived or frac >= 1.0:
                for r in self._active():
                    r.phase = HOLD
                    r.progress = 1.0
                self.mission = HOLD
                self.get_logger().info('전 기체 목표 도달 — 호버 유지')

        elif self.mission == LANDING:
            for r in self._active():
                r.progress = frac
            if frac >= 1.0:
                self.mission = IDLE
                for r in self.robots.values():
                    r.phase = IDLE
                    r.progress = 0.0
                self.get_logger().info('임무 종료')

    # -------------------------------------------------------------------- tick
    def _tick(self):
        self._step_mission()

        now = self._now()
        active = self._active()
        overall = (sum(r.progress for r in active) / len(active)) if active else 0.0

        payload = {
            'stamp': now,
            'mission': self.mission,
            'mission_progress': round(overall, 3),
            'takeoff_height': self.takeoff_height,
            'robots': [r.snapshot(now) for r in self.robots.values()],
        }
        self.state_pub.publish(String(data=json.dumps(payload)))


def main():
    rclpy.init()
    node = FmsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
