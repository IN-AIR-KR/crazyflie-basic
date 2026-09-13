"""드론 콜백 계층 — 바깥에서 들어오는 것을 받는 입력 쪽.

drone_stack.py 가 "명령을 내보내는" 출력이라면, 이 파일은 그 반대다.

    관제 UI ──(목표 지점 토픽)──┐
    /tf, /cfN/pose ────────────┼──> DroneCallback ──> drone_main 의 로직
    /cfN/status ───────────────┘

세 가지를 제공한다.
  1. 상태 조회  — position(), battery(), is_connected() 처럼 최신값을 꺼내 쓴다.
  2. 블로킹 대기 — wait_for_goal(), wait_until_arrived() 로 위에서 아래로 읽히는 코드.
  3. 이벤트 등록 — on_goal(fn) 처럼 미리 걸어두면 사건이 생길 때 불린다.

⚠️ 콜백은 **스핀이 도는 동안에만** 불린다. Crazyswarm 의 TimeHelper.sleep() 이 내부에서
   rclpy.spin_once() 를 돌리므로, 스택의 sleep()/hover()/대기 함수를 쓰는 한 자동으로
   처리된다. 반대로 main 이 스핀 없이 오래 계산만 하면 그동안 콜백은 멈춰 있다.

⚠️ 구독을 allcfs 노드에 붙이는 이유도 같다. Crazyswarm 이 spin_once 를 거는 대상이
   바로 그 노드라서, 다른 노드에 붙이면 아무도 스핀해주지 않아 콜백이 영원히 안 온다.
"""
from __future__ import annotations

import json
import math

import numpy as np
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# 관제 UI(crazyflie_fms)가 이미 쓰고 있는 토픽과 스키마를 그대로 받는다.
# 맵 클릭 한 번이 {"op":"set_goal","robot":"cf1","goal":[x,y,z]} 로 날아온다.
DEFAULT_COMMAND_TOPIC = '/fms/command'

ARRIVAL_RADIUS = 0.15   # m — 이 안에 들어오면 도착으로 본다
POSE_TIMEOUT = 1.5      # s — 이 시간 안에 위치가 안 들어오면 연결 끊김
LOW_BATTERY = 3.8       # V — crazyflies.yaml 의 voltage_warning 과 맞춘 값
EVENT_PERIOD = 0.1      # s — 이벤트 판정 주기


class DroneCallback:
    """텔레메트리와 외부 명령을 받아 보관하고, 사건이 생기면 알려주는 계층.

    ⚠️ 관제 UI 의 /fms/command 를 fms_node 도 구독하고 있다. 둘을 동시에 띄우면
       같은 목표에 대해 두 곳에서 기체를 조종해 충돌한다. UI 를 목표 입력용으로만
       쓰려면 fms_node 없이 이 스크립트만 돌리거나, command_topic 을 따로 줄 것.
    """

    def __init__(self, stack, command_topic=DEFAULT_COMMAND_TOPIC,
                 arrival_radius=ARRIVAL_RADIUS, low_battery=LOW_BATTERY):
        """구독을 걸고 이벤트 판정 타이머를 띄운다.

        Args:
            stack (DroneStack): 이미 만들어진 제어 스택. 노드와 기체 목록을 여기서 얻는다.
            command_topic (str): 관제 UI 가 명령을 쏘는 토픽 이름.
            arrival_radius (float): 도착 판정 반경 (m).
            low_battery (float): 배터리 경고 전압 (V).
        """
        self.stack = stack
        self.node = stack.allcfs          # Crazyswarm 이 spin_once 를 거는 바로 그 노드
        self.arrival_radius = arrival_radius
        self.low_battery = low_battery

        self.goals = {}                   # {기체이름: [x, y, z]}
        self.started = False              # UI 에서 START 를 눌렀는가

        self._tf_pos = {}                 # {기체이름: [x, y, z]} — sim 의 유일한 위치원
        self._tf_time = {}                # {기체이름: 수신 시각}
        self._handlers = {
            'goal': [], 'start': [], 'arrived': [],
            'low_battery': [], 'disconnect': [],
        }
        self._arrived = set()             # 도착 이벤트를 이미 쏜 기체
        self._warned_battery = set()
        self._offline = set()

        # sim 백엔드는 /cfN/pose 를 발행하지 않고 tf 만 쏜다. 이 구독이 그 공백을 메운다.
        # (/cfN/pose 와 /cfN/status 구독은 crazyflie_py 가 이미 걸어뒀으므로 중복하지 않는다)
        self.node.create_subscription(TFMessage, '/tf', self._on_tf, 10)
        self.node.create_subscription(String, command_topic, self._on_command, 10)
        self.node.create_timer(EVENT_PERIOD, self._check_events)

    # ====================================================== 내부 콜백
    def _on_tf(self, msg):
        """/tf 수신 — child_frame_id 가 기체 이름인 변환만 골라 위치를 갱신한다."""
        names = set(self.stack.names())
        now = self.stack.time()
        for tr in msg.transforms:
            if tr.child_frame_id not in names:
                continue
            t = tr.transform.translation
            self._tf_pos[tr.child_frame_id] = [t.x, t.y, t.z]
            self._tf_time[tr.child_frame_id] = now

    def _on_command(self, msg):
        """관제 UI 명령 수신 — set_goal / start / clear_goals 를 처리한다."""
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        op = cmd.get('op')
        if op == 'set_goal':
            name = cmd.get('robot')
            goal = cmd.get('goal')
            if name is None:
                return
            if goal is None:
                self.goals.pop(name, None)
                return
            self.goals[name] = [float(v) for v in goal]
            self._arrived.discard(name)     # 새 목표니 도착 이벤트를 다시 쏠 수 있게
            self._fire('goal', name, self.goals[name])
        elif op == 'start':
            self.started = True
            self._fire('start')
        elif op == 'clear_goals':
            self.goals.clear()
            self._arrived.clear()

    def _check_events(self):
        """주기 판정 — 도착/배터리/연결끊김을 살펴 해당 이벤트를 쏜다.

        각 사건은 기체당 한 번만 쏘고, 상태가 풀리면 다시 쏠 수 있게 되돌린다.
        """
        for name in self.stack.names():
            # 도착
            goal = self.goals.get(name)
            if goal is not None and name not in self._arrived:
                d = self.distance_to_goal(name)
                if d is not None and d <= self.arrival_radius:
                    self._arrived.add(name)
                    self._fire('arrived', name, d)

            # 배터리 (sim 에서는 status 가 안 와서 None 이라 그냥 넘어간다)
            v = self.battery(name)
            if v is not None:
                if v < self.low_battery and name not in self._warned_battery:
                    self._warned_battery.add(name)
                    self._fire('low_battery', name, v)
                elif v >= self.low_battery:
                    self._warned_battery.discard(name)

            # 연결 끊김
            if not self.is_connected(name):
                if name not in self._offline:
                    self._offline.add(name)
                    self._fire('disconnect', name)
            else:
                self._offline.discard(name)

    def _fire(self, event, *args):
        """등록된 핸들러들을 부른다. 하나가 터져도 나머지는 계속 돈다."""
        for fn in self._handlers[event]:
            try:
                fn(*args)
            except Exception as exc:                      # noqa: BLE001
                self.node.get_logger().warn(f'{event} 핸들러 예외: {exc}')

    # ====================================================== 상태 조회
    def position(self, which=0):
        """기체의 최신 위치를 읽는다.

        tf 를 먼저 보고, 없으면 crazyflie_py 가 /cfN/pose 로 채운 값을 쓴다.
        이 순서 덕분에 sim(tf만)과 실기(pose) 양쪽에서 같은 함수로 동작한다.

        Args:
            which (int | str): 기체 인덱스 또는 이름.

        Returns:
            np.ndarray | None: [x, y, z] (m). 아직 한 번도 못 받았으면 None.
        """
        name = self._name(which)
        if name in self._tf_pos:
            return np.array(self._tf_pos[name])
        pos = self.stack.get(name).position
        return None if pos == [0.0, 0.0, 0.0] else np.array(pos)

    def is_connected(self, which=0):
        """최근에 위치가 들어오고 있는지 본다.

        Args:
            which (int | str): 기체 인덱스 또는 이름.

        Returns:
            bool: POSE_TIMEOUT 안에 갱신됐으면 True.
        """
        name = self._name(which)
        t = self._tf_time.get(name)
        if t is None:
            return self.stack.get(name).position != [0.0, 0.0, 0.0]
        return (self.stack.time() - t) < POSE_TIMEOUT

    def battery(self, which=0):
        """배터리 전압을 읽는다.

        ⚠️ /cfN/status 는 실기(cflib/cpp)에서만 온다. **sim 에서는 항상 None** 이다.

        Args:
            which (int | str): 기체 인덱스 또는 이름.

        Returns:
            float | None: 전압 (V), 아직 못 받았으면 None.
        """
        status = self.stack.get(self._name(which)).get_status()
        return status.get('battery') if status else None

    def goal(self, which=0):
        """관제 UI 가 찍어준 목표 지점.

        Args:
            which (int | str): 기체 인덱스 또는 이름.

        Returns:
            list | None: [x, y, z] (m), 아직 안 받았으면 None.
        """
        return self.goals.get(self._name(which))

    def distance_to_goal(self, which=0):
        """현재 위치에서 목표까지 남은 거리.

        Args:
            which (int | str): 기체 인덱스 또는 이름.

        Returns:
            float | None: 거리 (m). 목표나 위치를 모르면 None.
        """
        name = self._name(which)
        goal = self.goals.get(name)
        pos = self.position(name)
        if goal is None or pos is None:
            return None
        return float(math.dist(list(pos), goal))

    def pending_goals(self):
        """목표가 찍힌 기체들의 이름과 좌표.

        Returns:
            dict: {기체이름: [x, y, z]} 의 사본.
        """
        return dict(self.goals)

    # ====================================================== 이벤트 등록
    def on_goal(self, fn):
        """관제 UI 가 목표를 찍었을 때 부를 함수를 등록한다.

        Args:
            fn (callable): fn(name, goal) 형태. goal 은 [x, y, z].
        """
        self._handlers['goal'].append(fn)

    def on_start(self, fn):
        """관제 UI 에서 START 를 눌렀을 때 부를 함수를 등록한다.

        Args:
            fn (callable): 인자 없음.
        """
        self._handlers['start'].append(fn)

    def on_arrived(self, fn):
        """기체가 목표 반경 안에 들어왔을 때 부를 함수를 등록한다.

        목표 하나당 한 번만 불린다. 새 목표가 오면 다시 불릴 수 있게 초기화된다.

        Args:
            fn (callable): fn(name, distance) 형태.
        """
        self._handlers['arrived'].append(fn)

    def on_low_battery(self, fn):
        """배터리가 경고 전압 아래로 떨어졌을 때 부를 함수를 등록한다.

        ⚠️ sim 에서는 status 가 없어 영원히 불리지 않는다.

        Args:
            fn (callable): fn(name, voltage) 형태.
        """
        self._handlers['low_battery'].append(fn)

    def on_disconnect(self, fn):
        """기체 위치 수신이 끊겼을 때 부를 함수를 등록한다.

        Args:
            fn (callable): fn(name) 형태.
        """
        self._handlers['disconnect'].append(fn)

    # ====================================================== 블로킹 대기
    def spin(self, seconds):
        """지정 시간 동안 콜백만 돌린다.

        하는 일 없이 외부 명령을 기다릴 때 쓴다. 내부적으로 스택의 sleep() 이라
        spin_once 가 돌아 콜백이 처리된다.

        Args:
            seconds (float): 대기 시간 (s).
        """
        self.stack.sleep(seconds)

    def wait_for_goal(self, timeout=None, poll=0.1):
        """관제 UI 가 목표를 하나라도 찍어줄 때까지 기다린다.

        Args:
            timeout (float | None): 최대 대기 시간 (s). None 이면 무한정.
            poll (float): 확인 주기 (s).

        Returns:
            dict | None: {기체이름: [x, y, z]}. 시간 초과면 None.
        """
        return self._wait(lambda: self.goals or None, timeout, poll)

    def wait_for_start(self, timeout=None, poll=0.1):
        """관제 UI 의 START 를 기다린다.

        한 번 True 가 되면 계속 True 이므로, 다음 임무를 받으려면 reset_start() 할 것.

        Args:
            timeout (float | None): 최대 대기 시간 (s). None 이면 무한정.
            poll (float): 확인 주기 (s).

        Returns:
            bool: START 를 받았으면 True, 시간 초과면 False.
        """
        return bool(self._wait(lambda: self.started or None, timeout, poll))

    def reset_start(self):
        """START 플래그를 내린다. 다음 임무를 기다리기 전에 부른다."""
        self.started = False

    def wait_until_arrived(self, which=0, timeout=30.0, poll=0.1):
        """기체가 목표 반경 안에 들어올 때까지 기다린다.

        goto() 의 wait 인자가 '명령 시간만큼' 기다리는 것과 달리, 이쪽은 **실제 위치를
        보고** 판단한다. 바람이나 추정 오차로 늦어져도 도착을 놓치지 않는다.

        Args:
            which (int | str): 기체 인덱스 또는 이름.
            timeout (float): 최대 대기 시간 (s). 넘으면 포기하고 False.
            poll (float): 확인 주기 (s).

        Returns:
            bool: 도착했으면 True, 시간 초과면 False.
        """
        name = self._name(which)

        def arrived():
            d = self.distance_to_goal(name)
            return True if (d is not None and d <= self.arrival_radius) else None

        return bool(self._wait(arrived, timeout, poll))

    def wait_until_all_arrived(self, timeout=30.0, poll=0.1):
        """목표가 찍힌 **모든** 기체가 도착할 때까지 기다린다.

        Args:
            timeout (float): 최대 대기 시간 (s).
            poll (float): 확인 주기 (s).

        Returns:
            bool: 전부 도착했으면 True, 시간 초과면 False.
        """
        def all_arrived():
            if not self.goals:
                return None
            for name in self.goals:
                d = self.distance_to_goal(name)
                if d is None or d > self.arrival_radius:
                    return None
            return True

        return bool(self._wait(all_arrived, timeout, poll))

    # ====================================================== 내부 헬퍼
    def _wait(self, predicate, timeout, poll):
        """predicate 가 참값을 돌려줄 때까지 스핀하며 기다린다.

        Returns:
            predicate 의 반환값, 또는 시간 초과 시 None.
        """
        start = self.stack.time()
        while not self.stack.is_shutdown():
            value = predicate()
            if value is not None:
                return value
            if timeout is not None and (self.stack.time() - start) > timeout:
                return None
            self.stack.sleep(poll)
        return None

    def _name(self, which):
        """인덱스든 이름이든 기체 이름 문자열로 바꾼다."""
        if isinstance(which, int):
            return self.stack.names()[which]
        return which.strip('/')
