"""드론 제어 스택 — 비행 동작을 함수 하나씩으로 정리한 계층.

시나리오(drone_main.py 등)는 Crazyswarm 객체를 직접 만지지 않고 이 스택의 함수만
호출한다. 이륙/착륙/이동을 스크립트마다 다시 쓰던 것을 여기 한 곳으로 모은 것이다.

    from crazyflie_test.drone_stack import DroneStack

    drone = DroneStack()
    drone.takeoff()
    drone.hover(3.0)
    drone.land()


 브로드캐스트와 개별 명령의 좌표계가 다르다 — 이게 이 파일에서 가장 헷갈리는 지점이다.
   - `goto_all_relative()` (브로드캐스트): crazyflie_py 의 allcfs.goTo 에는 relative 인자가
     아예 없다. **항상 현재 위치 기준 상대 이동**이다.
   - `goto()` (개별): relative=False 면 절대 좌표로 간다.
   optical flow 편대는 공통 절대 좌표계가 없으므로 브로드캐스트 상대 이동만 안전하다.
"""
from crazyflie_py import Crazyswarm
import numpy as np

# 기본값 — 호출부에서 인자로 덮어쓸 수 있다.
TAKEOFF_HEIGHT = 1.0
TAKEOFF_DURATION = 2.5
LAND_HEIGHT = 0.04
LAND_DURATION = 2.5
GOTO_DURATION = 3.0
SETTLE = 0.5        # s — 명령 시간 뒤 기체가 자세를 잡을 여유
STREAM_RATE = 50.0  # Hz — cmdFullState 스트리밍 주파수


class DroneStack:
    """Crazyswarm 위에 얹은 비행 동작 모음.

    생성 시점에 Crazyswarm() 이 ROS 노드를 만들고 서버 연결을 기다리므로,
    시나리오당 한 번만 만든다.
    """

    def __init__(self):
        """Crazyswarm 에 접속하고 편대 핸들을 잡는다.

        crazyflies.yaml 에서 enabled 로 잡힌 기체들이 self.allcfs 에 들어온다.
        """
        self.swarm = Crazyswarm()
        self.time_helper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs

    # ========================================================== 편대 조회
    def crazyflies(self):
        """편대의 Crazyflie 객체 리스트를 그대로 돌려준다.

        스택이 감싸지 않은 crazyflie_py 기능을 직접 써야 할 때의 탈출구다.

        Returns:
            list[Crazyflie]: config 에 적힌 순서 그대로.
        """
        return self.allcfs.crazyflies

    def count(self):
        """편대에 몇 대가 있는지 센다.

        Returns:
            int: enabled 기체 수.
        """
        return len(self.crazyflies())

    def names(self):
        """편대의 기체 이름 목록.

        Returns:
            list[str]: 예) ['cf1', 'cf2', 'cf3']. 앞의 '/' 는 떼고 준다.
        """
        return [cf.prefix.strip('/') for cf in self.crazyflies()]

    def get(self, which=0):
        """기체 하나를 집는다.

        Args:
            which (int | str): 인덱스(0) 또는 이름('cf1'). 둘 다 받는다.

        Returns:
            Crazyflie: 해당 기체 객체.

        Raises:
            KeyError: 그런 이름의 기체가 편대에 없을 때.
        """
        if isinstance(which, int):
            return self.crazyflies()[which]
        for cf in self.crazyflies():
            if cf.prefix.strip('/') == which.strip('/'):
                return cf
        raise KeyError(f'그런 기체가 없다: {which!r} (있는 기체: {self.names()})')

    def position(self, which=0):
        """기체의 최신 위치를 읽는다.

        crazyflie_py 는 /cfN/pose 를 받아 이 값을 채운다. backend:=sim 은 pose 토픽을
           발행하지 않으므로(tf 만 쏜다) **sim 에서는 항상 [0, 0, 0]** 이다. 시뮬레이션
           시나리오에서 위치 피드백이 필요하면 initial_position() 과 명령값으로 추론할 것.

        Args:
            which (int | str): 기체 인덱스 또는 이름.

        Returns:
            np.ndarray: [x, y, z] (m).
        """
        return np.array(self.get(which).position)

    def initial_position(self, which=0):
        """기체의 이륙 지점을 읽는다.

        crazyflies.yaml 의 initial_position 값이라 비행 중에도 변하지 않는다.
        도형/궤적을 이륙 지점 기준으로 얹을 때 오프셋으로 쓴다.

        Args:
            which (int | str): 기체 인덱스 또는 이름.

        Returns:
            np.ndarray: [x, y, z] (m).
        """
        return np.array(self.get(which).initialPosition)

    # ============================================================== 시간
    def sleep(self, seconds):
        """지정 시간만큼 기다린다.

        time.sleep 이 아니라 Crazyswarm 의 timeHelper 를 쓴다. sim 백엔드에서는
        시뮬레이션 시간에 맞춰 흐르기 때문이다.

        Args:
            seconds (float): 대기 시간 (s).
        """
        self.time_helper.sleep(seconds)

    def hover(self, seconds):
        """현재 자세 그대로 제자리에 머문다.

        고수준 커맨더는 마지막 목표를 계속 유지하므로, 호버는 '아무 명령도 안 보내고
        기다리는 것' 과 같다. 즉 sleep() 과 동작이 같고, 이름만 의도를 드러낸다.

        Args:
            seconds (float): 호버 시간 (s).
        """
        self.sleep(seconds)

    def is_shutdown(self):
        """ROS 가 종료 중인지 본다.

        스트리밍 루프처럼 오래 도는 반복문의 탈출 조건으로 쓴다.

        Returns:
            bool: 종료 신호를 받았으면 True.
        """
        return self.time_helper.isShutdown()

    def sleep_for_rate(self, rate_hz):
        """주기 루프의 남은 시간만큼 잔다.

        직전 호출 이후 흐른 시간을 빼고 자므로, 반복문이 지정 주파수로 돈다.

        Args:
            rate_hz (float): 목표 루프 주파수 (Hz).
        """
        self.time_helper.sleepForRate(rate_hz)

    def time(self):
        """현재 시각을 읽는다.

        sim 백엔드에서는 시뮬레이션 시간이다. 경과 시간 측정에 쓴다.

        Returns:
            float: 초 단위 시각.
        """
        return self.time_helper.time()

    # ====================================================== 이륙 / 착륙
    def takeoff(self, height=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION,
                which=None, wait=True):
        """이륙시킨다.

        which 를 생략하면 **전 기체 브로드캐스트**다. 브로드캐스트는 한 패킷이 모든
        기체에 동시에 도착해서, 기체 수가 늘어도 이륙 타이밍이 어긋나지 않는다.
        기체별로 루프를 돌며 명령하면 전송 지연이 쌓여 순차적으로 뜬다.

        Args:
            height (float): 목표 고도 (m).
            duration (float): 이륙에 쓸 시간 (s).
            which (int | str | None): 대상 기체. None 이면 전 기체 브로드캐스트.
            wait (bool): True 면 duration + SETTLE 만큼 기다렸다 반환한다.
        """
        target = self.allcfs if which is None else self.get(which)
        target.takeoff(targetHeight=height, duration=duration)
        if wait:
            self.sleep(duration + SETTLE)

    def land(self, height=LAND_HEIGHT, duration=LAND_DURATION,
             which=None, wait=True):
        """착륙시킨다.

        which 를 생략하면 전 기체 브로드캐스트다.

        Args:
            height (float): 착륙 목표 고도 (m). 바닥에 닿도록 0 에 가깝게 둔다.
            duration (float): 하강에 쓸 시간 (s).
            which (int | str | None): 대상 기체. None 이면 전 기체 브로드캐스트.
            wait (bool): True 면 duration + SETTLE 만큼 기다렸다 반환한다.
        """
        target = self.allcfs if which is None else self.get(which)
        target.land(targetHeight=height, duration=duration)
        if wait:
            self.sleep(duration + SETTLE)

    # ============================================================== 이동
    def goto(self, position, which=0, yaw=0.0, duration=GOTO_DURATION,
             relative=False, wait=True):
        """기체 **한 대**를 목표 지점으로 보낸다.

         optical flow 모드에서는 기체마다 원점이 자기 이륙 지점이라, 같은 절대 좌표가
           기체별로 다른 곳을 가리킨다. 편대를 통째로 옮기려면 goto_all_relative() 를 쓸 것.

        Args:
            position (list | np.ndarray): 목표 [x, y, z] (m).
            which (int | str): 대상 기체 인덱스 또는 이름.
            yaw (float): 목표 기수 방향 (rad).
            duration (float): 이동에 쓸 시간 (s). 짧을수록 빠르고 거칠다.
            relative (bool): True 면 현재 위치 기준 상대 이동, False 면 절대 좌표.
            wait (bool): True 면 duration + SETTLE 만큼 기다렸다 반환한다.
        """
        self.get(which).goTo(np.array(position, dtype=float), yaw=yaw,
                             duration=duration, relative=relative)
        if wait:
            self.sleep(duration + SETTLE)

    def goto_all_relative(self, delta, yaw=0.0, duration=GOTO_DURATION, wait=True):
        """전 기체를 **같은 변위만큼** 동시에 옮긴다 (브로드캐스트).

        crazyflie_py 의 allcfs.goTo 에는 relative 인자가 없다 — 언제나 상대 이동이다.
        그래서 편대 간격이 그대로 유지되고, 공통 절대 좌표계가 없는 optical flow
        편대에서도 안전하다.

        상대 이동을 반복하면 오차가 누적된다. 왕복을 여러 번 도는 시나리오라면
           출발점으로 정확히 돌아오지 않는다는 점을 감안할 것.

        Args:
            delta (list | np.ndarray): 이동 변위 [dx, dy, dz] (m).
            yaw (float): 목표 기수 방향 (rad).
            duration (float): 이동에 쓸 시간 (s).
            wait (bool): True 면 duration + SETTLE 만큼 기다렸다 반환한다.
        """
        self.allcfs.goTo(np.array(delta, dtype=float), yaw=yaw, duration=duration)
        if wait:
            self.sleep(duration + SETTLE)

    def waypoints(self, points, which=0, yaw=0.0, duration=GOTO_DURATION,
                  settle=1.0, relative=False):
        """경유점들을 순서대로 훑는다.

        각 점마다 goto() 를 부르고 다음 점으로 넘어가기 전에 settle 만큼 더 쉰다.
        점마다 자세를 잡고 가므로 경로가 각지게 나온다. 매끄러운 곡선이 필요하면
        stream_full_state() 를 쓸 것.

        Args:
            points (list): 경유점 리스트. 각 원소는 [x, y, z] (m).
            which (int | str): 대상 기체 인덱스 또는 이름.
            yaw (float): 전 구간 공통 기수 방향 (rad).
            duration (float): 한 구간당 이동 시간 (s).
            settle (float): 각 점 도달 후 추가 대기 (s).
            relative (bool): True 면 각 점을 상대 변위로 해석한다.
        """
        for point in points:
            self.goto(point, which=which, yaw=yaw, duration=duration,
                      relative=relative, wait=False)
            self.sleep(duration + settle)

    # ================================================ 저수준 스트리밍 제어
    def stream_full_state(self, eval_fn, duration, which=0, rate=STREAM_RATE,
                          hold=0.5):
        """cmdFullState 로 궤적을 실시간 스트리밍한다.

        eval_fn(t) 를 rate Hz 로 호출해 나온 상태를 그대로 기체에 흘린다.
        goto() 가 점과 점을 잇는 것과 달리, 위치·속도·가속도를 매 순간 지정하므로
        임의의 매끄러운 곡선을 그릴 수 있다.

        cmdFullState 는 low-level 제어라 상태추정이 튼튼해야 한다(sim/mocap 권장).
        
        스트리밍 뒤에는 goTo 가 통하지 않는다(sim/펌웨어 공통). 그래서 마지막
           setpoint 를 hold 초만큼 더 물려 흔들림을 재운 뒤 notifySetpointsStop() 으로
           고수준 커맨더에 제어권을 돌려준다. 이후엔 land() 만 하면 된다.

        Args:
            eval_fn (callable): t(초) -> (pos, vel, acc, yaw, yawrate).
                pos/vel/acc 는 길이 3 배열, yaw/yawrate 는 스칼라 (rad, rad/s).
            duration (float): 스트리밍할 총 시간 (s).
            which (int | str): 대상 기체 인덱스 또는 이름.
            rate (float): 스트리밍 주파수 (Hz). 실기에서 기체가 많으면 낮출 것.
            hold (float): 궤적 종료 후 마지막 setpoint 를 유지할 시간 (s).
        """
        cf = self.get(which)
        start = self.time()
        while not self.is_shutdown():
            t = self.time() - start
            if t > duration:
                break
            pos, vel, acc, yaw, yawrate = eval_fn(t)
            cf.cmdFullState(pos, vel, acc, yaw, np.array([0.0, 0.0, yawrate]))
            self.sleep_for_rate(rate)

        end_pos, _, _, end_yaw, _ = eval_fn(duration)
        for _ in range(int(hold * rate)):
            cf.cmdFullState(end_pos, np.zeros(3), np.zeros(3), end_yaw, np.zeros(3))
            self.sleep_for_rate(rate)

        cf.notifySetpointsStop()
        self.sleep(0.3)

    # ============================================================== 안전
    def emergency(self):
        """전 기체의 모터를 즉시 끈다.

        감속 없이 출력이 0 이 된다. 기체는 그 자리에서 **떨어진다**.
           정상 종료는 land() 를 쓰고, 이건 통제를 잃었을 때만 쓴다.
        """
        self.allcfs.emergency()

    def arm(self, armed=True, which=None):
        """기체를 armed 상태로 만든다.

        armed 가 아니면 이륙 명령이 무시되는 펌웨어가 있다. sim 에서는 대개 불필요하고
        실기에서 필요할 수 있다.

        Args:
            armed (bool): True 면 arm, False 면 disarm.
            which (int | str | None): 대상 기체. None 이면 전 기체 브로드캐스트.
        """
        target = self.allcfs if which is None else self.get(which)
        target.arm(armed)
