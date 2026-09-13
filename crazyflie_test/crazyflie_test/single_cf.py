"""단일 Crazyflie 제어 — 작성 전 참고용 정리.

실행:
    ros2 run crazyflie_test single_cf

전제: 터미널 1 에서 서버가 떠 있어야 한다.
    ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=sim rviz:=True   # 시뮬
    ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=cpp rviz:=True   # 실기체


═══════════════════════════════════════════════════════════════════════════
1. 구조 — 내 코드는 어디에 있나
═══════════════════════════════════════════════════════════════════════════

    [이 파일]              [터미널 1: crazyflie_server]         [기체]
    single_cf.py  ──서비스/토픽──▶  서버  ──무선(Crazyradio)──▶  cf7
                  ◀──────────────       ◀──────────────────
                     pose / status            상태 텔레메트리

이 파일은 무선 통신을 직접 하지 않는다. 서버가 열어둔 ROS 인터페이스를
호출할 뿐이다. crazyflie_py 가 그 호출을 감싸주므로 보통은 ROS API 를
직접 만질 일이 없다.


═══════════════════════════════════════════════════════════════════════════
2. 토픽 / 서비스 구조 (기체 이름 cf7 기준)
═══════════════════════════════════════════════════════════════════════════

── 서비스 (요청-응답, 고수준 명령) ──────────────────────────────────────
    /cf7/takeoff              이륙
    /cf7/land                 착륙
    /cf7/go_to                한 점으로 이동
    /cf7/notify_setpoints_stop  스트리밍 종료 통보
    /cf7/upload_trajectory    궤적 업로드 (펌웨어에 저장)
    /cf7/start_trajectory     업로드한 궤적 실행
    /cf7/emergency            비상 정지 (모터 즉시 차단)
    /cf7/arm                  시동  ※ 실기체(cpp)에만 존재, sim 에는 없음

    /all/...                  같은 이름으로 전 기체 브로드캐스트

── 토픽 (스트리밍) ──────────────────────────────────────────────────────
    발행(내가 보냄)
    /cf7/cmd_full_state       pos+vel+acc+yaw+omega  ← 궤적 추종용
    /cf7/cmd_hover            vx, vy, yaw_rate, z    ← 속도 제어용
    /cf7/cmd_vel_legacy       roll/pitch/yawrate/thrust (원시 제어, 권장 안 함)

    구독(서버가 보냄)
    /cf7/pose                 geometry_msgs/PoseStamped  현재 위치·자세
    /cf7/status               crazyflie_interfaces/Status 배터리·통신·시동상태
    /tf                       world → cf7 변환 (rviz 표시용)

⚠️ sim 백엔드는 /cf7/pose 와 /cf7/status 를 발행하지 않는다. (2026-08-28 확인)
   sim 에서 열리는 토픽은 cmd_* 와 robot_description 뿐이다.
   따라서 cf.get_position() / cf.get_status() 는 **실기체(cpp)에서만** 유효하다.
   sim 에서 get_position() 을 부르면 초기값 [0,0,0] 이 그대로 나온다.
   → 위치 피드백이 필요한 코드는 sim 에서 검증할 수 없다. 주의.


═══════════════════════════════════════════════════════════════════════════
3. 좌표계와 위치
═══════════════════════════════════════════════════════════════════════════

    z ↑        world 좌표계 (ENU, 오른손)
      |  y      x = 앞,  y = 왼쪽,  z = 위
      | /       단위: 미터, 각도: 라디안
      |/        yaw = z축 기준 반시계 회전
      +─────→ x

원점은 config/crazyflies_opticalflow.yaml 의 initial_position 이다.

    cf7:
      initial_position: [0.0, 0.0, 0.0]

⚠️ Flow deck(opticalflow) 모드에는 **절대 좌표계가 없다.**
   이륙 지점을 원점으로 삼아 이동량을 누적하는 상대 추정이라 오차가 쌓인다.
   오래 날수록 실제 위치와 추정 위치가 벌어진다. goTo 로 먼 곳을 반복해
   오가는 코드는 점점 어긋난다. 절대 위치가 필요하면 mocap 모드를 써야 한다.

코드에서 이륙 지점을 얻는 법:

    init = np.array(cf.initialPosition)      # yaml 의 initial_position
    off  = np.array([init[0], init[1], 0.0]) # 도형을 이륙 지점에 얹을 때 사용


═══════════════════════════════════════════════════════════════════════════
4. 제어 방법 — 세 가지 층위
═══════════════════════════════════════════════════════════════════════════

── (A) 고수준: takeoff / land / goTo ─────────────────────────────────────
펌웨어의 high-level commander 가 내부에서 부드러운 궤적을 만들어 준다.
명령을 던지고 duration 만큼 기다리면 된다. **가장 안전하고 쉬움.**

    cf.takeoff(targetHeight=1.0, duration=2.5)
    th.sleep(3.0)                            # duration + 여유

    cf.goTo([1.0, 0.0, 1.0], yaw=0.0, duration=3.0)          # 절대 좌표
    cf.goTo([0.5, 0.0, 0.0], yaw=0.0, duration=2.0, relative=True)  # 상대 이동
    th.sleep(3.5)

    cf.land(targetHeight=0.04, duration=2.5)
    th.sleep(3.0)

⚠️ 명령은 비동기다. 보내고 나서 반드시 th.sleep() 으로 기다려야 한다.
   안 기다리면 다음 명령이 이전 동작을 덮어쓴다.
⚠️ 참고: goto_square.py 가 이 방식이다.

── (B) 중간: cmdHover (속도 제어) ────────────────────────────────────────
"어디로" 가 아니라 "어느 방향으로 얼마나 빨리" 를 계속 보낸다.
고도는 z_distance 로 유지된다. 조이스틱 조작이 이 방식이다.

    cf.cmdHover(vx=0.3, vy=0.0, yaw_rate=0.0, z_distance=1.0)

⚠️ 스트리밍이므로 루프에서 계속 보내야 한다. 끊기면 기체가 정지/하강한다.
   보통 20~50 Hz.

── (C) 저수준: cmdFullState (궤적 스트리밍) ──────────────────────────────
위치·속도·가속도·yaw·각속도를 매 주기 직접 지정한다. 가장 정밀하지만
상태추정이 튼튼해야 한다(sim 또는 mocap 권장, Flow deck 단독은 위험).

    cf.cmdFullState(pos, vel, acc, yaw, omega)   # 모두 numpy 3-vector, yaw 는 float

    start = th.time()
    while not th.isShutdown():
        t = th.time() - start
        if t > duration:
            break
        pos, vel, acc, yaw, yawrate = traj.eval(t)
        cf.cmdFullState(pos, vel, acc, yaw, np.array([0.0, 0.0, yawrate]))
        th.sleepForRate(50.0)                    # 50 Hz

⚠️ cmdFullState 를 쓴 뒤에는 goTo 가 듣지 않는다(펌웨어·sim 공통).
   반드시 notifySetpointsStop() 으로 저수준 모드를 빠져나온 뒤 land 한다.

        cf.notifySetpointsStop()
        th.sleep(0.3)
        cf.land(targetHeight=0.04, duration=3.0)

⚠️ 참고: traj/flight.py 가 이 방식이다. circle/oval/figure8/clover/star 전부.


═══════════════════════════════════════════════════════════════════════════
5. 상태 읽기 (실기체 전용)
═══════════════════════════════════════════════════════════════════════════

    pos = cf.get_position()      # [x, y, z] float 리스트  ← /cf7/pose
    pose = cf.get_pose()         # {'position':…, 'orientation':…} 쿼터니언 포함
    st  = cf.get_status()        # dict ← /cf7/status

status 주요 필드:

    battery_voltage    4.2=완충, 3.7=임계(비행 불가), 3.0=방전
    supervisor_info    비트필드
                         1 CAN_BE_ARMED   2 IS_ARMED    4 AUTO_ARM
                         8 CAN_FLY       16 IS_FLYING  32 IS_TUMBLED
                        64 IS_LOCKED
                       0 이면 시동 불가 (대개 배터리 부족)
    pm_state           0 배터리 / 1 충전중 / 2 충전완료 / 3 저전력 / 4 종료
    latency_unicast    왕복 지연 [ms]
    num_rx_unicast     수신 패킷 수 (증가하면 통신 살아있음)

비행 전 점검 예:

    st = cf.get_status()
    if st and st.get('battery_voltage', 0.0) < 4.0:
        print('배터리 부족 — 충전 필요')
        return


═══════════════════════════════════════════════════════════════════════════
6. 기본 골격
═══════════════════════════════════════════════════════════════════════════

    from crazyflie_py import Crazyswarm

    def main():
        swarm = Crazyswarm()              # ROS 초기화 + 서버 연결 대기
        th = swarm.timeHelper             # 시간 유틸
        cf = swarm.allcfs.crazyflies[0]   # 첫 번째(유일한) 기체

        # ... 여기에 제어 코드 ...

TimeHelper 가 제공하는 것:

    th.time()                 현재 시각 [s]
    th.sleep(duration)        duration 초 대기
    th.sleepForRate(hz)       루프에서 hz 주기 유지
    th.isShutdown()           Ctrl+C 등으로 종료 요청됐는지

⚠️ time.sleep() 대신 th.sleep() 을 써야 한다. sim 에서 시뮬레이션 시간을
   따라가야 하기 때문이다.


═══════════════════════════════════════════════════════════════════════════
7. 안전
═══════════════════════════════════════════════════════════════════════════

- 반드시 **sim 에서 먼저** 검증한 뒤 실기체로 옮긴다.
- 실기체 첫 비행은 고도 0.3~0.5 m 로 낮춰서 시작할 것.
- 사방 2 m 이상 여유, 바닥에 무늬 필요(Flow deck 은 무늬로 위치를 추정한다).
- 비상 정지: cf.emergency() — 모터가 **즉시 차단**되어 그대로 추락한다.
  낮은 고도에서만 쓸 것.
- 터미널 2 의 Ctrl+C 는 명령 전송만 멈춘다. 기체는 마지막 명령을 계속 수행한다.
  확실히 멈추려면 터미널 1 을 끈다.


═══════════════════════════════════════════════════════════════════════════
8. 이 파일 — 키보드 텔레오퍼레이션
═══════════════════════════════════════════════════════════════════════════

    ros2 run crazyflie_test single_cf                    # 백엔드 자동 판별
    ros2 run crazyflie_test single_cf --mode setpoint    # 강제 지정
    ros2 run crazyflie_test single_cf --height 0.5 --speed 0.3

── 조작 ─────────────────────────────────────────────────────────────────
    t        이륙            l   착륙
    w/s ↑/↓  앞/뒤           a/d ←/→  왼/오른쪽
    q/e      좌/우 회전       r/f      상승/하강
    h        정지(호버)       ?   도움말
    x        비상 정지 (모터 즉시 차단 — 낮은 고도에서만)
    ESC / Ctrl+C  착륙 후 종료

키를 **누르고 있는 동안** 움직인다. 터미널은 키를 뗀 시점을 알려주지 않으므로
"마지막 입력 후 --hold 초가 지나면 속도 0" 규칙으로 대신한다. 즉 한 번 톡 치면
--hold 만큼 짧게 움직이고 멈춘다. 계속 가려면 누르고 있으면 된다.
(누른 직후 첫 반복까지 OS 의 키 반복 지연 ~0.5 s 가 있어 잠깐 끊겨 보인다.)

── 두 가지 제어 모드 ────────────────────────────────────────────────────
    hover     cmdHover 스트리밍. 4.(B) 방식. **실기체(cpp) 전용.**
              vx/vy 는 기수 기준(body frame)이라 회전하면 앞 방향도 같이 돈다.
    setpoint  키 입력을 목표 위치로 적분해 cmdFullState 스트리밍. 4.(C) 방식.
              **sim 은 cmd_hover 를 구현하지 않아(로그만 찍힘) 이 모드만 움직인다.**

기본값은 자동 판별이다. `/cf7/pose` 발행자가 있으면 실기체로 보고 hover,
없으면 sim 으로 보고 setpoint 를 고른다. --mode 로 덮어쓸 수 있다.

⚠️ setpoint 는 저수준 제어라 Flow deck 단독 실기체에서는 권장하지 않는다.
   실기체는 hover 모드를 쓸 것.
"""
import argparse
import math
import os
import select
import sys
import termios

from crazyflie_py import Crazyswarm
import numpy as np

DEFAULT_HEIGHT = 1.0        # m    이륙 고도
DEFAULT_SPEED = 0.4         # m/s  수평 이동 속도
DEFAULT_YAW_RATE = 60.0     # deg/s 회전 속도
DEFAULT_CLIMB = 0.3         # m/s  상승/하강 속도
DEFAULT_RATE = 30.0         # Hz   스트리밍 주파수
DEFAULT_HOLD = 0.35         # s    마지막 입력 후 속도를 0 으로 되돌리는 시간
MIN_HEIGHT = 0.15           # m
MAX_HEIGHT = 2.0            # m
RESET_SETTLE = 2.0          # s    칼만 리셋 후 수렴 대기
TAKEOFF_DURATION = 2.5
LAND_DURATION = 2.5

HELP = """
  t 이륙 / l 착륙        w,s ↑,↓ 앞뒤    a,d ←,→ 좌우
  q,e 회전               r,f 상승/하강
  h 정지  ? 도움말  x 비상정지  ESC 착륙 후 종료
"""

# 키 → (축, 부호). 축은 아래 Teleop._axis 에서 쌍으로 읽는다.
AXIS_KEYS = ('w', 's', 'a', 'd', 'q', 'e', 'r', 'f')
ARROWS = {'UP': 'w', 'DOWN': 's', 'LEFT': 'a', 'RIGHT': 'd'}


def _say(text):
    """raw 모드에서는 개행에 \\r 이 필요하다."""
    sys.stdout.write(text.replace('\n', '\r\n') + '\r\n')
    sys.stdout.flush()


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def reset_estimator(cf, th, settle=RESET_SETTLE):
    """칼만 추정기를 리셋해 **지금 기체가 놓인 자리**를 (0,0,0) 으로 만든다.

    왜 필요한가:
      기체는 전원을 켠 순간부터 칼만 필터를 돌린다. 바닥에 놓여 있어도 마찬가지다.
      그런데 바닥에서는 높이가 0 이라, flow 측정 모델이 0 으로 나누는 걸 막으려고
      높이를 0.1 m 로 강제한다(펌웨어 mm_flow.c:64). 실제 높이와 다른 값으로
      계산하는 데다 그 높이에서는 광류 센서 초점도 안 맞아 노이즈만 들어온다.
      → 가짜 속도가 계속 적분되어 위치가 한없이 흘러간다.

      그래서 서버를 오래 켜둔 뒤 이륙하면 이미 몇 m 밀린 좌표를 원점으로 알고
      날아오른다. 이륙 직전 리셋이 유일한 해결책이다.

    ⚠️ yaml 의 initial_position 은 실기체에서 쓰이지 않는다. C++ 서버는 펌웨어
       칼만 값을 그대로 TF 로 내보내며 오프셋을 더하지 않는다
       (crazyflie_server.cpp:890). 원점은 오직 이 리셋 시점으로 정해진다.

    ⚠️ 바닥 무늬는 **속도**를 재는 데만 쓰인다. 절대 위치를 알려주지 않는다.
    ⚠️ sim 백엔드에는 파라미터 서비스가 없으므로 실기체에서만 호출할 것.
    ⚠️ settle 동안 기체를 움직이면 안 된다.
    """
    cf.setParam('kalman.resetEstimation', 1)
    th.sleep(0.1)
    cf.setParam('kalman.resetEstimation', 0)
    th.sleep(settle)


class KeyPoller:
    """터미널을 논블로킹 raw 모드로 두고 눌린 키를 읽는다.

    ICANON/ECHO 만 끄므로 ISIG 는 살아 있다 → Ctrl+C 는 평소대로 동작한다.
    한 번에 대기 중인 바이트를 모두 읽어 화살표 이스케이프(\\x1b[A)까지 해석한다.
    """

    ESCAPES = {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        new = termios.tcgetattr(self.fd)
        new[3] &= ~(termios.ICANON | termios.ECHO)
        new[6][termios.VMIN] = 0
        new[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, new)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old)

    def poll(self):
        """대기 중인 입력을 키 토큰 리스트로 반환한다. 없으면 빈 리스트."""
        if not select.select([sys.stdin], [], [], 0.0)[0]:
            return []
        data = os.read(self.fd, 64).decode('utf-8', 'ignore')
        keys, i = [], 0
        while i < len(data):
            if data[i] != '\x1b':                          # 보통 키
                keys.append(data[i])
                i += 1
            elif data[i + 1:i + 2] == '[' and len(data) > i + 2:   # 화살표 등
                keys.append(self.ESCAPES.get(data[i + 2], 'ESC'))
                i += 3
            else:                                          # 단독 ESC
                keys.append('ESC')
                i += 1
        return keys


class Teleop:
    """키 입력을 스트리밍 명령으로 바꾸는 상태 기계.

    키를 뗀 시점을 알 수 없으므로 키마다 마지막 입력 시각을 기록하고
    (now - 마지막 입력) < hold 인 동안만 그 축이 살아 있는 것으로 본다.
    """

    def __init__(self, cf, th, args, mode, arm, real=False):
        self.cf = cf
        self.th = th
        self.args = args
        self.mode = mode
        self.arm = arm
        self.real = real
        self.flying = False
        self.z = args.height
        self.pressed = {}                  # 키 → 마지막 입력 시각
        self.pos = np.zeros(3)             # setpoint 모드의 목표 위치
        self.yaw = 0.0
        self.flown = False                 # 한 번이라도 이륙했나 (pos 재사용 판단)

    # ── 입력 ─────────────────────────────────────────────────────────
    def handle(self, key):
        """키 하나를 처리한다. 종료해야 하면 False 를 반환."""
        key = ARROWS.get(key, key)
        if key in AXIS_KEYS:
            self.pressed[key] = self.th.time()
            return True
        if key == 't':
            self.takeoff()
        elif key == 'l':
            self.land()
        elif key == 'h':
            self.pressed.clear()
            _say('정지 — 호버 유지')
        elif key == 'x':
            self.cf.emergency()
            self.flying = False
            _say('*** 비상 정지 — 모터 차단 ***')
        elif key == '?':
            _say(HELP)
        elif key in ('ESC', '\x03', '\x04'):
            return False
        return True

    def _axis(self, pos_key, neg_key):
        """두 키를 +1 / -1 축으로 읽는다. hold 시간이 지난 키는 0."""
        now = self.th.time()
        hold = self.args.hold
        p = 1.0 if now - self.pressed.get(pos_key, -1e9) < hold else 0.0
        n = 1.0 if now - self.pressed.get(neg_key, -1e9) < hold else 0.0
        return p - n

    # ── 비행 ─────────────────────────────────────────────────────────
    def takeoff(self):
        if self.flying:
            return
        if self.arm:
            self.cf.arm(True)
            self.th.sleep(0.3)
        # 실기체는 이륙 직전에 추정기를 리셋한다. 안 하면 지상에서 누적된
        # 표류가 그대로 원점이 되어 엉뚱한 자리를 기준으로 날아오른다.
        if self.real and self.args.reset:
            _say(f'추정기 리셋 — {RESET_SETTLE:.0f}초간 기체를 움직이지 말 것')
            reset_estimator(self.cf, self.th)
        self.z = self.args.height
        _say(f'이륙 → {self.z:.2f} m')
        self.cf.takeoff(targetHeight=self.z, duration=TAKEOFF_DURATION)
        self.th.sleep(TAKEOFF_DURATION + 0.5)

        # setpoint 모드의 출발점: 위치 피드백 > 직전 비행의 마지막 목표 > 이륙 지점
        base = self.cf.get_position()
        if not any(base):
            base = self.pos if self.flown else list(self.cf.initialPosition)
        self.pos = np.array([base[0], base[1], self.z], dtype=float)
        self.pressed.clear()
        self.flying = True
        self.flown = True

    def land(self):
        if not self.flying:
            return
        self.flying = False
        self.pressed.clear()
        _say('착륙 …')
        self.cf.notifySetpointsStop()      # 저수준 스트리밍 모드에서 빠져나온다
        self.th.sleep(0.3)
        self.cf.land(targetHeight=0.04, duration=LAND_DURATION)
        self.th.sleep(LAND_DURATION + 0.5)
        if self.arm:
            self.cf.arm(False)
        _say('착륙 완료')

    def step(self, dt):
        """한 주기 분의 스트리밍 명령을 보낸다."""
        a = self.args
        vx = a.speed * self._axis('w', 's')      # 기수 기준 앞/뒤
        vy = a.speed * self._axis('a', 'd')      # 기수 기준 왼/오른쪽
        vz = a.climb * self._axis('r', 'f')
        yaw_rate = math.radians(a.yaw_rate) * self._axis('q', 'e')

        self.z = _clamp(self.z + vz * dt, a.min_height, a.max_height)

        if self.mode == 'hover':
            # cmdHover 의 vx/vy 는 body frame, yaw_rate 는 rad/s (양수=반시계).
            self.cf.cmdHover(vx, vy, yaw_rate, self.z)
        else:
            # 키 입력을 world 좌표계 목표 위치로 적분해 cmdFullState 로 보낸다.
            self.yaw += yaw_rate * dt
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            vel = np.array([c * vx - s * vy, s * vx + c * vy, vz])
            self.pos[0] += vel[0] * dt
            self.pos[1] += vel[1] * dt
            self.pos[2] = self.z
            self.cf.cmdFullState(self.pos, vel, np.zeros(3), self.yaw,
                                 np.array([0.0, 0.0, yaw_rate]))

    def status_line(self):
        vx = self.args.speed * self._axis('w', 's')
        vy = self.args.speed * self._axis('a', 'd')
        wz = self.args.yaw_rate * self._axis('q', 'e')
        state = 'FLY ' if self.flying else 'IDLE'
        line = (f'[{state}] {self.mode:8s} v=({vx:+.2f},{vy:+.2f}) '
                f'yaw={wz:+5.1f}d/s z={self.z:.2f}')
        st = self.cf.get_status()
        if st and st.get('battery_voltage'):
            line += f" bat={st['battery_voltage']:.2f}V"
        return line

    def run(self, poller):
        t_prev = self.th.time()
        t_print = 0.0
        while not self.th.isShutdown():
            for key in poller.poll():
                if not self.handle(key):
                    return
            now = self.th.time()
            dt = _clamp(now - t_prev, 1e-3, 0.2)
            t_prev = now
            if self.flying:
                self.step(dt)
            if now - t_print > 0.2:
                t_print = now
                sys.stdout.write('\r' + self.status_line() + '   ')
                sys.stdout.flush()
            self.th.sleepForRate(self.args.rate)


def parse_args():
    p = argparse.ArgumentParser(description='키보드로 Crazyflie 한 대 조종')
    p.add_argument('--mode', choices=['auto', 'hover', 'setpoint'], default='auto',
                   help='제어 방식 (auto: 실기체=hover, sim=setpoint)')
    p.add_argument('--height', type=float, default=DEFAULT_HEIGHT, help='이륙 고도 [m]')
    p.add_argument('--speed', type=float, default=DEFAULT_SPEED, help='수평 속도 [m/s]')
    p.add_argument('--yaw-rate', type=float, default=DEFAULT_YAW_RATE, help='회전 속도 [deg/s]')
    p.add_argument('--climb', type=float, default=DEFAULT_CLIMB, help='상승/하강 속도 [m/s]')
    p.add_argument('--rate', type=float, default=DEFAULT_RATE, help='스트리밍 주파수 [Hz]')
    p.add_argument('--hold', type=float, default=DEFAULT_HOLD,
                   help='마지막 입력 후 속도를 0 으로 되돌리는 시간 [s]')
    p.add_argument('--min-height', type=float, default=MIN_HEIGHT, help='최저 고도 [m]')
    p.add_argument('--max-height', type=float, default=MAX_HEIGHT, help='최고 고도 [m]')
    p.add_argument('--no-arm', dest='arm', action='store_false',
                   help='이륙 시 arm 서비스를 부르지 않는다 (sim 은 arm 이 없다)')
    p.add_argument('--no-reset', dest='reset', action='store_false',
                   help='이륙 전 칼만 추정기 리셋을 생략한다 (실기체 기본은 리셋함)')
    args, _ = p.parse_known_args()          # ROS 인자(--ros-args …) 는 무시
    return args


def detect_real(cf, timeout=2.0):
    """<name>/pose 발행자가 보이면 실기체(cpp), 없으면 sim 으로 판단한다.

    sim 백엔드는 pose/status 를 발행하지 않고 cmd_hover 도 구현되어 있지 않다.
    """
    deadline = cf.node.get_clock().now().nanoseconds / 1e9 + timeout
    while cf.node.get_clock().now().nanoseconds / 1e9 < deadline:
        if cf.node.get_publishers_info_by_topic(cf.prefix + '/pose'):
            return True
    return False


def main():
    args = parse_args()
    if not sys.stdin.isatty():
        print('키보드 입력을 받을 수 없다 — 터미널에서 직접 실행할 것.')
        return

    swarm = Crazyswarm()
    th = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    real = detect_real(cf)
    mode = args.mode if args.mode != 'auto' else ('hover' if real else 'setpoint')
    if mode == 'hover' and not real:
        print('경고: sim 은 cmd_hover 를 구현하지 않아 기체가 움직이지 않는다.')

    print(f"백엔드: {'실기체(cpp)' if real else 'sim'} / 제어 모드: {mode}")
    print(HELP)

    teleop = Teleop(cf, th, args, mode, arm=args.arm and real, real=real)
    try:
        with KeyPoller() as poller:
            teleop.run(poller)
    except KeyboardInterrupt:
        pass
    finally:
        _say('')
        try:
            teleop.land()
        except Exception as exc:           # 종료 중 서비스가 이미 닫힌 경우
            print(f'착륙 명령 실패: {exc}')


if __name__ == '__main__':
    main()
