"""여러 드론을 키보드 버튼으로 조종 — 이륙 / 전후좌우 이동(대열 유지) / 착륙.

편대 전체를 한 덩어리로 다룬다(allcfs 브로드캐스트 — 항상 상대 이동).
대열 모양은 config 의 initial_position 그대로 유지되고 바꾸지 않는다.
브로드캐스트 명령 자체는 sim 과 실기체(Flow deck, mocap 없음)에서 똑같이
안전하게 동작한다.

⚠️ "전진(+x)"은 나침반 방향이 아니라 각 기체가 전원 켜질 때(칼만 추정기가
   초기화되는 시점) 보고 있던 방향이 기준이다(world 좌표계, single_cf.py §3
   참고). 여러 대를 같은 방향으로 전진시키려면 비행 전 모든 기체를
   **같은 방향을 보게** 놓을 것.

⚠️ 이륙 전 칼만 추정기 리셋(single_cf.py 의 reset_estimator())은 여기서 안
   한다 — backend:=cflib 서버는 kalman.resetEstimation 을 yaml 설정에 없는
   파라미터라 ROS 쪽에 등록하지 않아서 setParam() 이 조용히(경고만 찍고)
   실패한다(crazyflie/config/server.yaml 의 query_all_values_on_connect 를
   true 로 바꿔야 가능하나, 전체 서버 공용 설정이라 손대지 않았다).

    ros2 launch crazyflie_test launch.py mode:=opticalflow_multi backend:=sim
    ros2 run crazyflie_test formation_teleop

조작:
    t        이륙 (전체 동시)
    w,s      앞,뒤 한 걸음 (대열 유지한 채 --step m, 연속으로 눌러도 됨)
    a,d      왼쪽,오른쪽 한 걸음
    l        착륙 (전체 동시)
    x        비상정지 (전체 동시 — 모터 즉시 차단, 그대로 추락한다. 낮은 고도에서만 쓸 것)
    ESC / Ctrl+C  착륙 후 종료
"""
import argparse
import sys

from crazyflie_py import Crazyswarm

from .formation_flight import detect_real
from .single_cf import KeyPoller, _say

TAKEOFF_DURATION = 2.5
LAND_DURATION = 2.5
DEFAULT_HEIGHT = 1.0
DEFAULT_STEP = 0.5            # m   한 걸음 이동 거리
DEFAULT_STEP_DURATION = 1.5   # s   한 걸음 이동 시간

# 키 → world 좌표계 이동 방향(dx, dy). x=앞, y=왼쪽 (ENU).
MOVE_KEYS = {
    'w': (1.0, 0.0), 's': (-1.0, 0.0),
    'a': (0.0, 1.0), 'd': (0.0, -1.0),
}

HELP = """
  t 이륙   w,s 앞,뒤   a,d 왼쪽,오른쪽   l 착륙   x 비상정지   ? 도움말   ESC 착륙 후 종료
"""


def parse_args():
    p = argparse.ArgumentParser(description='편대 키보드 조종 (이륙/전진/착륙)')
    p.add_argument('--height', type=float, default=DEFAULT_HEIGHT, help='이륙 고도 [m]')
    p.add_argument('--step', type=float, default=DEFAULT_STEP, help='전진 한 걸음 거리 [m]')
    p.add_argument('--step-duration', type=float, default=DEFAULT_STEP_DURATION,
                   help='한 걸음 이동 시간 [s]')
    args, _ = p.parse_known_args()
    return args


def main():
    args = parse_args()
    if not sys.stdin.isatty():
        print('키보드 입력을 받을 수 없다 — 터미널에서 직접 실행할 것.')
        return

    swarm = Crazyswarm()
    th = swarm.timeHelper
    allcfs = swarm.allcfs
    flying = False
    real = detect_real(allcfs)

    print(f'[formation_teleop] {len(allcfs.crazyflies)} 대 / {"실기체" if real else "sim"}')
    print(HELP)

    def takeoff():
        nonlocal flying
        if flying:
            return
        _say(f'이륙 → {args.height:.2f} m')
        allcfs.takeoff(targetHeight=args.height, duration=TAKEOFF_DURATION)
        th.sleep(TAKEOFF_DURATION + 0.5)
        flying = True

    def move(dx, dy):
        if not flying:
            _say('아직 이륙 안 함 — t 먼저')
            return
        delta = [dx * args.step, dy * args.step, 0.0]
        _say(f'이동 ({delta[0]:+.2f}, {delta[1]:+.2f}) m')
        allcfs.goTo(delta, yaw=0.0, duration=args.step_duration)

    def land():
        nonlocal flying
        if not flying:
            return
        _say('착륙 …')
        allcfs.land(targetHeight=0.04, duration=LAND_DURATION)
        th.sleep(LAND_DURATION + 0.5)
        flying = False

    def emergency():
        nonlocal flying
        allcfs.emergency()
        flying = False
        _say('*** 비상정지 — 모터 차단 ***')

    try:
        with KeyPoller() as poller:
            while not th.isShutdown():
                for key in poller.poll():
                    if key == 't':
                        takeoff()
                    elif key in MOVE_KEYS:
                        move(*MOVE_KEYS[key])
                    elif key == 'l':
                        land()
                    elif key == 'x':
                        emergency()
                    elif key == '?':
                        _say(HELP)
                    elif key in ('ESC', '\x03', '\x04'):
                        raise KeyboardInterrupt
                th.sleepForRate(20.0)
    except KeyboardInterrupt:
        pass
    finally:
        _say('')
        try:
            land()
        except Exception as exc:
            print(f'착륙 명령 실패: {exc}')


if __name__ == '__main__':
    main()
