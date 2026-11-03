"""Multiranger(전후좌우 거리 센서)로 장애물을 반사적으로 피하는 제어.

센서 읽기 + 회피 속도 계산은 multiranger.py 의 전용 함수(RangeSensor,
repulsion)로 빼놨다. 이 파일은 그걸로 takeoff→회피→land 흐름만 짠다.

⚠️ sim 은 Multiranger 를 전혀 흉내 내지 않는다 — crazyflie_sim 백엔드는
   firmware_logging.default_topics 자체를 구현하지 않아서(pose/status 도
   안 뜨는 것과 같은 이유) /cf1/scan 이 아예 안 생긴다. 그래서 이 코드는
   **실기체에서만** 의미가 있다. "시뮬 검증" 단계는 센서 연결 없이 반발력
   계산 로직만 확인하는 --selftest 로 대신한다 (아래 실행법 참고).

읽는 토픽: <prefix>/scan (sensor_msgs/LaserScan, multiranger.py 참고).
   config 의 firmware_logging.default_topics 에 scan 이 있어야 뜬다
   (crazyflies_opticalflow.yaml 에 추가해둠).

알고리즘 — 반발력(potential field), 매 tick 마다:
    각 방향 거리가 --safe-dist 보다 가까우면, 가까울수록(선형) 그 반대 방향으로
    미는 속도를 world 좌표계로 합산한다(4방향 모두 가능, 동시에 여러 방향에서
    가까우면 벡터 합). 속도 크기는 --max-speed 로 clamp. 아무것도 안 가까우면
    속도 0 (제자리 호버 유지) — "장애물 생기면 밀어내고, 없으면 가만히"가
    1차 목표고, 목표 지점으로 이동하는 길찾기는 아직 없다.

    cmdFullState 로 스트리밍하므로(traj/flight.py 와 동일 방식) sim/실기체
    둘 다에서 "명령 보내기" 자체는 통하지만, 위에 적었듯 sim 은 장애물 입력이
    없어서 항상 속도 0(제자리 호버)으로만 동작한다.

⚠️ 회피 속도는 world 좌표계 기준이라, single_cf.py §3 / formation_teleop.py 의
   경고와 같은 이유로 기체가 전원 켜질 때 보던 방향이 "앞(front)"의 기준이다.
   비행 중 yaw 가 크게 틀어지면 센서 방향과 회피 속도 방향이 어긋난다 —
   이 버전은 yaw 를 고정(0)하고 돌지 않는 걸 전제로 한다.

조작: 비행 중 키보드로 멈출 수 있다 (single_cf.py 의 KeyPoller 재사용).
    x             비상정지 — 모터 즉시 차단, 그대로 추락한다. 낮은 고도에서만 쓸 것
    ESC / Ctrl+C  정상 착륙 후 종료 (--duration 다 안 돼도 바로 착륙)

    ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=cflib
    ros2 run crazyflie_test avoid
    ros2 run crazyflie_test avoid --duration 30 --safe-dist 0.3
    ros2 run crazyflie_test avoid --selftest    # 센서/비행 없이 반발력 계산만 검증
"""
import argparse
import sys

import numpy as np

from .multiranger import repulsion

TAKEOFF_DURATION = 2.5
LAND_DURATION = 2.5
DEFAULT_HEIGHT = 1.0
DEFAULT_SAFE_DIST = 0.2    # m   이보다 가까우면 밀어내기 시작
DEFAULT_MAX_SPEED = 0.5    # m/s 회피 속도 상한
DEFAULT_DURATION = 20.0    # s   회피 비행 지속 시간
RATE = 30.0                # Hz  cmdFullState 스트리밍 주파수


def _selftest():
    """ROS 연결 없이 multiranger.repulsion() 의 기본 성질만 확인한다."""
    import math
    clear = {'front': math.inf, 'back': math.inf, 'left': math.inf, 'right': math.inf}
    assert repulsion(clear, 0.2, 0.5) == (0.0, 0.0), '장애물 없으면 속도 0 이어야 함'

    vx, vy = repulsion({**clear, 'front': 0.05}, safe_dist=0.2, max_speed=0.5)
    assert vx < 0 and abs(vy) < 1e-9, f'앞이 가까우면 뒤로(-x) 밀어야 함: {(vx, vy)}'

    vx, vy = repulsion({**clear, 'left': 0.05}, safe_dist=0.2, max_speed=0.5)
    assert vy < 0 and abs(vx) < 1e-9, f'왼쪽이 가까우면 오른쪽(-y)으로 밀어야 함: {(vx, vy)}'

    far_vx, _ = repulsion({**clear, 'front': 0.18}, safe_dist=0.2, max_speed=0.5)
    near_vx, _ = repulsion({**clear, 'front': 0.02}, safe_dist=0.2, max_speed=0.5)
    assert abs(near_vx) > abs(far_vx), '더 가까울수록 더 세게 밀어야 함'

    vx, vy = repulsion({'front': 0.0, 'back': math.inf, 'left': 0.0, 'right': math.inf},
                        safe_dist=0.2, max_speed=0.3)
    assert math.hypot(vx, vy) <= 0.3 + 1e-9, f'MAX_SPEED 를 넘으면 안 됨: {(vx, vy)}'

    print('avoid.py --selftest: 전부 통과')


def parse_args():
    p = argparse.ArgumentParser(description='Multiranger 반발력 장애물 회피')
    p.add_argument('--height', type=float, default=DEFAULT_HEIGHT, help='비행 고도 [m]')
    p.add_argument('--safe-dist', type=float, default=DEFAULT_SAFE_DIST,
                    help='이보다 가까우면 밀어내기 시작 [m]')
    p.add_argument('--max-speed', type=float, default=DEFAULT_MAX_SPEED, help='회피 속도 상한 [m/s]')
    p.add_argument('--duration', type=float, default=DEFAULT_DURATION, help='회피 비행 시간 [s]')
    p.add_argument('--selftest', action='store_true', help='센서/비행 없이 로직만 검증하고 종료')
    args, _ = p.parse_known_args()
    return args


def main():
    args = parse_args()
    if args.selftest:
        _selftest()
        return
    if not sys.stdin.isatty():
        print('키보드 입력을 받을 수 없다 — 터미널에서 직접 실행할 것.')
        return

    # ROS/Crazyswarm 은 --selftest 가 아닐 때만 임포트 — 셀프테스트는 ROS 환경 없이도 돌아가게.
    from crazyflie_py import Crazyswarm

    from .multiranger import RangeSensor
    from .single_cf import KeyPoller, _say

    swarm = Crazyswarm()
    th = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    sensor = RangeSensor(cf)

    print(f'[avoid] safe_dist={args.safe_dist} max_speed={args.max_speed} '
          f'duration={args.duration}s')
    print('  x = 비상정지   ESC/Ctrl+C = 착륙 후 종료')

    init = np.array(cf.initialPosition)
    off = np.array([init[0], init[1], 0.0])
    z = args.height
    pos = off + np.array([0.0, 0.0, z])

    cf.takeoff(targetHeight=z, duration=TAKEOFF_DURATION)
    th.sleep(TAKEOFF_DURATION + 0.5)

    emergency = False
    start = th.time()
    with KeyPoller() as poller:
        while not th.isShutdown():
            for key in poller.poll():
                if key == 'x':
                    cf.emergency()
                    emergency = True
                    _say('*** 비상정지 — 모터 차단 ***')
                elif key in ('ESC', '\x03', '\x04'):
                    _say('착륙 후 종료')
                    start = -1e9  # 아래 duration 체크로 바로 빠지게
            if emergency or th.time() - start > args.duration:
                break
            vx, vy = sensor.avoid_velocity(args.safe_dist, args.max_speed)
            dt = 1.0 / RATE
            pos[0] += vx * dt
            pos[1] += vy * dt
            vel = np.array([vx, vy, 0.0])
            cf.cmdFullState(pos, vel, np.zeros(3), 0.0, np.zeros(3))
            th.sleepForRate(RATE)

    if emergency:
        return  # 모터 이미 차단됨 — land 명령 보내지 않는다
    cf.notifySetpointsStop()
    th.sleep(0.3)
    cf.land(targetHeight=0.04, duration=LAND_DURATION)
    th.sleep(LAND_DURATION + 0.5)


if __name__ == '__main__':
    main()
