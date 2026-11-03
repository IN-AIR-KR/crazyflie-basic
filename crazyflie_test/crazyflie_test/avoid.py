"""Multiranger(전후좌우 거리 센서)로 장애물을 반사적으로 피하는 제어.

센서 읽기 + 회피 방향 계산은 multiranger.py 의 전용 함수(RangeSensor,
avoid_direction)로 빼놨다. 이 파일은 그걸로 takeoff→회피→land 흐름만 짠다.

⚠️ sim 은 Multiranger 를 전혀 흉내 내지 않는다 — crazyflie_sim 백엔드는
   firmware_logging.default_topics 자체를 구현하지 않아서(pose/status 도
   안 뜨는 것과 같은 이유) /cf1/scan 이 아예 안 생긴다. 그래서 이 코드는
   **실기체에서만** 의미가 있다. "시뮬 검증" 단계는 센서 연결 없이 회피
   로직만 확인하는 --selftest 로 대신한다 (아래 실행법 참고).

읽는 토픽: <prefix>/scan (sensor_msgs/LaserScan, multiranger.py 참고).
   config 의 firmware_logging.default_topics 에 scan 이 있어야 뜬다
   (crazyflies_opticalflow.yaml 에 추가해둠).

알고리즘 — 감지되면 한 걸음 물러나기 (연속 속도 제어 아님):
    매 폴링 주기마다 4방향 거리를 보고, --safe-dist 보다 가까운 방향이 있으면
    그 반대쪽으로 --step-dist 만큼 goTo 한 걸음 보낸다(여러 방향에서 동시에
    가까우면 방향을 합쳐서 정규화). 걸음을 보내면 --step-duration 동안은 새
    걸음을 안 보낸다(쿨다운 — 이전 걸음이 끝나기 전에 또 보내면 기체가
    끊기면서 움직인다). 가까운 게 없으면 goTo 자체를 안 보내고 제자리
    유지(고수준 명령이라 마지막 목표에서 알아서 호버함).

    goTo(고수준)라서 traj/flight.py 의 cmdFullState(저수준) 스트리밍과 달리
    위치를 직접 적분할 필요가 없다 — single_cf.py §4(A) 참고.

⚠️ 회피 방향은 world 좌표계 기준이라, single_cf.py §3 / formation_teleop.py 의
   경고와 같은 이유로 기체가 전원 켜질 때 보던 방향이 "앞(front)"의 기준이다.
   비행 중 yaw 가 크게 틀어지면 센서 방향과 회피 방향이 어긋난다 — 이 버전은
   yaw 를 고정(0)하고 돌지 않는 걸 전제로 한다.

조작: 비행 중 키보드로 멈출 수 있다 (single_cf.py 의 KeyPoller 재사용).
    x             비상정지 — 모터 즉시 차단, 그대로 추락한다. 낮은 고도에서만 쓸 것
    ESC / Ctrl+C  정상 착륙 후 종료 (--duration 다 안 돼도 바로 착륙)

    ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=cflib
    ros2 run crazyflie_test avoid
    ros2 run crazyflie_test avoid --duration 30 --safe-dist 0.3 --step-dist 0.3
    ros2 run crazyflie_test avoid --selftest    # 센서/비행 없이 회피 로직만 검증
"""
import argparse
import sys

TAKEOFF_DURATION = 2.5
LAND_DURATION = 2.5
DEFAULT_HEIGHT = 1.0
DEFAULT_SAFE_DIST = 0.2       # m   이보다 가까우면 회피 걸음 시작
DEFAULT_STEP_DIST = 0.2       # m   감지되면 물러나는 한 걸음 거리
DEFAULT_STEP_DURATION = 1.0   # s   한 걸음 이동 시간(= 다음 걸음까지 쿨다운)
DEFAULT_DURATION = 20.0       # s   회피 비행 지속 시간
POLL_RATE = 20.0              # Hz  센서/키 입력 폴링 주기


def _selftest():
    """ROS 연결 없이 multiranger.avoid_direction() 의 기본 성질만 확인한다."""
    import math

    from .multiranger import avoid_direction
    clear = {'front': math.inf, 'back': math.inf, 'left': math.inf, 'right': math.inf}
    assert avoid_direction(clear, 0.2) is None, '장애물 없으면 None 이어야 함'

    vx, vy = avoid_direction({**clear, 'front': 0.1}, 0.2)
    assert vx < 0 and abs(vy) < 1e-9, f'앞이 가까우면 뒤로(-x) 물러나야 함: {(vx, vy)}'

    vx, vy = avoid_direction({**clear, 'left': 0.1}, 0.2)
    assert vy < 0 and abs(vx) < 1e-9, f'왼쪽이 가까우면 오른쪽(-y)으로 물러나야 함: {(vx, vy)}'

    # 방향 벡터라 거리에 상관없이 크기는 항상 1(혹은 None) — 세기가 아니라 방향만 결정.
    vx, vy = avoid_direction({**clear, 'front': 0.19}, 0.2)
    assert abs(math.hypot(vx, vy) - 1.0) < 1e-9, '단위벡터여야 함(세기는 --step-dist 가 결정)'

    assert avoid_direction({'front': 0.1, 'back': 0.1, 'left': math.inf, 'right': math.inf}, 0.2) is None, \
        '정반대 방향에서 동시에 감지되면 상쇄돼 None 이어야 함'

    print('avoid.py --selftest: 전부 통과')


def parse_args():
    p = argparse.ArgumentParser(description='Multiranger 감지 시 한 걸음 회피')
    p.add_argument('--height', type=float, default=DEFAULT_HEIGHT, help='비행 고도 [m]')
    p.add_argument('--safe-dist', type=float, default=DEFAULT_SAFE_DIST,
                    help='이보다 가까우면 회피 걸음 시작 [m]')
    p.add_argument('--step-dist', type=float, default=DEFAULT_STEP_DIST,
                    help='감지되면 물러나는 한 걸음 거리 [m]')
    p.add_argument('--step-duration', type=float, default=DEFAULT_STEP_DURATION,
                    help='한 걸음 이동 시간(= 쿨다운) [s]')
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

    print(f'[avoid] safe_dist={args.safe_dist} step_dist={args.step_dist} '
          f'duration={args.duration}s')
    print('  x = 비상정지   ESC/Ctrl+C = 착륙 후 종료')

    cf.takeoff(targetHeight=args.height, duration=TAKEOFF_DURATION)
    th.sleep(TAKEOFF_DURATION + 0.5)

    emergency = False
    stop_now = False
    last_step = -1e9
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
                    stop_now = True
            if emergency or stop_now or th.time() - start > args.duration:
                break

            now = th.time()
            if now - last_step > args.step_duration:
                direction = sensor.avoid_direction(args.safe_dist)
                if direction is not None:
                    dx, dy = direction[0] * args.step_dist, direction[1] * args.step_dist
                    _say(f'감지 → 회피 ({dx:+.2f}, {dy:+.2f}) m')
                    cf.goTo([dx, dy, 0.0], yaw=0.0, duration=args.step_duration, relative=True)
                    last_step = now
            th.sleepForRate(POLL_RATE)

    if emergency:
        return  # 모터 이미 차단됨 — land 명령 보내지 않는다
    cf.land(targetHeight=0.04, duration=LAND_DURATION)
    th.sleep(LAND_DURATION + 0.5)


if __name__ == '__main__':
    main()
