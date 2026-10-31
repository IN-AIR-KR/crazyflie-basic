"""여러 대의 Crazyflie를 편대(formation)로 묶어 경로를 따라 비행시킨다.

편대를 다루는 방법은 두 가지뿐이고, 섞어 쓰면 위험하다:

  (A) absolute — 편대 중심이 절대좌표 경로(기본: 사각형)를 따라가고, 각 기체는
      중심 기준 고정 오프셋(대열 모양)을 유지한다. 경로 구간마다 대열 모양을
      바꿀 수 있다(세로줄 → V자 → 원형). 구간마다 cf.goTo(center+offset, ...)
      를 기체별로 걸어두고(비동기), 한 번만 기다린다.
      **공통 절대좌표계가 있을 때만 안전하다 — sim, 또는 mocap.**
      (single_cf.py §3 "Flow deck 에는 절대 좌표계가 없다" 참고)

  (B) broadcast — 편대 전체를 하나의 덩어리처럼 상대 이동시킨다. 대열 모양은
      config 의 initial_position 으로 이미 박아둔 그대로 유지되고 바꿀 수
      없다. **Flow deck 멀티 비행(mocap 없는 실기체)에서 유일하게 안전한
      방법** — 기체마다 자기 이륙 지점 기준 상대 추정이라 공통 좌표계가 없기
      때문이다(multi_hello_world.py 참고). allcfs.goTo(delta, ...) 로 보낸다
      (브로드캐스트 goTo 는 항상 relative).

  mode 를 안 주면 single_cf.py 의 detect_real() 과 같은 방법으로 실기체를
  감지해 자동으로 broadcast(실기체) / absolute(sim) 를 고른다.

    ros2 launch crazyflie_test launch.py mode:=opticalflow_multi backend:=sim
    ros2 run crazyflie_test formation_flight
    ros2 run crazyflie_test formation_flight --shape v --spacing 0.6
    ros2 run crazyflie_test formation_flight --shape line,v,circle   # 구간마다 대열 변경(absolute 전용)
    ros2 run crazyflie_test formation_flight --shape line --path line --side 2.0  # 한 줄로 직진 후 복귀
    ros2 run crazyflie_test formation_flight --mode broadcast        # 실기체 강제(대열 변경 없음)

⚠️ 월요일 실제 비행(동글 + cflib/cpp, mocap 없음): --mode broadcast 를 쓸 것.
   crazyflies_opticalflow_multi.yaml 의 initial_position 간격이 곧 대열 모양이다.
"""
import argparse
import itertools
import math

from crazyflie_py import Crazyswarm
import numpy as np

TAKEOFF_DURATION = 2.5
LAND_DURATION = 2.5
DEFAULT_HEIGHT = 1.0
DEFAULT_SPACING = 0.75      # m   대열 내 기체 간 간격
DEFAULT_SIDE = 1.5          # m   경로(사각형) 한 변
DEFAULT_LEG_DURATION = 3.0  # s   구간당 이동 시간
DEFAULT_SETTLE = 1.0        # s   구간 사이 정착 여유


# ── 대열 모양: 기체 n 대의, 편대 중심 기준 오프셋 ────────────────────────
def _line(n, spacing):
    """좌우로 한 줄 (옆으로 나란히)."""
    ys = (np.arange(n) - (n - 1) / 2.0) * spacing
    return [np.array([0.0, y, 0.0]) for y in ys]


def _v(n, spacing):
    """첨병(0,0,0)을 선두로 좌우 번갈아 뒤로 벌어지는 V자."""
    offs = [np.zeros(3)]
    side, k = 1, 1
    while len(offs) < n:
        offs.append(np.array([-k * spacing, side * k * spacing, 0.0]))
        if side == -1:
            k += 1
        side *= -1
    return offs[:n]


def _circle(n, spacing):
    """중심 기준 반지름 r 의 원 위에 등간격 배치 (이웃 간 간격 ≈ spacing)."""
    if n == 1:
        return [np.zeros(3)]
    r = spacing / (2.0 * math.sin(math.pi / n))
    return [np.array([r * math.cos(2 * math.pi * i / n),
                       r * math.sin(2 * math.pi * i / n), 0.0])
            for i in range(n)]


SHAPES = {'line': _line, 'v': _v, 'circle': _circle}


def _square_path(side, height):
    """편대 중심이 따라갈 경로 — 사각형 한 바퀴(goto_square.py 와 동일 패턴)."""
    return [
        np.array([side, 0.0, height]),
        np.array([side, side, height]),
        np.array([0.0, side, height]),
        np.array([0.0, 0.0, height]),
    ]


def _square_deltas(side):
    """broadcast 모드용 — 사각형 한 바퀴를 네 번의 상대 이동으로."""
    return [np.array([side, 0.0, 0.0]), np.array([0.0, side, 0.0]),
            np.array([-side, 0.0, 0.0]), np.array([0.0, -side, 0.0])]


def _line_path(distance, height):
    """편대 중심이 따라갈 경로 — 직진 후 출발점으로 복귀(왕복 1회)."""
    return [np.array([distance, 0.0, height]), np.array([0.0, 0.0, height])]


def _line_deltas(distance):
    """broadcast 모드용 — 직진 후 복귀를 두 번의 상대 이동으로."""
    return [np.array([distance, 0.0, 0.0]), np.array([-distance, 0.0, 0.0])]


PATHS = {'square': (_square_path, _square_deltas), 'line': (_line_path, _line_deltas)}


# ── (A) absolute: 중심 경로 + 대열 오프셋, 구간마다 대열 모양 변경 가능 ──
def run_absolute(swarm, path, shape_names, spacing, leg_duration, settle, height):
    th = swarm.timeHelper
    cfs = swarm.allcfs.crazyflies
    n = len(cfs)
    # 각 기체의 initial_position 중심을 편대 중심 경로의 원점으로 삼는다
    # (single-cf traj/flight.py 가 cf.initialPosition 을 더하는 것과 동일한 이유).
    centroid_xy = np.mean([cf.initialPosition[:2] for cf in cfs], axis=0)
    print(f'[formation_flight] absolute 모드, {n} 대, 대열 순서 {shape_names}')

    swarm.allcfs.takeoff(targetHeight=height, duration=TAKEOFF_DURATION)
    th.sleep(TAKEOFF_DURATION + 0.5)

    shape_cycle = itertools.cycle(shape_names)
    for center in path:
        shape = next(shape_cycle)
        c = center + np.array([centroid_xy[0], centroid_xy[1], 0.0])
        offsets = SHAPES[shape](n, spacing)
        for cf, off in zip(cfs, offsets):
            cf.goTo(c + off, yaw=0.0, duration=leg_duration)
        th.sleep(leg_duration + settle)

    swarm.allcfs.land(targetHeight=0.04, duration=LAND_DURATION)
    th.sleep(LAND_DURATION + 0.5)


# ── (B) broadcast: 대열 모양 고정(= config initial_position), 전체를 한 덩어리로 상대 이동 ──
def run_broadcast(swarm, deltas, leg_duration, settle, height):
    th = swarm.timeHelper
    allcfs = swarm.allcfs
    print(f'[formation_flight] broadcast 모드, {len(allcfs.crazyflies)} 대 '
          '(대열 모양은 config 의 initial_position 그대로 유지)')

    allcfs.takeoff(targetHeight=height, duration=TAKEOFF_DURATION)
    th.sleep(TAKEOFF_DURATION + 0.5)

    for d in deltas:
        allcfs.goTo(d, yaw=0.0, duration=leg_duration)
        th.sleep(leg_duration + settle)

    allcfs.land(targetHeight=0.04, duration=LAND_DURATION)
    th.sleep(LAND_DURATION + 0.5)


def detect_real(allcfs, timeout=2.0):
    """single_cf.py 의 detect_real() 과 동일 — <name>/pose 발행자가 있으면 실기체."""
    cf = allcfs.crazyflies[0]
    deadline = cf.node.get_clock().now().nanoseconds / 1e9 + timeout
    while cf.node.get_clock().now().nanoseconds / 1e9 < deadline:
        if cf.node.get_publishers_info_by_topic(cf.prefix + '/pose'):
            return True
    return False


def parse_args():
    p = argparse.ArgumentParser(description='여러 Crazyflie를 편대로 묶어 경로 비행')
    p.add_argument('--mode', choices=['auto', 'absolute', 'broadcast'], default='auto',
                    help='auto: 실기체 감지 시 broadcast, 아니면 absolute')
    p.add_argument('--shape', default='line',
                    help="absolute 모드 대열(쉼표로 나열, 경로 구간마다 순환): "
                         f"{','.join(SHAPES)} (예: line,v,circle)")
    p.add_argument('--path', choices=list(PATHS), default='square',
                    help='편대 중심 경로: square(사각형 한 바퀴) 또는 line(직진 후 복귀)')
    p.add_argument('--spacing', type=float, default=DEFAULT_SPACING, help='대열 내 기체 간 간격 [m]')
    p.add_argument('--side', type=float, default=DEFAULT_SIDE,
                    help='경로 크기 [m] (square: 한 변, line: 직진 거리)')
    p.add_argument('--height', type=float, default=DEFAULT_HEIGHT, help='비행 고도 [m]')
    p.add_argument('--leg-duration', type=float, default=DEFAULT_LEG_DURATION, help='구간당 이동 시간 [s]')
    p.add_argument('--settle', type=float, default=DEFAULT_SETTLE, help='구간 사이 정착 여유 [s]')
    args, _ = p.parse_known_args()
    return args


def main():
    args = parse_args()
    swarm = Crazyswarm()

    mode = args.mode
    if mode == 'auto':
        mode = 'broadcast' if detect_real(swarm.allcfs) else 'absolute'
    print(f'[formation_flight] mode={mode}')

    path_fn, deltas_fn = PATHS[args.path]
    if mode == 'absolute':
        shapes = args.shape.split(',')
        unknown = [s for s in shapes if s not in SHAPES]
        if unknown:
            raise SystemExit(f'알 수 없는 대열: {unknown} (가능: {", ".join(SHAPES)})')
        path = path_fn(args.side, args.height)
        run_absolute(swarm, path, shapes, args.spacing, args.leg_duration, args.settle, args.height)
    else:
        if args.shape != 'line':
            print('경고: broadcast 모드는 config 의 initial_position 대열을 그대로 쓴다. '
                  '--shape 은 무시됨.')
        deltas = deltas_fn(args.side)
        run_broadcast(swarm, deltas, args.leg_duration, args.settle, args.height)


if __name__ == '__main__':
    main()
