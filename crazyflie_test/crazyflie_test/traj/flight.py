"""주기 도형을 **연속으로 지정 랩 수만큼** 비행하는 공유 러너.

시간워핑 스케줄(ṡ 0→최대→0)로 임의 도형을 **s-공간**에서 다룬다.
도형(period=1 기준)에 속도 스케일 T 를 곱해 목표 `--speed` 를 맞추고, cmdFullState 로
스트리밍한다. 시작/종료만 부드럽게 램프시켜 호버에서 매끄럽게 진입·이탈한다.

    T          = max|dp/ds| / max_speed          # 속도 격리: 도형 고정, T 만 조절
    s(t)       : ṡ 를 0→1/T→0 으로 램프 (총 s 이동 = laps)
    pos = shape(s%1) + [takeoff_xy, 0]
    vel = d1·ṡ ,  acc = d2·ṡ² + d1·s̈
    yaw : forward = atan2(d1y, d1x) (접선, 속도 0 에서도 정의) / constant = 0

⚠️ cmdFullState 는 low-level 제어라 상태추정이 튼튼해야 한다(sim/mocap 권장).
   스트리밍 종료 시 notifySetpointsStop() 후 goTo·land.
"""
import argparse
import math

from crazyflie_py import Crazyswarm
import numpy as np

from . import shapes as sh

DEFAULT_LAPS = 3
DEFAULT_SPEED = 1.0     # m/s  목표 최대 속도
DEFAULT_HEIGHT = 1.0    # m
DEFAULT_RATE = 50.0     # Hz   스트리밍 주파수
DEFAULT_RAMP = 2.0      # s    시작/종료 가감속 시간
DEFAULT_SCALE = 1.0     # 도형 수평 크기 배율


def _smoothstep(u):
    return u * u * (3.0 - 2.0 * u)


def _smoothstep_du(u):
    return 6.0 * u * (1.0 - u)


class StreamSchedule:
    """s(t) 스케줄: ṡ 를 0→(1/T)→0 으로 램프시켜 총 s 이동이 laps 가 되게 한다.

    figure8.PhaseSchedule 를 (φ=2πs 대신) s-공간으로 옮긴 것. eval(t)=(s, ṡ, s̈).
    """

    def __init__(self, period, laps, ramp):
        self.w = 1.0 / period                # 정속 구간 ṡ (1랩/period)
        self.total = float(laps)             # 총 s 이동량 (랩 수)
        max_ramp = self.total / self.w       # 두 램프 합이 전체를 넘지 않도록
        self.ramp = min(ramp, max_ramp)
        self.s_ramp = 0.5 * self.w * self.ramp
        self.t_cruise = max(0.0, (self.total - 2.0 * self.s_ramp) / self.w)
        self.duration = 2.0 * self.ramp + self.t_cruise

    def eval(self, t):
        w, Tr = self.w, self.ramp
        if Tr > 0.0 and t < Tr:                       # 가속
            u = t / Tr
            s = w * Tr * (u**3 - 0.5 * u**4)
            return s, w * _smoothstep(u), w * _smoothstep_du(u) / Tr
        if t < Tr + self.t_cruise:                    # 정속
            return self.s_ramp + w * (t - Tr), w, 0.0
        if Tr > 0.0:                                  # 감속
            u = min((t - Tr - self.t_cruise) / Tr, 1.0)
            v = 1.0 - u
            s = (self.s_ramp + w * self.t_cruise
                 + w * Tr * (0.5 - (v**3 - 0.5 * v**4)))
            return s, w * _smoothstep(v), -w * _smoothstep_du(v) / Tr
        return self.total, 0.0, 0.0


def _scaled(shape_fn, scale):
    """도형의 수평(xy) 크기만 scale 배 (z 오프셋은 유지)."""
    if scale == 1.0:
        return shape_fn

    def fn(s):
        pos, d1, d2 = shape_fn(s)
        m = np.array([scale, scale, 1.0])
        return pos * m, d1 * m, d2 * m
    return fn


class Trajectory:
    """도형 + 속도 스케줄 → 시간 함수. pos/vel/acc/yaw/yawrate 반환."""

    def __init__(self, shape_fn, max_speed, laps, ramp, yaw_mode='forward'):
        self.shape = shape_fn
        unit = sh.max_speed_unit(shape_fn)
        self.period = unit / max(max_speed, 1e-6)     # T = max|dp/ds| / v
        self.sched = StreamSchedule(self.period, laps, ramp)
        self.duration = self.sched.duration
        self.yaw_mode = yaw_mode

    def eval(self, t):
        s_tot, sd, sdd = self.sched.eval(t)
        pos, d1, d2 = self.shape(s_tot % 1.0)
        vel = d1 * sd
        acc = d2 * (sd * sd) + d1 * sdd
        if self.yaw_mode == 'forward':
            # 접선 방향(속도가 0인 시작/끝에서도 정의)
            yaw = math.atan2(d1[1], d1[0])
            denom = d1[0] * d1[0] + d1[1] * d1[1]
            dyaw_ds = (d1[0] * d2[1] - d1[1] * d2[0]) / denom if denom > 1e-9 else 0.0
            yawrate = dyaw_ds * sd
        else:
            yaw, yawrate = 0.0, 0.0
        return pos, vel, acc, yaw, yawrate


def preview(traj, samples=2000):
    """비행 전 확인: 실현 최대 속도/가속도, 형상 bbox."""
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    vmax = amax = 0.0
    for i in range(samples + 1):
        pos, vel, acc, _, _ = traj.eval(traj.duration * i / samples)
        lo = np.minimum(lo, pos)
        hi = np.maximum(hi, pos)
        vmax = max(vmax, float(np.linalg.norm(vel)))
        amax = max(amax, float(np.linalg.norm(acc)))
    return {'vmax': vmax, 'amax': amax, 'bbox': (lo, hi)}


def parse_args(shape_name):
    p = argparse.ArgumentParser(description=f'{shape_name} 주기 궤적 연속 비행')
    p.add_argument('--laps', type=int, default=DEFAULT_LAPS, help='바퀴 수')
    p.add_argument('--speed', type=float, default=DEFAULT_SPEED, help='목표 최대 속도 [m/s]')
    p.add_argument('--yaw', choices=['forward', 'constant'], default='forward',
                   help='forward=진행 방향, constant=고정(0)')
    p.add_argument('--height', type=float, default=DEFAULT_HEIGHT, help='비행 고도 [m]')
    p.add_argument('--scale', type=float, default=DEFAULT_SCALE, help='도형 수평 크기 배율')
    p.add_argument('--rate', type=float, default=DEFAULT_RATE, help='스트리밍 주파수 [Hz]')
    p.add_argument('--ramp', type=float, default=DEFAULT_RAMP, help='시작/종료 가감속 [s]')
    args, _ = p.parse_known_args()
    return args


def run(shape_name, **shape_kwargs):
    """진입점. shape_kwargs 로 도형 파라미터를 덮어쓸 수 있다."""
    args = parse_args(shape_name)
    shape_fn = _scaled(
        sh.get_shape(shape_name, height=args.height, **shape_kwargs), args.scale)
    traj = Trajectory(shape_fn, args.speed, args.laps, args.ramp, args.yaw)
    info = preview(traj)
    lo, hi = info['bbox']

    print(f'[{shape_name}] laps={args.laps} speed={args.speed} yaw={args.yaw} '
          f'height={args.height} scale={args.scale}')
    print(f'  period(1랩) {traj.period:.2f} s, 총 {traj.duration:.1f} s '
          f'(가감속 {traj.sched.ramp:.1f}s×2)')
    print(f'  실현 최대 속도 {info["vmax"]:.2f} m/s (목표 {args.speed}), '
          f'최대 가속도 {info["amax"]:.2f} m/s²')
    print(f'  형상 bbox x[{lo[0]:+.2f},{hi[0]:+.2f}] y[{lo[1]:+.2f},{hi[1]:+.2f}] '
          f'z[{lo[2]:.2f},{hi[2]:.2f}] (이륙 지점 기준 xy 오프셋)')

    swarm = Crazyswarm()
    th = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    init = np.array(cf.initialPosition)
    off = np.array([init[0], init[1], 0.0])       # 도형 xy 를 이륙 지점에 얹는다
    p0 = traj.eval(0.0)[0] + off                  # 궤적 시작 world 좌표 (z 는 도형 고도)

    cf.takeoff(targetHeight=float(p0[2]), duration=3.0)
    th.sleep(3.5)
    cf.goTo(p0, yaw=traj.eval(0.0)[3], duration=3.0)
    th.sleep(3.5)

    start = th.time()
    while not th.isShutdown():
        t = th.time() - start
        if t > traj.duration:
            break
        pos, vel, acc, yaw, yawrate = traj.eval(t)
        cf.cmdFullState(pos + off, vel, acc, yaw, np.array([0.0, 0.0, yawrate]))
        th.sleepForRate(args.rate)

    # 궤적은 s=laps(정수)=시작점에서 속도 0 으로 끝난다. 마지막 setpoint 를 잠깐
    # 더 물려 흔들림을 재운 뒤 착륙.
    # ⚠️ cmdFullState(low-level) 뒤에는 goTo 가 통하지 않는다(sim/펌웨어 공통). land 는
    #    현재 위치에서 바로 되므로 goTo 없이 notifySetpointsStop → land 로 마친다.
    end_pos, _, _, end_yaw, _ = traj.eval(traj.duration)
    for _ in range(int(0.5 * args.rate)):
        cf.cmdFullState(end_pos + off, np.zeros(3), np.zeros(3), end_yaw, np.zeros(3))
        th.sleepForRate(args.rate)
    cf.notifySetpointsStop()
    th.sleep(0.3)
    cf.land(targetHeight=0.04, duration=3.0)
    th.sleep(3.5)
