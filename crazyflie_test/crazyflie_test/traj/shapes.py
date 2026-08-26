"""도형 정의 — `shape_fn(s) -> (pos, d1, d2)` 계약.

phase `s ∈ [0, 1)` (주기 1) 를 받아 **위치와 s에 대한 1·2차 도함수**를 반환한다.
`s` 는 스칼라 또는 numpy 배열 모두 지원(벡터화). 반환은 각각 위치/도함수이며,
스칼라 입력이면 `(3,)`, 배열 입력이면 `(N, 3)`.

    pos  = p(s)
    d1   = dp/ds
    d2   = d²p/ds²

닫힌 파라메트릭 수식(circle/oval/figure8/clover)은 이미 C∞·주기적이라 도함수를
**해석적으로**(체인룰, `a = 2π·s`, `da/ds = 2π`) 구현한다 — 유한차분 금지.
임의 도형(star 등)만 `scipy` 주기 스플라인으로 처리한다.

속도 스케일링(속도 격리)은 `flight.py` 가 담당한다. 여기서는 period=1
기준의 순수 도형과 그 s-도함수만 제공한다.
"""
import math

import numpy as np

TWO_PI = 2.0 * math.pi


def _wrap(fn):
    """스칼라/배열 s 를 모두 받도록 감싸고, 스칼라면 (3,) 로 돌려준다."""
    def wrapped(s):
        scalar = np.isscalar(s)
        s = np.atleast_1d(np.asarray(s, dtype=float))
        pos, d1, d2 = fn(s)               # 각각 (N, 3)
        if scalar:
            return pos[0], d1[0], d2[0]
        return pos, d1, d2
    return wrapped


# --------------------------------------------------------------------------- #
# 해석적 도형 (a = 2π·s)
# --------------------------------------------------------------------------- #
def circle(R=1.0, height=1.0):
    """원: x=R cos a, y=R sin a, z=height."""
    w = TWO_PI

    def fn(s):
        a = w * s
        ca, sa = np.cos(a), np.sin(a)
        z = np.full_like(s, height)
        zero = np.zeros_like(s)
        pos = np.stack([R * ca, R * sa, z], axis=-1)
        d1 = w * np.stack([-R * sa, R * ca, zero], axis=-1)
        d2 = w * w * np.stack([-R * ca, -R * sa, zero], axis=-1)
        return pos, d1, d2
    return _wrap(fn)


def oval(A=1.2, B=0.8, height=1.0):
    """타원: x=A cos a, y=B sin a, z=height."""
    w = TWO_PI

    def fn(s):
        a = w * s
        ca, sa = np.cos(a), np.sin(a)
        z = np.full_like(s, height)
        zero = np.zeros_like(s)
        pos = np.stack([A * ca, B * sa, z], axis=-1)
        d1 = w * np.stack([-A * sa, B * ca, zero], axis=-1)
        d2 = w * w * np.stack([-A * ca, -B * sa, zero], axis=-1)
        return pos, d1, d2
    return _wrap(fn)


def figure8(A=1.2, B=1.6, height=1.0):
    """리사주 8자: x=A sin a, y=(B/2) sin 2a, z=height. (y 진폭 = B/2)"""
    w = TWO_PI
    h = 0.5 * B

    def fn(s):
        a = w * s
        sa, ca = np.sin(a), np.cos(a)
        s2, c2 = np.sin(2 * a), np.cos(2 * a)
        z = np.full_like(s, height)
        zero = np.zeros_like(s)
        pos = np.stack([A * sa, h * s2, z], axis=-1)
        # d/da: [A ca, h·2 c2, 0];  d²/da²: [-A sa, -h·4 s2, 0]
        d1 = w * np.stack([A * ca, 2 * h * c2, zero], axis=-1)
        d2 = w * w * np.stack([-A * sa, -4 * h * s2, zero], axis=-1)
        return pos, d1, d2
    return _wrap(fn)


def clover(A=1.0, height=1.0, z_amp=0.4):
    """네잎 장미: r=A cos(2θ), θ=a → x=r cosθ, y=r sinθ; z=height+z_amp·sin(2θ) (약한 3D)."""
    w = TWO_PI

    def fn(s):
        th = w * s
        c2, s2 = np.cos(2 * th), np.sin(2 * th)
        ct, st = np.cos(th), np.sin(th)
        r = A * c2
        dr = -2 * A * s2          # dr/dθ
        ddr = -4 * A * c2         # d²r/dθ²
        # θ-도함수
        x_th = dr * ct - r * st
        y_th = dr * st + r * ct
        x_thth = ddr * ct - 2 * dr * st - r * ct
        y_thth = ddr * st + 2 * dr * ct - r * st
        z = height + z_amp * s2
        z_th = z_amp * 2 * c2
        z_thth = -z_amp * 4 * s2
        pos = np.stack([r * ct, r * st, z], axis=-1)
        d1 = w * np.stack([x_th, y_th, z_th], axis=-1)
        d2 = w * w * np.stack([x_thth, y_thth, z_thth], axis=-1)
        return pos, d1, d2
    return _wrap(fn)


# --------------------------------------------------------------------------- #
# 주기 스플라인 (임의 waypoint)
# --------------------------------------------------------------------------- #
def make_periodic_spline(waypoints):
    """waypoint (M,3) 를 지나는 **주기 3차 스플라인** shape_fn.

    닫기 위해 첫 점을 끝에 복제하고 knot 을 `linspace(0,1,M+1)` 로 둔다.
    `scipy.interpolate.CubicSpline(bc_type='periodic')` 는 위치·1·2차 도함수가 연속이다.
    도함수는 파라미터(=s)에 대한 것이므로 그대로 d1/d2 로 쓴다.
    """
    from scipy.interpolate import CubicSpline

    wp = np.asarray(waypoints, dtype=float)
    if wp.ndim != 2 or wp.shape[1] != 3 or len(wp) < 3:
        raise ValueError('waypoints 는 (M>=3, 3) 이어야 한다')
    closed = np.vstack([wp, wp[:1]])            # 첫 점 복제로 폐곡선
    knots = np.linspace(0.0, 1.0, len(closed))
    cs = CubicSpline(knots, closed, bc_type='periodic', axis=0)
    d1s = cs.derivative(1)
    d2s = cs.derivative(2)

    def fn(s):
        u = np.mod(s, 1.0)
        return cs(u), d1s(u), d2s(u)
    return _wrap(fn)


def star(n=5, R_out=1.2, R_in=0.5, height=1.0):
    """n-각 별: 외곽/내곽 반지름을 교대로 놓은 2n 점을 주기 스플라인으로 잇는다."""
    ang = np.arange(2 * n) * (math.pi / n)      # 2n 개, π/n 간격
    rad = np.where(np.arange(2 * n) % 2 == 0, R_out, R_in)
    wp = np.stack([rad * np.cos(ang), rad * np.sin(ang),
                   np.full(2 * n, height)], axis=-1)
    return make_periodic_spline(wp)


# --------------------------------------------------------------------------- #
# 디스패처
# --------------------------------------------------------------------------- #
_SHAPES = {
    'circle': circle,
    'oval': oval,
    'figure8': figure8,
    'clover': clover,
    'star': star,
}


def get_shape(name, **kwargs):
    """이름으로 shape_fn 을 만든다. kwargs 는 각 도형의 파라미터로 전달."""
    if name not in _SHAPES:
        raise KeyError(f'알 수 없는 도형 {name!r}. 가능: {list(_SHAPES)}')
    return _SHAPES[name](**kwargs)


def shape_names():
    return list(_SHAPES)


def max_speed_unit(shape_fn, samples=2000):
    """period=1 기준 최대 속도 |dp/ds| — 속도 스케일링용."""
    s = np.linspace(0.0, 1.0, samples, endpoint=False)
    _, d1, _ = shape_fn(s)
    return float(np.max(np.linalg.norm(d1, axis=-1)))
