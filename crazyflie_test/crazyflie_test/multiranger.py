"""멀티레인저(전후좌우 거리 센서) 전용 함수 — 읽기 + 회피 속도 자동 계산.

crazyflie_py 에는 멀티레인저를 감싸는 함수가 없다(get_pose()/get_status() 같은
건 있는데 scan 은 없음) — 그래서 avoid.py 가 직접 토픽을 구독하던 걸 여기로
빼냈다. get_pose()/get_status() 와 같은 "구독해서 최신값 캐싱" 패턴을 그대로
따른다.

    sensor = RangeSensor(cf)           # <prefix>/scan 구독 시작
    ...
    direction = sensor.avoid_direction(safe_dist=0.2)
    # → 가까운 방향이 있으면 그 반대쪽 단위벡터(vx,vy), 없으면 None
"""
import math

# 센서 이름 → world 좌표계 단위 벡터(그 센서가 보는 방향). x=앞, y=왼쪽 (ENU).
# yaw 고정(돌지 않음)을 전제로 body frame == world frame 으로 취급한다.
SENSOR_DIRS = {'front': (1.0, 0.0), 'back': (-1.0, 0.0),
               'left': (0.0, 1.0), 'right': (0.0, -1.0)}


def avoid_direction(ranges, safe_dist):
    """ranges: {'front':d, 'back':d, 'left':d, 'right':d} (m, inf=장애물 없음).

    safe_dist 보다 가까운 방향이 있으면 그 반대쪽 단위벡터(vx,vy)를 돌려준다
    (여러 방향에서 동시에 가까우면 합쳐서 정규화). 가까운 게 하나도 없거나,
    정반대 방향에서 동시에 감지돼 방향이 상쇄되면 None(제자리 유지).
    ROS 없이도 테스트 가능한 순수 함수(avoid.py --selftest 가 직접 검증한다)."""
    vx = vy = 0.0
    triggered = False
    for name, dist in ranges.items():
        if dist is None or math.isinf(dist) or dist >= safe_dist:
            continue
        triggered = True
        ux, uy = SENSOR_DIRS[name]
        vx -= ux  # 센서가 보는 방향의 반대로
        vy -= uy
    if not triggered:
        return None
    mag = math.hypot(vx, vy)
    if mag < 1e-9:
        return None
    return vx / mag, vy / mag


class RangeSensor:
    """cf 의 <prefix>/scan 을 구독해 전후좌우 거리를 들고 있다가, 요청하면
    "가까운 방향 있으면 그 반대로 밀어내는 속도"까지 바로 계산해준다."""

    def __init__(self, cf):
        from sensor_msgs.msg import LaserScan  # ROS 없이 import 가능하게 지연 로드
        self.ranges = {'front': math.inf, 'back': math.inf,
                        'left': math.inf, 'right': math.inf}
        cf.node.create_subscription(LaserScan, cf.prefix + '/scan', self._on_scan, 10)

    def _on_scan(self, msg):
        # 서버가 고정하는 순서: [back, right, front, left] (crazyflie_server_py 기준)
        if len(msg.ranges) >= 4:
            (self.ranges['back'], self.ranges['right'],
             self.ranges['front'], self.ranges['left']) = msg.ranges[:4]

    def avoid_direction(self, safe_dist):
        """지금 들어온 거리값 기준 회피 방향 단위벡터 (vx, vy) — world 좌표계.
        가까운 게 없으면 None."""
        return avoid_direction(self.ranges, safe_dist)
