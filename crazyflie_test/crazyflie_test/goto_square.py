"""정사각형 경로 비행 — mocap 좌표계 및 goTo 정확도 확인용."""
from crazyflie_py import Crazyswarm

HEIGHT = 1.0
SIDE = 1.0          # m — 비행 공간에 맞춰 조정
LEG_DURATION = 3.0
SETTLE = 1.0


def main():
    swarm = Crazyswarm()
    th = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    waypoints = [
        [SIDE, 0.0, HEIGHT],
        [SIDE, SIDE, HEIGHT],
        [0.0, SIDE, HEIGHT],
        [0.0, 0.0, HEIGHT],
    ]

    cf.takeoff(targetHeight=HEIGHT, duration=2.5)
    th.sleep(3.0)

    for wp in waypoints:
        cf.goTo(wp, yaw=0.0, duration=LEG_DURATION)
        th.sleep(LEG_DURATION + SETTLE)

    cf.land(targetHeight=0.04, duration=2.5)
    th.sleep(3.0)


if __name__ == '__main__':
    main()
