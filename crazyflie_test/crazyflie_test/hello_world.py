"""가장 기본적인 이륙/호버/착륙 테스트."""
from crazyflie_py import Crazyswarm

TAKEOFF_HEIGHT = 1.0
TAKEOFF_DURATION = 2.5
HOVER_TIME = 5.0
LAND_DURATION = 2.5


def main():
    swarm = Crazyswarm()
    th = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
    th.sleep(TAKEOFF_DURATION + 0.5)

    th.sleep(HOVER_TIME)

    cf.land(targetHeight=0.04, duration=LAND_DURATION)
    th.sleep(LAND_DURATION + 0.5)


if __name__ == '__main__':
    main()
