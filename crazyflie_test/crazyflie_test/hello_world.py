"""가장 기본적인 이륙/호버/착륙 테스트 — 기체 한 대만 움직인다.

전 기체를 동시에 띄우려면 multi_hello_world 를 쓸 것.

    ros2 run crazyflie_test hello_world
"""
from crazyflie_test.drone_stack import DroneStack

TAKEOFF_HEIGHT = 1.0
HOVER_TIME = 5.0


def main():
    drone = DroneStack()

    # which=0 — 첫 번째 기체 하나에만 명령이 간다.
    drone.takeoff(height=TAKEOFF_HEIGHT, which=0)
    drone.hover(HOVER_TIME)
    drone.land(which=0)


if __name__ == '__main__':
    main()
