"""정사각형 경로 비행 — mocap 좌표계 및 goto 정확도 확인용.

절대 좌표 경유점을 쓰므로 공통 절대 좌표계가 있는 mocap 또는 sim 에서 의미가 있다.

    ros2 run crazyflie_test goto_square
"""
from crazyflie_test.drone_stack import DroneStack

HEIGHT = 1.0
SIDE = 1.0          # m — 비행 공간에 맞춰 조정
LEG_DURATION = 3.0


def main():
    drone = DroneStack()

    square = [
        [SIDE, 0.0, HEIGHT],
        [SIDE, SIDE, HEIGHT],
        [0.0, SIDE, HEIGHT],
        [0.0, 0.0, HEIGHT],
    ]

    drone.takeoff(height=HEIGHT, which=0)
    drone.waypoints(square, which=0, duration=LEG_DURATION)
    drone.land(which=0)


if __name__ == '__main__':
    main()
