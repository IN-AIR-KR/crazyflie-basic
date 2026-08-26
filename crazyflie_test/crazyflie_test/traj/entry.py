"""도형별 비행 진입점 — 각 도형이 `flight.run(name)` 을 부르는 얇은 래퍼.

    ros2 run crazyflie_test circle  --laps 5 --speed 1.5 --yaw forward
    ros2 run crazyflie_test clover  --laps 3 --speed 2.0
    ros2 run crazyflie_test figure8 --laps 4 --speed 1.0 --yaw constant
    ros2 run crazyflie_test star    --laps 3 --speed 1.2
    ros2 run crazyflie_test oval    --laps 5 --speed 1.5
"""
from . import flight


def circle():
    flight.run('circle')


def oval():
    flight.run('oval')


def figure8():
    flight.run('figure8')


def clover():
    flight.run('clover')


def star():
    flight.run('star')
