"""hello_world 의 군집 버전 — 전 기체 동시 이륙/호버/착륙.

기체 하나를 집어 쓰는 hello_world 와 달리 브로드캐스트 명령을 보낸다. 한 번의 무선
패킷이 모든 기체에 동시에 도착하므로 기체 수가 늘어도 이륙/착륙 타이밍이 어긋나지 않는다.

⚠️ Flow deck(opticalflow_multi) 은 각 기체가 자기 이륙 지점 기준으로 상대 추정을 해서
   공통 절대 좌표계가 없다. 그래서 절대 좌표 goto 대신 브로드캐스트만 쓰고, 기체들은
   바닥에서 서로 충분히 떨어뜨려 배치한다(config 의 initial_position 과 같게).

    ros2 launch crazyflie_test launch.py mode:=opticalflow_multi backend:=sim
    ros2 run crazyflie_test multi_hello_world
"""
from crazyflie_test.drone_stack import DroneStack

TAKEOFF_HEIGHT = 1.0
HOVER_TIME = 5.0


def main():
    drone = DroneStack()
    print(f'[multi_hello_world] {drone.count()} 기체 동시 비행')

    # which 를 생략하면 전 기체 브로드캐스트.
    drone.takeoff(height=TAKEOFF_HEIGHT)
    drone.hover(HOVER_TIME)
    drone.land()


if __name__ == '__main__':
    main()
