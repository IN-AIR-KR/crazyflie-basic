"""비행 시나리오 — 관제 UI 가 찍어준 목표로 편대를 보내는 실 로직.

역할 분담:
    drone_stack.py     제어 출력 — 이륙/이동/착륙 명령을 내보낸다
    drone_callback.py  입력 수신 — UI 목표, 위치, 상태를 받아온다
    drone_main.py      이 파일 — 무엇을 어떤 순서로 할지 (실 로직)

흐름:
    1. 관제 UI(http://localhost:8080)에서 기체를 고르고 맵을 클릭해 목표를 찍는다
    2. START 를 누른다
    3. 이 스크립트가 이륙 → 각 기체를 자기 목표로 → 도착 확인 → 호버 → 착륙

    ros2 run crazyflie_test drone_main

⚠️ crazyflie_fms 의 fms_node 와 동시에 띄우지 말 것. 같은 /fms/command 를 둘 다 듣고
   있어서 한 기체에 두 곳이 명령을 내린다. UI 만 띄우려면 rosbridge 와 web_server 만
   돌리거나, DroneCallback(command_topic=...) 으로 토픽을 갈라 줄 것.
"""
from crazyflie_test.drone_callback import DroneCallback
from crazyflie_test.drone_stack import DroneStack

TAKEOFF_HEIGHT = 1.0
HOVER_AFTER_ARRIVAL = 3.0
GOTO_DURATION = 4.0
ARRIVAL_TIMEOUT = 30.0


def main():
    drone = DroneStack()
    ui = DroneCallback(drone)

    print(f'[drone_main] {drone.count()} 기체: {", ".join(drone.names())}')

    # 사건이 생길 때마다 로그로 흘려둔다. 실제 대응은 아래 순차 로직이 한다.
    ui.on_goal(lambda name, goal:
               print(f'  [목표] {name} → {[round(v, 2) for v in goal]}'))
    ui.on_arrived(lambda name, dist:
                  print(f'  [도착] {name} (오차 {dist:.2f} m)'))
    ui.on_low_battery(lambda name, volts:
                      print(f'  [경고] {name} 배터리 {volts:.2f} V'))
    ui.on_disconnect(lambda name:
                     print(f'  [경고] {name} 연결 끊김'))

    # 1) UI 에서 목표를 찍어줄 때까지 기다린다.
    print('[drone_main] 관제 UI 에서 목표를 찍어주세요...')
    if ui.wait_for_goal() is None:
        print('[drone_main] 목표를 못 받고 종료')
        return

    # 2) START 를 누를 때까지 기다린다. 그 사이 목표를 더 찍거나 바꿔도 된다.
    print('[drone_main] 목표 접수. START 를 누르세요...')
    if not ui.wait_for_start():
        print('[drone_main] START 없이 종료')
        return

    goals = ui.pending_goals()
    print(f'[drone_main] 임무 시작 — {len(goals)} 기체')

    # 3) 전 기체 동시 이륙 (브로드캐스트라 타이밍이 어긋나지 않는다)
    drone.takeoff(height=TAKEOFF_HEIGHT)

    # 4) 각 기체를 자기 목표로. wait=False 로 명령만 뿌려 동시에 출발시킨다.
    for name, goal in goals.items():
        drone.goto(goal, which=name, duration=GOTO_DURATION, wait=False)

    # 5) 명령 시간이 아니라 **실제 위치**를 보고 도착을 판정한다.
    if ui.wait_until_all_arrived(timeout=ARRIVAL_TIMEOUT):
        print('[drone_main] 전 기체 도착')
    else:
        print('[drone_main] 도착 대기 시간 초과 — 그대로 진행')

    # 6) 잠시 호버 후 착륙
    drone.hover(HOVER_AFTER_ARRIVAL)
    drone.land()

    ui.reset_start()
    print('[drone_main] 임무 종료')


if __name__ == '__main__':
    main()
