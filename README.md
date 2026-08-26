# crazyflie-basic

Crazyswarm2 기반 Crazyflie 입문용 비행 예제 레포

- 단일 / 군집 예제 모두 제공
- Flow deck(opticalflow) / mocap(Qualisys) 설정 모두 제공
- 하드웨어 없이 `backend:=sim` 으로 전 예제 실행 가능

## 폴더 구조

```
cf_ws/src/crazyflie-basic
├── crazyswarm2/               # Crazyflie ROS 2 스택 (서버·드라이버)  [서브모듈]
├── motion_capture_tracking/   # 모션 캡쳐 패키지                      [서브모듈]
└── crazyflie_test/            # 비행 예제 패키지
    ├── config/                # 기체 설정(crazyflies_*.yaml) · mocap 설정
    ├── launch/launch.py       # 서버 실행 (mode / backend 인자)
    └── crazyflie_test/
        ├── hello_world.py        # 이륙 → 호버 → 착륙
        ├── goto_square.py        # goTo 로 정사각형
        ├── multi_hello_world.py  # 군집 동시 이륙/호버/착륙
        └── traj/                 # 주기 궤적 (circle/oval/figure8/clover/star)
```

## Getting Started

### 실행 환경

- Ubuntu 22.04 + ROS 2 Humble
- Ubuntu 24.04 + ROS 2 Jazzy

### 설치

```bash
mkdir -p ~/cf_ws/src
cd ~/cf_ws/src
git clone --recursive https://github.com/IN-AIR-KR/crazyflie-basic.git
```

이미 `--recursive` 없이 클론했다면

```bash
git submodule update --init --recursive
```

### 빌드

```bash
cd ~/cf_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # 최초 1회 crazyswarm2 의존성 설치
colcon build --symlink-install
source install/setup.bash
```

### 첫 실행 (시뮬레이션)

기체 없이 시뮬레이터로 돌려보는 안전한 시작점.

```bash
# 터미널 1 — 시뮬레이션 서버
ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=sim
```

```bash
# 터미널 2 — 예제 실행 (서버는 켜둔 채 몇 번이고 다시 실행 가능)
ros2 run crazyflie_test hello_world
```

```bash
# 터미널 3 — 시각화
rviz2
```

rviz2 에서 **Global Options → Fixed Frame → `world`** 로 바꾸고 **Add → TF** 를 추가하면
기체가 움직이는 게 보인다.

## 예제

### 서버 실행

모든 예제는 먼저 서버를 띄워야 한다. 서버가 어떤 기체 설정을 쓸지는 `mode`,
실기체/시뮬 중 무엇에 연결할지는 `backend` 가 정한다.

```bash
ros2 launch crazyflie_test launch.py mode:=opticalflow                # 기본 (Flow deck 단일)
ros2 launch crazyflie_test launch.py mode:=opticalflow_multi          # Flow deck 군집
ros2 launch crazyflie_test launch.py mode:=mocap                      # mocap (QTM 세팅 필요)
ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=sim   # 하드웨어 없이 로직 검증
```

| 인자      | 기본값        | 값                                            |
| --------- | ------------- | --------------------------------------------- |
| `mode`    | `opticalflow` | `opticalflow` / `opticalflow_multi` / `mocap` |
| `backend` | `cflib`       | `cflib` / `cpp` / `sim`                       |

`mode` 는 `config/crazyflies_<mode>.yaml` 을 그대로 고른다. **실기체를 날리기 전에 그
파일의 `uri` 와 `initial_position` 을 실제 값으로 반드시 바꿀 것** (기본값은 예시다).
`mode:=mocap` 일 때만 `config/motion_capture.yaml` 의 QTM 노드가 함께 뜨므로, 그 파일의
`hostname` 도 QTM 이 도는 PC 의 IP 로 바꿔야 한다.

서버를 켠 상태에서 다른 터미널에서 실행한다.

### 기본

```bash
ros2 run crazyflie_test hello_world         # 이륙 → 5초 호버 → 착륙
ros2 run crazyflie_test goto_square         # goTo 로 한 변 1 m 정사각형
ros2 run crazyflie_test multi_hello_world   # 군집 동시 이륙/호버/착륙
```

`multi_hello_world` 는 `mode:=opticalflow_multi` 로 서버를 띄운 뒤 실행한다.
Flow deck 은 기체마다 자기 이륙 지점 기준으로 상대 추정을 해서 공통 절대 좌표계가
없다. 그래서 군집은 브로드캐스트 명령(`allcfs`)만 쓰고, 기체들을 바닥에서 서로
충분히 떨어뜨려 놓아야 한다(설정의 `initial_position` 과 같은 간격으로).

### 주기 궤적

닫힌 도형을 지정한 바퀴 수만큼 **멈추지 않고 연속으로** 돈다. 도형은 고정하고 속도만
바꾸는 방식이라(`--speed`), 같은 궤적을 느리게/빠르게 반복 비교하기 좋다.

```bash
ros2 run crazyflie_test circle  --laps 5 --speed 1.5
ros2 run crazyflie_test oval    --laps 5 --speed 1.5
ros2 run crazyflie_test figure8 --laps 4 --speed 1.0 --yaw constant
ros2 run crazyflie_test clover  --laps 3 --speed 2.0
ros2 run crazyflie_test star    --laps 3 --speed 1.2
```

| 옵션       | 기본      | 설명                                               |
| ---------- | --------- | -------------------------------------------------- |
| `--laps`   | 3         | 바퀴 수                                            |
| `--speed`  | 1.0       | 목표 최대 속도 [m/s]. **클수록 빠름**              |
| `--yaw`    | `forward` | `forward`=진행 방향으로 기수 정렬, `constant`=고정 |
| `--height` | 1.0       | 비행 고도 [m]                                      |
| `--scale`  | 1.0       | 도형 수평 크기 배율                                |
| `--rate`   | 50        | setpoint 스트리밍 주파수 [Hz]                      |
| `--ramp`   | 2.0       | 시작/종료 가감속 시간 [s]                          |

실행하면 이륙 전에 궤적 시간, 실현 최대 속도/가속도, 형상 bbox 를 출력한다.
**비행 공간에 들어가는 크기인지 이 출력으로 먼저 확인할 것.**

> ⚠️ 주기 궤적은 `cmdFullState`(low-level 제어)로 setpoint 를 직접 스트리밍한다.
> high-level commander 를 우회하므로 **상태추정이 튼튼해야 한다.** sim 또는 mocap 을
> 권장하고, Flow deck 으로 날린다면 `--speed` 를 낮게 잡을 것.

도형 정의는 [shapes.py](crazyflie_test/crazyflie_test/traj/shapes.py) 에 파라메트릭
수식으로 들어있다. 반지름·진폭 같은 도형 파라미터를 바꾸거나 새 도형을 추가하려면
이 파일을 고치면 된다.

## 비행 데이터 기록 (선택)

`firmware_logging` 이 켜져 있어 기체의 추정 pose 와 상태가 토픽으로 나온다.
비행 직전에 rosbag 을 시작한다.

```bash
ros2 bag record /cf231/pose /cf231/status -o ~/flight_logs/$(date +%Y%m%d_%H%M%S)
```

- `/cf231/pose` — 기체 온보드 추정값 (기체 이름이 다르면 토픽 이름도 바뀐다)
- `/cf231/status` — 배터리 전압·통신 상태
- `/poses` — mocap 원본(ground truth). **`mode:=mocap` 에서만 존재**하며,
  `/cf231/pose` 와 비교하면 온보드 추정 오차를 볼 수 있다

군집이라면 기체별 토픽으로 바꾼다 (`/cf1/pose /cf2/pose /cf3/pose ...`).
IMU·모터 PWM 같은 추가 로그가 필요하면 `config/crazyflies_*.yaml` 의
`firmware_logging` 아래에 `custom_topics` 를 직접 정의한다.
