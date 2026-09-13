# crazyflie_fms

Crazyflie 편대를 웹 브라우저에서 모니터링하고 임무를 내리는 Fleet Management System.

## 구성

```
브라우저 (맵 UI)
   │  websocket :9090
rosbridge_server
   │  /fms/fleet_state  (편대 스냅샷, JSON)
   │  /fms/command      (버튼/클릭, JSON)
fms_node
   │  /tf, /<cf>/pose, /<cf>/status        ← 텔레메트리
   │  /<cf>/takeoff, /go_to, /land         → 명령
crazyflie_server  (backend: sim | cflib | cpp)
```

UI 정적 파일은 `web_server` 노드가 `:8080` 으로 내려준다. 프런트엔드는 **빌드 단계가
없다** — 순수 HTML/JS 에 `roslib.min.js` 를 동봉했으므로 node/npm 도, 인터넷도 필요 없다.

## 실행

```bash
# 터미널 1 — 편대 (시뮬레이션)
cd /home/kante/crazyflie
./docker/launch.sh mode:=opticalflow_multi backend:=sim

# 터미널 2 — FMS
./docker/cf_basic.sh
source install/setup.bash
ros2 launch crazyflie_fms fms.launch.py mode:=opticalflow_multi
```

브라우저에서 <http://localhost:8080> 접속. (`network_mode: host` 라 포트 매핑 없이 바로 열린다.)

`crazyflie_server` 는 이 launch 가 띄우지 않는다. 편대를 먼저 올리고 FMS 가 거기 붙는
구조라, 시뮬레이션을 재시작하지 않고 UI 만 껐다 켤 수 있다.

## 사용법

1. 사이드바에서 기체를 고른다.
2. 맵을 클릭해 목표를 찍는다 (고도는 `takeoff_height`).
3. **START** — 이륙 → 목표 이동 → 호버 유지 순으로 진행되고, 맵과 사이드바에 진행률이 뜬다.
4. **착륙** 으로 마친다. **비상 정지** 는 `/all/emergency` 로 모터를 즉시 끈다 (기체는 떨어진다).

## 임무 상태머신

`idle → takeoff → enroute → hold → landing → idle`

`enroute` 는 목표까지 남은 거리로 진행률을 낸다. goTo 명령 시간이 끝나도 기체는 아직
수렴 중이므로 `ENROUTE_GRACE` 만큼 더 기다렸다가 `arrival_radius` 안에 들어오면 도착으로
처리한다.

## 파라미터 (`fms_node`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `crazyflies_yaml_file` | launch 가 `mode` 로 결정 | 기체 목록의 출처 |
| `takeoff_height` | 1.0 | 이륙 고도 및 맵 클릭 목표의 z [m] |
| `takeoff_duration` / `land_duration` | 2.5 | [s] |
| `cruise_speed` | 0.5 | goTo 시간 산출용 [m/s] |
| `arrival_radius` | 0.15 | 도착 판정 반경 [m] |
| `publish_rate` | 10.0 | `/fms/fleet_state` 발행 주기 [Hz] |

## ⚠️ 알아둘 것

**텔레메트리 소스가 백엔드마다 다르다.** `backend:=sim` 은 `/tf` 만 쏘고 `/<cf>/pose`,
`/<cf>/status` 토픽이 **없다**. 실기(`cflib`/`cpp`)는 `firmware_logging.default_topics`
설정에 따라 pose/status 를 쏜다. `fms_node` 가 셋 다 구독해 합치므로 UI 는 같은 모양을
보지만, **sim 에서는 배터리·RSSI 가 빈 값**으로 나온다. 정상이다.

**기체 목록은 토픽 스캔이 아니라 config YAML 에서 읽는다.** 아직 연결 안 된 기체도
"연결 끊김" 상태로 UI 에 떠 있어야 편대 관리가 되기 때문이다.

**opticalflow 모드의 절대 좌표 목표.** Flow deck 은 각 기체가 자기 이륙 지점 기준으로
상대 추정을 하므로 공통 절대 좌표계가 없다. 맵 클릭이 내리는 `go_to(relative=False)` 는
**sim 에서는 정확하지만 실기에서는 기체별로 어긋난다.** 실기 편대 운용에는 mocap 모드를
쓰거나, 상대 이동(`relative=True`) 기반으로 바꿔야 한다.
