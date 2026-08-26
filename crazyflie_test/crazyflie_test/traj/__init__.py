"""주기(periodic) 궤적 — 도형 정의 · 연속-랩 비행.

주기성 · 속도 격리(도형은 고정하고 속도만 조절) · yaw 모드를 원칙으로, 도형을 코드로
직접 정의한다. 깔끔한 도형(circle/oval/figure8/clover)은 파라메트릭 수식과 해석적
도함수로, 임의 도형(star)은 주기 스플라인으로 만든다.

    shapes.py  도형 정의 (pos, dp/ds, d²p/ds²)
    flight.py  속도 스케줄 + cmdFullState 스트리밍 러너
    entry.py   도형별 ros2 run 진입점
"""
