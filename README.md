# 자율주행 지게차 (Autonomous Forklift)

> **"A* 경로계획과 Pure Pursuit 제어를 통합한 물류 창고 자동화 시스템"**
> ROS2 기반의 실시간 경로 탐색 및 장애물 회피, 그리고 FSM(상태 기계)을 활용한 안정적인 미션 관리와 귀환 시스템을 구현한 프로젝트입니다.

![Category](https://img.shields.io/badge/Project-Robotics%20&%20Navigation-green)
![Framework](https://img.shields.io/badge/Framework-ROS2%20Humble-blue)
![Tech](https://img.shields.io/badge/Tech-Python%20|%20A*%20|%20Pure%20Pursuit-orange)

---


## 1. 프로젝트 개요

* **배경**: 물류 창고의 고위험 반복 작업(상·하차)을 자동화하여 안전사고를 예방하고 작업 효율을 극대화하기 위해 기획.
* **목적**: 맵 기반 최적 경로 생성(A*), 정밀한 경로 추종(Pure Pursuit), 돌발 장애물 대응 및 자동 귀환 시스템 구축.
* **기간**: 2026.01.23 ~ 2026.04.10 (약 3개월)
* **역할**: 하드웨어 설계 및 펌웨어 구현과 주행로직 설계 및 구현.


---


## 2. 시스템 아키텍처

* **Navigation**: A* 알고리즘(격자 기반 경로 탐색) + Pure Pursuit(경로 추종 제어).
* **Sensor Fusion**: LiDAR(LDS-01) 데이터를 6개 구역으로 분할하여 실시간 장애물 회피.
* **Management**: FSM 기반 `ReturnManager`를 통한 미션 중단 및 안전 귀환 제어.
* **Interface**: Flask 웹 서버와 MySQL을 연동한 실시간 모니터링 및 원격 관제.


---


## 3. 기술 스택

* **Software**: `ROS2 Humble`, `Python 3.10`, `Flask`, `MySQL`
* **Algorithm**: `A* Search`, `Pure Pursuit`, `AMCL (Localization)`
* **Hardware**: `TurtleBot3 Waffle Pi`, `Arduino`, `LiDAR LDS-02`


---


## 4. 주요 코드 구현 (A* & Pure Pursuit)

### **A* 안전 마진 및 패널티 로직**
벽과의 충돌을 방지하기 위해 장애물 주변에 가상의 패널티 구역을 설정하는 핵심 코드입니다.

```python
# 안전 마진 + 패널티 핵심 로직
for r in range(1, preferred_margin + 1):
    for cy, cx in [(ny+r,nx),(ny-r,nx),(ny,nx+r),(ny,nx-r)]:
        if map_data[cy][cx] > obstacle_threshold:
            if r <= safety_margin:
                too_close = True  # 경로 차단 (Hard Margin)
                break
            penalty += (preferred_margin - r) * 15  # 거리 비례 패널티 (Soft Margin)
```

Pure Pursuit 조향각 계산
Lookahead Distance를 기반으로 목표 지점을 향한 최적의 조향각을 산출하고, 측방 벽과의 거리에 따른 반발력을 추가합니다.

```Python
# 1. Pure Pursuit 기본 조향각 계산
alpha = atan2(target_y - pose_y, target_x - pose_x) - current_yaw
steering = (2.2 * sin(alpha)) / lookahead_dist

# 2. 측방 보정 (벽 반발력 추가)
if l_dist < 0.35: 
    steering -= 0.6 * (0.35 - l_dist)  # 왼쪽 벽에서 멀어지도록 보정
if r_dist < 0.35: 
    steering += 0.6 * (0.35 - r_dist)  # 오른쪽 벽에서 멀어지도록 보정
```

---


## 5. 주요 기술적 해결 (Troubleshooting)

### **a. Non-blocking 경로 계산 구조 설계**
* **문제**: A* 경로 계산(Heavy 연산) 시 메인 제어 루프가 일시 정지되어 로봇이 주행 중 멈칫거리는 현상 발생.
* **해결**: 경로 계산을 별도 **데몬 스레드**에서 비동기로 실행하고, 결과값 교체 시에만 **Lock**을 사용하는 구조를 도입하여 제어 연속성 확보.

### **b. FSM 기반 안전 귀환(Return) 시스템**
* **문제**: 미션 중단 요청 시 기존 미션 경로와 귀환 경로 데이터가 충돌하여 로봇이 경로를 이탈하는 문제.
* **해결**: `ReturnManager` FSM을 설계하여 미션 중단 시점의 데이터를 스냅샷(`deepcopy`)으로 저장하고, 원자적 교체 방식을 통해 안전하게 귀환 모드로 전환.


---


## 6. 기여도 및 성과

* **담당한 부분** : 하드웨어 제어 및 이중 안전 시스템 구축, 자율주행 알고리즘 설계, 시스템 통합 및 통신 최적화
