# 재난환경에서의 UAV/UGV 협동 미션
🚁 본 프로젝트는 **ROS2**와 **Gazebo 시뮬레이터**를 활용하여 **UAV(드론)와** **UGV(지상 차량)** 의 협력 미션을 수행한다.   

🚗 UAV와 UGV가 협력하여 무너진 건물 사이를 탐색하고 구조자(ArUco 마커)의 신원(ID), 위치(좌표)를 파악한 뒤,  
   **지정된 랑데뷰 지점**에서 다시 만나 UAV가 UGV 위에 정밀 착륙하는것이 최종 미션이다.  
<br><br>
⚖️ **평가 방식**: 정확한 좌표 저장(오차 1m 이하) / 충돌 횟수 / 정밀 착륙 정확도 / 랑데뷰 착륙 랩 타임 측정(UGV 출발 시점 ~ UAV disarm 명령 시점)  
   ※ 8개의 팀중에 가장 빨리 들어오는 팀이 우승한다  
   
<img width="979" height="549" alt="Competition Area" src="https://github.com/user-attachments/assets/905f8a3a-d6c0-44af-aacf-3bc6f496ceb6" />
[Sky View]
<br><br>
<img width="530" height="467" alt="Start Point" src="https://github.com/user-attachments/assets/c8d9b12a-115a-41ea-bb07-baeb3cf344be" />  

[Start Point]
<br><br>

 **👥 팀 구성도**  
 6인 1조로 팀이 구성되었으며, 본인은 UAV 팀에 소속되었음    
 
| 팀 영역      | 인원 | 주요 역할                                   |
| --------- | -- | --------------------------------------- |
| **UGV 팀** | 3명 | 최단거리 주행, 장애물 회피, UAV 이륙 시점 동기화, 랑데뷰 도착 |
| **UAV 팀** | 3명 | 최단거리 비행, 마커 탐색 및 좌표 기록, 정밀 착륙 제어     |

----
**✍ 시나리오 순서도**  
1. UGV가 UAV를 탑재한 채로 안정적으로 운반하여 구조 현장으로 이동  
2. 특정 위치에서 UAV는 자동 이륙하고 UGV는 장애물을 피하며 랑데뷰 포인트로 이동  
3. UAV는 폐허 지대 내 정해진 waypoint를 방문하며 구조자(0번~9번 ArUco 마커)를 탐색  
4. 마커를 발견하면 위치 및 ID를 csv에 저장  
5. 탐색이 종료되면 UAV는 랑데뷰 지점으로 복귀  
6. UAV는 최종적으로 UGV 마커 위에 정밀 착륙을 시도
----

**🧭 UAV 순차적 상태 전이**  

1. **Init (초기화 단계)**  
   - '/status/drone_ready' 토픽 발행 → UGV에게 "드론 준비 완료" 알림  
   - '/command/takeoff' 수신 시 다음 단계로 전환

2. **TakeOff (이륙)**  
   - 상승 중 특정 고도 도달 시 '/status/takeoff_complete' 발행 → UGV 출발 신호
     UAV가 Arming까지 시간이 걸려 공중에 떴을때 토픽을 발행
   - 목표 고도 도달 후 'PlanRoute'로 전환

3. **PlanRoute (경로 계획)**  
   - 'uav_path.csv', 'rendezvous.csv'를 읽어 UAV 탐색 경로 및 랑데뷰 지점 파싱  
   - 각 웨이포인트마다 탐색(ArUco 마커 검색) 행동 정의됨  
   - 파싱 완료되면 'FlyToWaypoint'로 전환

4. **FlyToWaypoint (웨이포인트 비행)**  
   - 현재 웨이포인트로 이동  
   - 도착하면 'SearchForMarker' 단계로 전환

5. **SearchForMarker (마커 탐색)**  
   - 설정된 yaw 및 gimbal pitch로 자세 조정  
   - 일정 시간 안정화 대기  
   - ArUco 마커 탐색 수행  
   - 조건 만족 시 다음 탐색 또는 다음 웨이포인트로 전환

6. **Rendezvous (랑데뷰)**  
   - UGV가 '/command/rendezvous'를 발행하면 시작됨
     UGV가 UAV보다 미리 도착해 있는 대기 상태임  
   - UAV는 랑데뷰 지점 상공으로 이동 후 안정화  
   - ArUco 마커를 탐지하여 상대 좌표 계산  
   - 반복적으로 마커 중심에 접근하며 수직 하강  
   - 고도 임계점 도달 시 정지 상태 판단 후 착륙 완료 처리

7. **Completed (미션 완료)**  
   - 착륙 후 'Disarm' 명령 발행  
   - 전체 미션 소요 시간 로그 출력 및 종료

<img src="https://github.com/user-attachments/assets/ee56b8d0-0f0c-402f-ba21-b1a9c7ab5d62" alt="RDV Point" width="600" height="500"/>  

[Rendezvous Point]

----

**✅ UAV 최적화 실험**  
- UAV의 탐색 경로를 구성하기 위해 utilities_pkg의 keyboard_control 노드를 이용한 수동 비행 활용  
  RViz 상에서 실시간으로 마커 인식 위치를 시각적으로 확인하며, 마커가 인식된 지점에 웨이포인트(Waypoint)를 직접 지정하고,  
  이후 각 지점을 연결하여 최단 경로(route) 실험을 통해 최적의 경로를 구성  
  
- 랩타임 단축을 위해 랑데뷰 착륙 중 UGV 상단 중심에 도달했을 때, 안전 착륙 절차를 생략하고 곧바로 Disarm 명령을 공중에서 수행  
   
<br><br>  
**📌 실험 중 확인된 문제점 및 개선 사항**  
- 1m 이상 오차 발생 문제  
  2개의 마커를 한번에 인식할 경우 마커와 UAV 간의 거리가 너무 멀어져, 좌표 오차가 1m 이상 발생  
→ 따라서 마커 중심을 안정적으로 인식하기 위해 최적의 waypoint 거리(적정 근접 거리)를 도출  

- 짐벌 회전 지연 문제  
  탐색 도중 UAV가 도착 후 짐벌을 회전시키는 방식은 회전 지연으로 인해 마커 탐색이 비효율적  
→ 이를 개선하기 위해 이동 중 UAV 자세 및 짐벌 pitch 각도를 미리 설정하는 동작을 추가하여  
  도착 시점에 바로 탐색이 가능한 상태로 준비되도록 최적화  

- 랑데뷰 중 불안정한 착륙 문제  
  랑데뷰 지점에서 안정화 시간 없이 즉시 착륙을 시도해 드론이 UGV 중심이 아닌 가장자리나 바깥으로 떨어지는 문제가 발생  
  → 이를 해결하기 위해 랑데뷰 과정에 "반복 하강 기반의 단계적 착륙 방식"을 도입  
  → UAV는 일정 고도에서 **안정화 대기 → 마커 중심 정렬 → 하강 → 재안정화 → 마커 중심 정렬 → 안정화 후 착륙** 단계를 반복하며  
    중심에 가까워질 때까지 하강을 수행


UGV는 UAV보다 빠르게 도착하므로 별도의 정밀 안정화 절차 없이 waypoint 기반 주행만으로 랑데뷰 지점에 도달  


---------
총 2회의 시도 기회가 주어졌기 때문에, 파라미터를 YAML 파일로 분리하여 유지보수 설계함  
## 🚀 UAV 실행 모드: Stable vs Fast  

UAV는 'uav.launch.py' 파일을 통해 실행되며, 런치 시 'mode' 인자를 이용해 두 가지 설정 모드 중 하나를 선택:  

- **Stable 모드 (`mode:=stable`)**: 완주 중심의 안전한 설정값 사용    
- **Fast 모드 (`mode:=fast`)**: 랩타임 단축을 목표로 최적화된 튜닝 모드 적용  

런치 파일 내부에서 해당 `mode` 값에 따라 아래의 YAML 파일을 자동 선택하여 UAV 노드에 주입합니다:  

| 모드 | 적용 파라미터 파일 |
|------|-------------------|
| `stable` | `config/stable_params.yaml` |
| `fast`   | `config/fast_params.yaml`   |

실험 반복이나 랩타임 튜닝 과정에서 빠르게 수치 수정 및 테스트가 가능

----
**Competition**


https://github.com/user-attachments/assets/3a64362f-4599-4f32-9e56-45601fbf5ad1





