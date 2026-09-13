# STM32 CAN · Ethernet · FreeRTOS 기반 Zonal E/E Architecture

> STM32 다중 노드와 Jetson을 연계하여 차량의 Zonal E/E Architecture를 축소 구현한 프로젝트입니다.  
> CAN 기반 노드 통신, Ethernet/UDP 기반 상위 시스템 연동, FreeRTOS 실시간 제어, Linux Character Device Driver, DTC 진단, GUI 모니터링, 4-Layer PCB 설계·제작까지 전체 시스템을 직접 구현하고 검증했습니다.

---

## 1. Project Overview

기존의 기능 중심 MCU 프로젝트에서 확장하여, 여러 제어기가 통신을 통해 협력하는 차량 분산 제어 구조를 구현하는 것을 목표로 진행했습니다.

각 Zone 내부에서는 **CAN**을 이용해 센서·제어·상태 정보를 교환하고, Zone Controller에서 수집한 정보는 **Ethernet/UDP**를 통해 상위 시스템인 Jetson으로 전달하도록 구성했습니다.

또한 시스템 규모가 커지면서 발생하는 통신 오류, 실시간성 문제, 고장 진단 문제를 직접 분석하고 검증했습니다.

### 주요 구현 내용

- STM32 기반 다중 노드 Zonal Architecture 구성
- CAN 기반 Zone 내부 통신
- Ethernet/UDP 기반 STM32 ↔ Jetson 연동
- V-Cycle 관점의 요구사항 및 검증 기준 정의
- CAN ID / Data Structure 기반 ICD 작성
- FreeRTOS 기반 실시간 Task 구조 구현
- Linux Character Device Driver 구현
- DTC 기반 고장 진단 시스템
- GUI 기반 시스템 상태 모니터링
- CAN/Ethernet End-to-End Latency 계측
- KiCad 기반 4-Layer PCB 설계 및 실제 제작
- 반복시험 및 자동화된 검증 환경 구성

---

## 2. System Architecture

```text
                  ┌────────────────────────────┐
                  │       Jetson / Linux       │
                  │                            │
                  │  Character Device Driver   │
                  │  DTC / GUI / Monitoring    │
                  └─────────────┬──────────────┘
                                │
                         Ethernet / UDP
                                │
            ┌───────────────────┴───────────────────┐
            │                                       │
┌───────────▼───────────┐               ┌───────────▼───────────┐
│ Front Zone Controller │               │ Rear Zone Controller  │
│     STM32H723         │               │      STM32H723        │
└───────────┬───────────┘               └───────┬─────────┬─────┘
            │ CAN                               │ CAN     │ CAN
            │                                   │         │
     ┌──────▼──────┐                     ┌──────▼───┐ ┌──▼──────────┐
     │ Front Node  │                     │Rear Node │ │Rear Body    │
     │ STM32F407   │                     │STM32F446 │ │STM32F407   │
     └─────────────┘                     └──────────┘ └─────────────┘
```

### 통신 구조
- Zone 내부의 MCU 간 상태 및 제어 데이터 교환
- 센서 데이터 전달
- DTC 및 성능 계측 데이터 전달
- GUI/HMI와 임베디드 제어 시스템 연동

---

## 3. Repository Structure

```text
STM32_CAN_RTOS_ETHERNET/
│
├── Node_F407_Front/
│   └── Front Zone I/O Node
│
├── Node_F407_Rear/
│   └── Rear Body Node
│
├── Node_F447_Rear/
│   └── Rear Zone Sensor Node
│       ※ 실제 MCU는 STM32F446 계열
│
├── ZoneController_Front_H723/
│   └── Front Zone Controller
│
├── ZoneController_Rear_H723_real/
│   └── Rear Zone Controller
│
├── jetson_device_driver/
│   └── Linux Character Device Driver
│
├── jetson_device_driver_lcd/
│   └── I2C LCD Character Device Driver
│
├── rear_node_board/
│   └── KiCad 4-Layer PCB 설계 및 제조 데이터
│
├── DEBUG_JOURNAL_최종/
│   └── Debug / Measurement / Verification Records
│
└── README.md
```

---

# 4. CAN Communication

- 노드별 송수신 역할 정의
- Bit Timing 설정
- CAN Filter 구성
- CAN 상태 및 오류 계측
- 정상/이상 네트워크 비교 검증

---

## 4.1 CAN Frame Loss 47% → 0%

시스템 검증 과정에서 특정 500 kbps CAN Bus에서 수신 프레임의 약 47%가 유실되는 문제가 발생했습니다.

단순히 코드를 수정하기보다 정상 동작하는 CAN 네트워크와 문제가 발생하는 네트워크의 설정을 비교했습니다.

그 결과 두 노드의 **Bit Timing과 Sample Point가 서로 다르게 설정되어 있음**을 확인했습니다.

설정을 정합한 뒤 다시 시험한 결과 프레임 유실률을 0%까지 개선했습니다.

```text
Before : Frame Loss 47 %
After  : Frame Loss 0 %
```

이 경험을 통해 통신 오류 발생 시 정상 시스템을 대조군으로 두고 설정과 데이터를 단계적으로 비교하는 방식의 중요성을 확인했습니다.

---

# 5. Ethernet / UDP Communication

각 Zone에서 수집한 데이터를 상위 시스템인 Jetson으로 전달하기 위해 Ethernet을 사용했습니다.

### 수행 내용

- STM32H723 Ethernet 설정
- lwIP 기반 UDP 통신
- Zone Controller → Jetson Telemetry 전달
- Jetson → Zone Controller Command 전달
- Zone 간 정보 전달
- DTC 및 성능 계측 데이터 전송

CAN은 **Zone 내부의 실시간 데이터 교환**, Ethernet은 **상위 시스템과의 데이터 연동**에 사용하도록 역할을 분리했습니다.

---

# 6. End-to-End Latency Optimization

센서 입력부터 Jetson 상위 시스템까지 데이터가 전달되는 전체 경로를 측정한 결과 최대 **150.6 ms**의 지연이 발생했습니다.

처음에는 개별 코드의 연산시간을 의심했지만, 구간별 시간을 직접 측정한 결과 기존의 **주기 처리 방식**에서 데이터가 준비된 뒤에도 다음 처리 시점까지 대기하는 시간이 반복되고 있음을 확인했습니다.

이에 상행/하행 데이터 처리를 필요한 이벤트가 발생했을 때 수행하도록 **Event Trigger 방식**으로 변경했습니다.

| 항목 | 개선 전 | 개선 후 |
|---|---:|---:|
| 최대 End-to-End Latency | 150.6 ms | 27.5 ms |
| 평균 End-to-End Latency | 65.5 ms | 6.0 ms |
| 최대 지연 개선 | - | **81.7% 감소** |

단순히 코드를 빠르게 만드는 것이 아니라 전체 데이터 흐름을 측정하고 병목 구조를 변경하는 방식으로 문제를 해결했습니다.

---

# 7. FreeRTOS

STM32 Zone Controller의 기능을 Task 단위로 분리하고 실시간 시스템의 동작을 검증하기 위해 FreeRTOS를 적용했습니다.

### 주요 내용

- Task Priority 설계
- Semaphore
- Mutex
- Task Notification
- 공유 자원 동기화
- Stack Usage 계측
- Priority Inversion 검증
- Priority Inheritance 적용

---

## 7.1 Priority Inversion

공유 자원에 접근하는 여러 Task가 동시에 동작할 때 높은 우선순위 Task가 즉시 수행되지 못하는 Priority Inversion 상황을 구성하고 직접 계측했습니다.

Binary Semaphore와 Mutex 사용 시의 차이를 비교했습니다.

| 방식 | 최대 지연 |
|---|---:|
| Semaphore / Priority Inheritance 없음 | 7,096 μs |
| Mutex / Priority Inheritance 적용 | 2,068 μs |

Priority Inheritance를 적용해 지연시간을 약 **3.4배 감소**시켰습니다.

이를 통해 RTOS에서 Task Priority만 설정하는 것뿐 아니라 공유 자원의 동기화 방식까지 실시간 성능에 영향을 준다는 점을 확인했습니다.

---

# 8. FreeRTOS Debugging

FreeRTOS 전환 과정에서는 단순 HardFault가 아닌 Kernel 정지 현상도 경험했습니다.

초기에는 assert가 발생한 위치를 각각 수정했지만 문제 발생 위치가 계속 바뀌었습니다.

이를 메모리 오염 가능성으로 판단하고 Stack Overflow Hook과 High Water Mark를 추가해 Task별 Stack 사용량을 측정했습니다.

그 결과 Ethernet Interface Thread의 Stack이 실제 필요량보다 작게 설정되어 있었음을 확인했습니다.

### 원인

CMSIS-RTOS v2의 `stack_size` 단위를 잘못 해석해 Stack 공간이 부족하게 설정되어 있었습니다.

Stack 크기를 조정한 뒤 반복 시험을 통해 Kernel 정지와 Stack Overflow가 발생하지 않음을 확인했습니다.

이 과정은 `DEBUG_JOURNAL_최종/`에 기록되어 있습니다.

---

# 9. DTC Diagnostic System

시스템 이상이 발생할 때마다 여러 MCU와 통신 로그를 각각 확인해야 해 원인을 파악하는 데 시간이 걸렸습니다.

이를 개선하기 위해 각 노드의 고장 상태를 **DTC(Diagnostic Trouble Code)** 형태로 코드화했습니다.

### 구현 내용

- Node Fault 상태 정의
- DTC Code 정의
- Pending / Confirmed 상태 관리
- CAN Fault Monitoring
- 통신 Timeout Detection
- Ethernet을 통한 DTC 전달
- Jetson GUI에서 Fault 상태 표시
- 반복시험을 통한 진단 결과 검증

---

## 9.1 CAN Link Failure Detection

초기에는 CAN Bus-Off 상태만으로 통신 이상을 판정했습니다.

하지만 실제 CAN Link가 끊겨도 Bus-Off에 진입하지 않아 고장을 감지하지 못하는 경우가 발생했습니다.

이에 Bus-Off뿐 아니라 **수신 데이터 Timeout**을 함께 진단 조건으로 사용하도록 수정했습니다.

```text
CAN Link Failure Detection

Before : 일부 링크 단절 검출 불가
After  : 약 500 ms 이내 검출
```

단순 오류 Flag에만 의존하지 않고 실제 통신 상태까지 함께 확인하도록 진단 기준을 변경했습니다.

---

# 10. GUI / HMI Monitoring

Jetson에서 각 Zone의 상태를 통합해 확인할 수 있도록 GUI 기반 HMI를 구현했습니다.

### 확인 정보

- Zone 상태
- CAN 상태
- 센서 데이터
- DTC
- 통신 상태
- Fault 위치
- System Telemetry

이를 통해 고장 발생 시 여러 Node의 로그를 각각 확인하기 전에 GUI에서 먼저 이상 위치와 상태를 파악할 수 있도록 구성했습니다.

반복적인 Closed-Loop 시험을 통해 HMI와 실제 시스템 동작을 검증했습니다.

---

# 11. Linux Character Device Driver

Jetson Linux 환경에서 Kernel Space와 User Space 간 HW/SW Interface를 직접 구현하기 위해 Character Device Driver를 작성했습니다.

---

## 11.1 I2C LCD Driver

PCF8574 기반 I2C LCD를 제어하는 Linux Character Device Driver를 구현했습니다.

### 구현 내용

- `i2c_driver`
- `probe()` / `remove()`
- Character Device 등록
- `/dev` Interface 제공
- `open`
- `write`
- `ioctl`
- Mutex 기반 동기화
- LCD Command 처리

User Space Application이 `/dev`를 통해 LCD 기능을 사용할 수 있도록 구성했습니다.

---

## 11.2 UDP Status Driver

Zone Controller에서 전달되는 UDP Telemetry를 Linux Kernel에서 수신하고 `/dev` Interface를 통해 User Space에 제공하는 Driver를 구현했습니다.

### 구현 내용

- Kernel UDP Socket
- Kernel Thread
- Blocking Read
- Wait Queue
- Spinlock
- `poll()`
- Packet Validation
- Sequence Monitoring

초기에는 SocketCAN Network Device 형태로 구현하려 했지만, CAN Subsystem이 요구하는 초기화 조건을 충족하지 못해 Kernel이 정지하는 문제가 발생했습니다.

Kernel UDP Socket과 Thread는 정상 동작하고 있음을 확인한 뒤 Network Device 등록 계층을 분리해 문제를 좁혔고, Character Device 방식으로 변경해 해결했습니다.

---

# 12. PCB Design & Manufacturing

초기 시스템은 Breadboard와 Jumper Wire를 이용해 구성했습니다.

프로젝트가 커지면서 다음 문제가 반복적으로 발생했습니다.

- 접촉 불량
- 배선 복잡도 증가
- CAN 배선 오류 가능성
- 재현성이 낮은 HW 구성

이를 개선하기 위해 KiCad를 이용해 **4-Layer Custom PCB를 직접 설계하고 실제 제작업체에 제조를 의뢰했습니다.**

### 수행 과정

1. 기존 배선 및 Pin Map 정리
2. 회로도 작성
3. CAN Transceiver 회로 구성
4. CAN Termination 설계
5. Sensor / Actuator Interface 설계
6. Power / Signal Net 정의
7. Footprint 배치
8. 4-Layer Stack 구성
9. Routing
10. ERC / DRC 검증
11. Gerber 및 Drill File 생성
12. 외부 PCB 제조 의뢰
13. 실제 보드 제작 및 시스템 적용

---

## 12.1 CAN Hardware Design

CAN 통신부에는 SN65HVD230 기반 CAN Transceiver와 Split Termination 구조를 적용했습니다.

Rear CAN Network에서는 3개의 Node가 연결되므로 버스의 양 끝 Node에만 Termination을 적용할 수 있도록 Jumper 구조를 구성했습니다.

센서, LED, Switch 및 기타 I/O 회로도 PCB에 통합했습니다.

PCB 관련 설계 파일 및 제조 데이터는 아래 폴더에 있습니다.

```text
rear_node_board/
```

---

# 13. V-Cycle & ICD

프로젝트 규모가 커지면서 여러 Node의 기능과 통신 기준을 기억에 의존해 관리하기 어려워졌습니다.

이에 V-Cycle 관점에서 요구사항과 검증 기준을 정리했습니다.

### 수행 내용

- System Requirement 정의
- Functional Requirement 분해
- 검증 기준 작성
- CAN ID 정의
- CAN Data Structure 정의
- UDP Packet 정의
- Interface Control Document 작성
- Test Case 구성
- 요구사항 ↔ 검증 항목 연결

기능 구현 이후 시험 방법을 생각하는 것이 아니라 요구사항 정의 단계에서 검증 방법까지 함께 고려하도록 개발 방식을 정리했습니다.

---

# 14. Debugging & Verification

프로젝트 과정에서 발생한 문제와 분석 과정을 별도의 Debug Journal로 관리했습니다.

```text
DEBUG_JOURNAL_최종/
```

### 주요 Debugging 사례

- CAN Frame Loss
- CAN Bit Timing / Sample Point 문제
- Ethernet Power-On Failure
- FreeRTOS Stack Overflow
- Priority Inversion
- DTC False Detection
- CAN Link Failure Detection
- UDP Telemetry 검증
- Network IP / Subnet 문제
- Sensor Data 이상
- Closed-Loop System Verification

단순히 해결 결과만 기록하지 않고 아래와 같은 형태로 관리했습니다.

```text
Problem
  ↓
Hypothesis
  ↓
Measurement
  ↓
Root Cause
  ↓
Modification
  ↓
Regression Test
```

---

# 15. Measurement & Verification

시스템 동작을 감으로 판단하지 않고 가능한 항목은 실제 데이터를 측정해 검증했습니다.

### 대표 검증 항목

| 항목 | 결과 |
|---|---:|
| CAN Frame Loss | 47% → 0% |
| End-to-End Maximum Latency | 150.6 ms → 27.5 ms |
| End-to-End Average Latency | 65.5 ms → 6.0 ms |
| Priority Inversion Delay | 7,096 μs → 2,068 μs |
| CAN Link Failure Detection | 검출 불가 → 약 500 ms |
| DTC 반복 검출 편차 | 약 1.5 ms 수준 |
| HMI Closed-Loop Test | 8 / 8 Pass |

---

# 16. Tech Stack

### Embedded

- STM32H723
- STM32F407
- STM32F446
- C
- STM32CubeIDE
- STM32 HAL
- FreeRTOS
- CMSIS-RTOS2

### Communication

- CAN
- FDCAN
- Ethernet
- UDP
- I2C
- RS485

### Linux

- Jetson
- Linux Kernel Module
- Character Device Driver
- Kernel UDP Socket
- Kernel Thread
- `/dev` Interface

### Hardware

- KiCad
- 4-Layer PCB
- CAN Transceiver
- Gerber
- BOM
- PCB Manufacturing

### System Engineering

- V-Cycle
- ICD
- DTC
- Test Case
- System Integration
- Verification
- Debugging

---

# 17. Key Takeaways

이 프로젝트에서는 단순히 MCU의 기능을 구현하는 것을 넘어 여러 장치가 연결된 시스템 전체를 설계하고 검증했습니다.

특히 다음을 직접 경험했습니다.

- CAN / Ethernet 기반 분산 통신 구조 설계
- 통신 오류의 Root Cause 분석
- FreeRTOS 기반 실시간 시스템 설계
- Task Synchronization 및 Priority Inversion 검증
- Linux Kernel Device Driver 구현
- DTC 기반 고장 진단
- GUI 기반 상태 모니터링
- V-Cycle과 ICD를 활용한 Interface 관리
- 4-Layer PCB 설계 및 실제 제작
- HW / FW / Linux를 포함한 전체 시스템 통합
- 측정 데이터를 기반으로 한 성능 개선 및 Regression Test

기능이 한 번 동작하는 것보다 **문제가 발생했을 때 원인을 추적할 수 있고, 수정한 결과를 다시 검증할 수 있는 시스템을 만드는 것**을 목표로 프로젝트를 진행했습니다.

---

## Author

**Lee Seungmin**

- Kyungpook National University
- Department of Electronics Engineering
- CAN ID 대역 정의
- 메시지별 Data Structure 정의
각 Zone 내부의 MCU 간 통신은 CAN을 기반으로 구성했습니다.

### 수행 내용
- 액추에이터 명령 전달
- 고장 및 상태 정보 전달

**Ethernet / UDP**
- 각 Zone Controller에서 수집한 데이터를 Jetson으로 전달
- 상위 시스템 명령 전달

CAN과 Ethernet은 서로 다른 역할로 사용했습니다.
**CAN**

