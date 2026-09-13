# Zone Controller — CAN ICD

> 소스에서 직접 추출한 실측 문서. 추정 없음.
> 기준 커밋 시점: 2026-08-01

---

## 0. 버스 구성

| 버스 | 물리 | 속도 | 노드 |
|---|---|---|---|
| **FRONT-CAN** | Front H723 `FDCAN2` ↔ F407_Front `CAN1` | 250 kbps | Front H723 (게이트웨이), F407_Front (I/O) |
| **REAR-CAN250** | Rear H723 `FDCAN2` ↔ F446RE `CAN1` | 250 kbps | Rear H723 (게이트웨이), Node_F446 |
| **REAR-CAN500** | Rear H723 `FDCAN1` ↔ F407G `CAN1` | 500 kbps | Rear H723 (게이트웨이), F407G (에코 노드) |

전 프레임 **Classical CAN / Standard ID(11bit) / BRS off**.

### 비트 타이밍 (실측 일치 확인)

| 노드 | 클럭 | Prescaler | Seg1/Seg2 | tq | 결과 | 샘플점 |
|---|---|---|---|---|---|---|
| H723 FDCAN | 48 MHz | 12 | 13/2 | 16 | 250 kbps | 87.5 % |
| F407 / F446 | 42 MHz (APB1) | 12 | 11/2 | 14 | 250 kbps | 85.7 % |

샘플점 1.8 % 차는 허용 범위. 현재 통신 이상 없음.

---

## 1. ID 대역 배정 규칙

```
0x1xx   H723 → 하위노드   링크 확인 (ping)
0x18x   하위노드 → H723   링크 확인 응답 (echo)      [CAN500]
0x2xx   H723 → 하위노드   링크 확인 (ping)           [CAN250]
0x21x   F407_Front → H723 상태 신호
0x28x   하위노드 → H723   링크 확인 응답 (echo)
0x3xx   H723 → 하위노드   제어 명령 / 노드 → H723 센서
```

**설계 원칙: 신호 1개 = CAN ID 1개.** 여러 신호를 한 프레임에 묶지 않는다.
사유 — RTOS Task 분리, 유지보수, 디버깅 용이.

---

## 2. FRONT-CAN (250 kbps)

### 2.1 링크 확인

| ID | 방향 | DLC | 페이로드 | 주기 |
|---|---|---|---|---|
| `0x201` | H723 → F407 | 8 | `A5 5A 02 01` + counter(u32 LE) | 500 ms |
| `0x281` | F407 → H723 | 8 | `5A A5 02 02` + 수신 counter 그대로 | 0x201 수신 시 |

F407은 `data[0..3] == A5 5A 02 01` 일치할 때만 응답.

### 2.2 F407_Front → H723 (상태)

| ID | 신호 | DLC | 인코딩 | 주기 |
|---|---|---|---|---|
| `0x210` | Steering ADC | 2 | u16 LE (0–4095) | 20 ms |
| `0x211` | Brake ADC | 2 | u16 LE (0–4095) | 20 ms |
| `0x212` | Accelerator ADC | 2 | u16 LE (0–4095) | 20 ms |
| `0x213` | 좌 방향지시등 출력 | 1 | u8 0/1 | 50 ms |
| `0x214` | 우 방향지시등 출력 | 1 | u8 0/1 | 50 ms |
| `0x215` | 방향지시 스위치 눌림 | 1 | u8 0/1 | 50 ms |
| `0x216` | 전조등 출력 | 1 | u8 0/1 | 50 ms |
| `0x217` | 온도 ×10 | 2 | **i16** LE (℃×10) | 1000 ms |
| `0x218` | 습도 ×10 | 2 | u16 LE (%×10) | 1000 ms |
| `0x219` | DHT11 유효 | 1 | u8 0/1 | 1000 ms |
| `0x21A` | 액추에이터 위치 | 2 | u16 LE | 20 ms |
| `0x21B` | 액추에이터 enable 상태 | 1 | u8 0/1 | 50 ms |
| `0x21C` | 조향 override 활성 | 1 | u8 0/1 | 50 ms |
| `0x21D` | 전조등 override 활성 | 1 | u8 0/1 | 50 ms |
| `0x21E` | CAN Bus-Off 상태 | 1 | u8 0/1 | 100 ms |

`0x217`만 부호 있는 정수. 영하 온도 표현용.
송신은 15슬롯 라운드로빈 스케줄러 — 루프당 1프레임씩 순회하며 각자 주기 도달 시 송신.

### 2.3 H723 → F407_Front (명령)

| ID | 신호 | DLC | 인코딩 | 주기 |
|---|---|---|---|---|
| `0x310` | 조향 override enable | 1 | u8 0/1 | 100 ms |
| `0x311` | 액추에이터 enable | 1 | u8 0/1 | 100 ms |
| `0x312` | 액추에이터 목표값 | 2 | u16 LE (0–4095 클램프) | 100 ms |
| `0x313` | 전조등 override enable | 1 | u8 0/1 | 100 ms |
| `0x314` | 전조등 ON/OFF | 1 | u8 0/1 | 100 ms |
| `0x315` | 방향지시 override enable | 1 | u8 0/1 | 100 ms |
| `0x316` | 방향지시 모드 | 1 | u8 0=OFF 1=LEFT 2=RIGHT | 100 ms |

7슬롯 라운드로빈. Jetson 명령 500 ms 미수신 시 전 override 자동 해제(페일세이프).

### 2.4 수신 필터

**H723 FDCAN2**
- 필터0: RANGE `0x210`–`0x21E` → RxFIFO0
- 필터1: MASK `0x281` / `0x7FF` → RxFIFO0
- 글로벌: 비일치 표준/확장 REJECT, 원격프레임 REJECT

**F407 CAN1**: 전체 통과 후 `switch(StdId)` 분기

---

## 3. REAR-CAN250 (250 kbps)

### 3.1 링크 확인

| ID | 방향 | DLC | 페이로드 | 주기 |
|---|---|---|---|---|
| `0x201` | Rear H723 → 노드 | 8 | `A5 5A 02 01` + counter(u32 LE) | 500 ms |
| `0x281` | 노드 → Rear H723 | 8 | `5A A5 02 02` + counter | 수신 시 |

### 3.2 Rear H723 → 후방 노드 (명령)

| ID | 신호 | DLC | 인코딩 | 주기 | 수신 노드 |
|---|---|---|---|---|---|
| `0x320` | 좌 후미등 | 1 | u8 0/1 | 100 ms | Node_F446 |
| `0x321` | 우 후미등 | 1 | u8 0/1 | 100 ms | Node_F446 |
| `0x322` | 제동등 | 1 | u8 0/1 | 100 ms | Node_F446 |
| `0x323` | 윈도우 | 1 | u8 0/1 | 100 ms | **없음** — §5 항목 ③ |

### 3.3 Node_F446 → Rear H723 (센서)

| ID | 신호 | DLC | 인코딩 | 주기 | H723 수신 |
|---|---|---|---|---|---|
| `0x301` | IMU (BNO085) | **8** | 쿼터니언 4×int16 LE (`i,j,k,real`) | 리포트 수신 시 | ✅ |
| `0x311` | 토글 스위치 | **8** | `data[0]` 0/1, 나머지 0 패딩 | 100 ms | ✅ |
| `0x312` | 초음파 거리 | **8** | `data[0..1]` u16 LE (mm), 나머지 0 패딩 | 60 ms | ✅ |

> **DLC는 세 신호 모두 8입니다.** 유효 바이트만 앞에 채우고 나머지는 `memset` 0 패딩.
> 하행 램프 명령(`0x320`~`0x323`)은 DLC 1이라 방향에 따라 규약이 다릅니다. 수신부 작성 시 주의.
>
> `0x301` 쿼터니언은 BNO08x SHTP 리포트의 `rx[13..20]`입니다.
> 2026-08-01 이전 코드는 `rx[14..21]`을 실어 1바이트 밀려 있었고 값이 요동쳤습니다 (교정 완료).
> `0x311`은 FRONT-CAN의 `0x311`(액추에이터 enable)과 번호가 같지만 **물리 버스가 완전히 분리**되어 있어 문제 없음.
> Front H723 ↔ Rear H723은 CAN이 아니라 이더넷(UDP 5103→5003)으로만 연결됨. 문서 읽을 때만 주의.

### 3.4 수신 필터

**Rear H723 FDCAN2**
- 필터0: MASK `0x281` / `0x7FF` → RxFIFO0 (핑 에코)
- 필터1: RANGE `0x301`–`0x312` → RxFIFO0 (F446RE 센서)
- 글로벌: REJECT
- `StdFiltersNbr = 2` (`.ioc` + `fdcan.c` 양쪽 반영 필요)
**Node_F446 CAN1**: 전체 통과 (`FilterMaskIdLow = 0`), 코드에서 ID 분기
**RearBody_F407 CAN1**: MASK `0x101` / `0x7FF` 단일

---

## 4. REAR-CAN500 (500 kbps)

| ID | 방향 | DLC | 페이로드 | 주기 |
|---|---|---|---|---|
| `0x101` | Rear H723 → F407G | 8 | `A5 5A 01 01` + counter(u32 LE) | 500 ms |
| `0x181` | F407G → Rear H723 | 8 | `5A A5 01 02` + 수신 counter 그대로 | 수신 시 |

필터: MASK `0x181` / `0x7FF`.
노드 정본: `workspace_2.2.0/RearBody_F407_real` (F407VGT, presc 6 / 12·1 tq = 500 kbps, 샘플점 92.9 %).
현재는 링크 확인용 에코만 수행. 실 차량 신호 미탑재 — 용도 확정 필요.

> 이 보드는 250 kbps 버스에서 `0x201`/`0x281`을 쓰다가 500 kbps로 이설하며 ID를 재배정했습니다.
> 2026-08-01에 `RearBody_F407_real`을 500 k 규약으로 전환 완료(`can.c` + `.ioc` + `main.c`).
> `참고용파일/STM32_F407G_REAR`는 동작이 같은 참조 스냅샷(초기화가 전부 `main.c`에 있는 구조).

---

## 4.5 CAN → 이더넷 전달 (요약)

CAN으로 올라온 값은 게이트웨이가 UDP로 Jetson에 중계합니다. 상세는 `zstatus_proto.h` 참조.

| 존 | 포트 | magic | ver | 크기 | 주기 | 담는 CAN 신호 |
|---|---|---|---|---|---|---|
| Front | 5102 | `'SV'` | 1 | 28 B | 100 ms | `0x210`~`0x21E` |
| Rear | 5002 | `'SR'` | **2** | **48 B** | 100 ms | `0x301`/`0x311`/`0x312` + 하행 명령 상태 |

Rear v2는 초음파·쿼터니언·스위치와 **센서별 신선도 비트**(`RST_SEN_*_FRESH`, 1초 기준)를 포함합니다.
신선도 비트가 있어야 Jetson이 "값이 0"과 "센서 무응답"을 구분할 수 있습니다.

`zstatus_proto.h`는 **STM32와 Jetson이 같은 파일을 공유**합니다. 한쪽만 고치면 크기/버전 검사에서
전량 거부되므로 반드시 동시 반영해야 합니다.

---

## 5. 발견된 불일치 (조치 필요)

### ① ~~Rear H723가 후방 센서 3종을 전부 버림~~ — 해결됨 (2026-08-01)

`ZoneController_Rear_H723_real/Core/Src/main.c` 의 `CAN250_Start()` 필터가 `0x281` 하나뿐이고 글로벌 필터는 `FDCAN_REJECT`. Node_F446이 보내는 `0x301`(IMU) · `0x311`(스위치) · `0x312`(초음파)가 RxFIFO에 **들어오지 못합니다.**

조치 — Front처럼 RANGE 필터 추가:
```c
filter.FilterIndex  = 1U;
filter.FilterType   = FDCAN_FILTER_RANGE;
filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
filter.FilterID1    = 0x301U;
filter.FilterID2    = 0x312U;
```
`StdFiltersNbr`도 1 → 2로 상향 (`.ioc`와 `fdcan.c` **양쪽** 필요 —
실제 하드웨어에 적용되는 건 `fdcan.c` 쪽이고, `.ioc`는 CubeMX 재생성 시 되돌아가는 걸 막는 용도).

**적용 완료.** 수신 파싱·진단 카운터·신선도 tick까지 추가했고,
값은 Rear 텔레메트리 v2(48B)에 실려 Jetson으로 전달됩니다.

### ② ~~`0x281` 응답 노드 중복~~ — 해소됨 (2026-08-01 정정)

구 `RearBody_F407_real`이 250 kbps 버스에서 F446과 같은 `0x281`을 쓰던 문제였으나,
해당 보드가 **500 kbps 버스로 이설되며 `0x101`/`0x181`로 재배정**되어 충돌이 없어졌습니다.
현재 각 버스에 노드가 하나씩이라 ID 중복 여지 자체가 없습니다.

### ③ `0x323` 윈도우 명령 수신자 없음

Rear H723은 100 ms 주기로 송신 중이나 Node_F446 · RearBody_F407 어느 쪽도 `0x323`을 처리하지 않습니다. 버스 대역만 소모.

조치 — 담당 노드 지정 후 수신 구현, 또는 송신 중단.

### ④ ~~REAR-CAN500 대응 노드 없음~~ — 해소됨 (2026-08-01 정정)

`참고용파일/STM32_F407G_REAR`가 이 버스의 노드입니다. `0x101` 수신 → `0x181` 응답을 정상 구현하고 있고,
비트 타이밍도 500 kbps로 일치합니다.

단, `PROJECT_HANDOVER.md` §3의 *"Rear H723 · CAN500 · F446"* 기술은 여전히 부정확합니다.
CAN500에 붙는 건 **F446이 아니라 F407G**입니다. 핸드오버 문서 수정 필요.

또한 이 노드는 현재 에코 응답만 하며 실제 차량 신호를 싣지 않습니다. 용도 확정이 필요합니다.

---

## 6. 프로젝트 위치

| 역할 | 프로젝트 | 경로 | MCU |
|---|---|---|---|
| Front 게이트웨이 | ZoneController_Front_H723 | `c:\stm_workspace` | STM32H723ZGT |
| Front I/O | F407_Front | `c:\stm_workspace` | STM32F407VGT |
| Rear 게이트웨이 | ZoneController_Rear_H723_real | `workspace_2.2.0` | STM32H723ZGT |
| Rear 노드 (CAN250) | Node_F447_Real | `workspace_2.2.0` | **STM32F446RET** (폴더명만 F447, `.ioc`는 `Node_F446.ioc`) |
| Rear body (CAN500) | RearBody_F407_real | `workspace_2.2.0` | STM32F407VGT |

**5개 보드 모두 위 경로가 정본입니다.**
`c:\stm_workspace\참고용파일\`은 2026-08-01 시점 참조 스냅샷이며, Rear 3종은 정본보다 구버전입니다
(v1 텔레메트리 / CAN 필터 미적용 / IMU 오프셋 미교정 / F466RE는 램프 ID가 구 `0x310`).
**참고용을 플래시하면 작업이 되돌아갑니다.**

미사용 사본 (수정 금지): `c:\stm_workspace\{Node_F446, RearBody_F407, ZoneController_Rear_H723}`,
`workspace_2.2.0\ZoneController_Rear_H723_backup`

---

## 7. 변경 이력

**2026-08-01**
- Rear H723 FDCAN2에 RANGE 필터(`0x301`~`0x312`) 추가 → 후방 센서 수신 복구 (§5 ①)
- Rear 텔레메트리 v1(32 B) → v2(48 B), 초음파·IMU·스위치·신선도 비트 추가
- F446RE IMU 쿼터니언 오프셋 `rx[14..21]` → `rx[13..20]` 교정
- `RearBody_F407_real`을 250 k/`0x201`·`0x281` → 500 k/`0x101`·`0x181`로 전환
- §5 ②④ 무효 확인 (버스 분리로 ID 중복 없음, CAN500 노드는 F407G)
