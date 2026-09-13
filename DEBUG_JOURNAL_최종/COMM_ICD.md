ㅋ# Zone Controller — 이더넷/UDP 통신 ICD

> 소스에서 직접 추출한 실측 문서. 추정 없음.
> CAN 계층은 `CAN_ICD.md` 참조. 이 문서는 이더넷 계층과 게이트웨이 매핑을 다룬다.
> 기준: 2026-08-01

---

## 0. 네트워크 구성

| 노드 | IP | MAC | 비고 |
|---|---|---|---|
| Front H723 | `192.168.1.201` | `02:00:00:00:00:02` | 정적 |
| Rear H723 | `192.168.1.200` | `00:80:E1:00:00:00` | 정적 |
| Jetson | `192.168.1.60` | `3C:6D:66:1E:A4:B2` | DHCP (`enP8p1s0`) |
| Gateway | `192.168.1.254` | — | NEXTU 공유기 |

> **2026-08-06 대역 변경.** 공유기를 NEXTU 로 교체하면서 유선 DHCP 대역이
> `172.30.1.0/24` → `192.168.1.0/24` 로 바뀌었다. 젯슨 유선은 DHCP 라 자동으로
> 따라갔지만 보드는 정적이라 통신이 끊겼다. 펌웨어 5곳을 갱신하고 두 보드를
> 재플래시했다 (§METRICS 요약표 참조). 주소가 박혀 있는 곳:
>
> | 위치 | 항목 |
> |---|---|
> | `ZoneController_Front_H723/LWIP/App/lwip.c` | 자기 IP + 게이트웨이 |
> | `ZoneController_Rear_H723_real/LWIP/App/lwip.c` | 자기 IP + 게이트웨이 |
> | `Core/Src/front_telemetry.c` | `FT_JETSON_IP0..3` |
> | `Core/Src/rear_telemetry.c` | `RT_JETSON_IP0..3` |
> | `Core/Src/front_zone_network.c` | `IP_ADDR4(&rear_ip_address, ...)` |
>
> `lwip.c` 의 IP 블록은 USER CODE 구역 밖이라 **CubeMX Generate Code 가
> 덮어쓴다.** 재생성 후 반드시 확인할 것.
>
> 젯슨 유선 주소는 DHCP 인데 펌웨어는 고정으로 안다. 재할당되면 텔레메트리가
> 끊기므로 공유기에서 MAC `3C:6D:66:1E:A4:B2` 에 DHCP 예약을 거는 것이 안전하다.

Netmask `255.255.255.0`. lwIP 2.2.1, **NO_SYS=1** (폴링). 전 구간 **UDP**.

> **TCP를 쓰지 않는 이유** — 차량 상태는 주기적으로 덮어써지는 데이터다.
> 한 패킷을 놓쳐도 다음 주기가 최신값을 싣고 오므로 재전송이 무의미하고,
> TCP의 head-of-line 블로킹이 오히려 최신 데이터 도착을 지연시킨다.
> 대신 시퀀스 번호로 손실을 **감지**한다.

---

## 1. 포트 맵

| 포트 | 방향 | 패킷 | 크기 | 주기 |
|---|---|---|---|---|
| **5101** | Jetson → Front | FRONT_COMMAND | 12 B | 이벤트 |
| **5102** | Front → Jetson | `'SV'` 텔레메트리 | 28 B | 100 ms |
| **5103** | Front ↔ Rear | FRONT_STATE 송신 / REAR_STATUS 수신 | 12 B | 50 ms |
| **5104** | Front → Jetson | `'SP'` 성능 통계 | 104 B | 1000 ms |
| **5001** | Jetson → Rear | REAR_COMMAND | 12 B (구형 4 B 호환) | 이벤트 |
| **5002** | Rear → Jetson | `'SR'` 텔레메트리 | 48 B | 100 ms |
| **5003** | Rear ↔ Front | FRONT_STATE 수신 / REAR_STATUS 송신 | 12 B | 응답 시 |
| **5105** | Front·Rear → Jetson | `'SD'` 진단(DTC) | 232 B | 상태변화 즉시 + 1 s |

`5104`는 RTOS 전환 전후 비교용 임시 계측 채널. 완료 후 제거 가능.

---

## 2. 존 프로토콜 (12 바이트) — `zone_udp_protocol.h`

Front ↔ Rear ↔ Jetson 제어 계통 공통 포맷.

### 2.1 공통 헤더

```
 오프셋  크기  필드
   0      1    0x5A                  고정 매직
   1      1    존 식별   'F'=0x46 Front / 'R'=0x52 Rear / 'J'=0x4A Jetson
   2      1    프로토콜 버전 = 1
   3      1    메시지 타입
   4      2    시퀀스 번호 (u16 LE)
   6..10  5    페이로드 (타입별)
  11      1    XOR 체크섬 (byte 0~10 전체 XOR)
```

메시지 타입: `0x10` FRONT_STATE / `0x11` REAR_STATUS / `0x20` FRONT_COMMAND / `0x21` REAR_COMMAND

**검증은 4중** — 길이, 매직 2바이트, 버전, 체크섬. 하나라도 틀리면 `*_bad_count` 증가 후 폐기.

### 2.2 FRONT_STATE (`0x10`) — Front → Rear, 50 ms

| 오프셋 | 내용 |
|---|---|
| 6 | 방향지시 모드 0=OFF 1=LEFT 2=RIGHT |
| 7 | 브레이크 눌림 0/1 |
| 8~9 | 브레이크 ADC 원값 (u16 LE) |
| 10 | 플래그 — bit0 브레이크 신선, bit1 방향지시 신선, bit2 F407 버스오프, bit3 브레이크 캘리브레이션 완료 |

### 2.3 REAR_STATUS (`0x11`) — Rear → Front, FRONT_STATE 수신 시 + 100 ms 하트비트

| 오프셋 | 내용 |
|---|---|
| 4~5 | **수신한 FRONT_STATE의 시퀀스를 그대로 반향** (왕복 추적용) |
| 6 | 실제 적용된 방향지시 모드 |
| 7 | 실제 적용된 브레이크 |
| 8 | 출처 플래그 — bit0 Front링크 생존, bit1 방향지시 출처=Jetson, bit2 브레이크 출처=Jetson, bit3 Jetson링크 생존 |
| 9 | CAN250 버스오프 |
| 10 | 윈도우 명령 상태 |

### 2.4 FRONT_COMMAND (`0x20`) — Jetson → Front, 포트 5101

| 오프셋 | 내용 |
|---|---|
| 6 | 오버라이드 플래그 (아래) |
| 7~8 | 액추에이터 목표값 (u16 LE, **0~4095 범위 검사**) |
| 9 | 방향지시 모드 (**0~2 범위 검사**) |

```c
FRONT_CMD_FLAG_STEERING_OVERRIDE   (1<<0)
FRONT_CMD_FLAG_ACTUATOR_ENABLE     (1<<1)
FRONT_CMD_FLAG_HEADLAMP_OVERRIDE   (1<<2)
FRONT_CMD_FLAG_HEADLAMP_ON         (1<<3)
FRONT_CMD_FLAG_TURN_OVERRIDE       (1<<4)
```

### 2.5 REAR_COMMAND (`0x21`) — Jetson → Rear, 포트 5001

| 오프셋 | 내용 |
|---|---|
| 6 | 오버라이드 플래그 `REAR_CMD_FLAG_{TURN,BRAKE,WINDOW}_OVERRIDE` (1<<0,1,2) |
| 7 | 방향지시 모드 (0~2) |
| 8 | 브레이크 (0/1) |
| 9 | 윈도우 (0/1) |

> **구형 4바이트 호환 경로 존재** — `[window, left, right, brake]` 각 0/1.
> 수신 시 3개 오버라이드가 전부 켜진 것으로 간주한다. 신규 개발은 12바이트 사용.

### 2.6 ACK

Front·Rear 모두 명령 수신 시 **받은 패킷을 그대로 송신지로 반향**한다.
Jetson은 이걸로 명령 도달을 확인할 수 있다. (단, *반영*이 아니라 *수신* 확인)

---

## 3. 텔레메트리 패킷

### 3.1 `'SV'` Front — 28 B, 포트 5102, 100 ms

`vstatus_proto.h` (STM32) = `zstatus_proto.h` (Jetson) 의 VST 부분.

```
 0  u16  magic 0x5653 'SV'      12  u16  steering_adc
 2  u8   version = 1            14  u16  accel_adc
 3  u8   flags (VST_FLAG_*)     16  u16  brake_adc
 4  u32  seq                    18  u16  actuator_pos
 8  u32  uptime_ms              20  i16  temp_x10
                                22  u16  humidity_x10
                                24  u8   turn_left
                                25  u8   turn_right
                                26  u8   headlamp
                                27  u8   reserved
```

### 3.2 `'SR'` Rear — 48 B, 포트 5002, 100 ms, **버전 2**

```
 0  u16  magic 0x5253 'SR'      20  u16  ultrasonic_mm     CAN 0x312
 2  u8   version = 2            22  i16  quat_i            CAN 0x301
 3  u8   source (RST_SRC_*)     24  i16  quat_j
 4  u32  seq                    26  i16  quat_k
 8  u32  uptime_ms              28  i16  quat_real
12  u8   turn_mode              30  u8   sensor (RST_SEN_*)
13  u8   led_left    CAN 0x320  31  u8   reserved
14  u8   led_right   CAN 0x321  32  u32  can500_rx_count
15  u8   led_brake   CAN 0x322  36  u32  can250_rx_count
16  u8   window      CAN 0x323  40  u16  cmd_rx_count
17  u8   health (RST_HLT_*)     42  u16  imu_rx_count
18  u16  front_brake_adc        44  u16  ultra_rx_count
                                46  u16  switch_rx_count
```

> ⚠ **`zstatus_proto.h`는 STM32와 Jetson이 같은 파일을 공유한다.**
> 한쪽만 고치면 커널 드라이버의 크기/버전 검사에서 **전량 거부**된다.
> 구조체 크기는 양쪽 모두 컴파일 타임 assert 로 검증된다.

**상태 비트 정의**

```c
RST_SRC_FRONT_LINK   (1<<0)   Front H723 링크 생존
RST_SRC_TURN_JETSON  (1<<1)   방향지시 출처: 1=Jetson, 0=Front
RST_SRC_BRAKE_JETSON (1<<2)   브레이크 출처
RST_SRC_JETSON_LINK  (1<<3)   Jetson 명령 링크 생존

RST_HLT_CAN500_ERR   (1<<0)   FDCAN1 버스오프
RST_HLT_CAN250_ERR   (1<<1)   FDCAN2 버스오프

RST_SEN_SWITCH       (1<<0)   토글 스위치 눌림 (값)
RST_SEN_ULTRA_FRESH  (1<<1)   초음파 1초 내 수신 (신선도)
RST_SEN_IMU_FRESH    (1<<2)   IMU 1초 내 수신
RST_SEN_SWITCH_FRESH (1<<3)   스위치 1초 내 수신
```

> **신선도 비트가 필요한 이유** — 값이 0인 것과 센서가 죽은 것은 완전히 다르다.
> 초음파 0 mm는 "장애물 접촉"일 수도, "센서 무응답"일 수도 있다.
> 이 비트가 없으면 상위에서 구분이 불가능하다. **DTC 판정의 1차 근거.**

### 3.3 `'SP'` 성능 통계 — 104 B, 포트 5104, 1000 ms (임시)

헤더 8 B + `perf_stat_wire` 24 B × 4구간 (loop / lwip / can_isr / can_to_udp).
각 구간은 `last/min/max/avg/count/over_1ms` (µs 단위).

---

### 3.4 `'SD'` 진단 — 232 B, 포트 5105, 상태변화 즉시 + 1 s 하트비트

텔레메트리(`'SV'`/`'SR'`)와 **분리한 패킷**이다. 이유 셋:

- 기존 패킷 크기를 바꾸면 Jetson 커널 드라이버가 크기·버전 검사로 전량 거부한다
- 진단은 이벤트성이라 주기가 다르다
- UDS 조회 인터페이스로 확장하기 쉽다

**헤더 16 B**

| off | 크기 | 필드 | 내용 |
|---|---|---|---|
| 0 | 2 | `magic` | `'SD'` = `0x4453` (LE) |
| 2 | 1 | `version` | 1 |
| 3 | 1 | `zone` | `'F'`=0x46 / `'R'`=0x52 |
| 4 | 4 | `seq` | 송신 시퀀스 |
| 8 | 4 | `uptime_ms` | |
| 12 | 1 | `count` | 유효 엔트리 수 (≤ 12) |
| 13 | 1 | `active` | 현재 TEST_FAILED 인 개수 |
| 14 | 1 | `confirmed` | CONFIRMED 이력 개수 |
| 15 | 1 | reserved | |

**엔트리 16 B × 12 = 192 B** (off 16..207)

| off | 크기 | 필드 | 내용 |
|---|---|---|---|
| 0 | 2 | `code` | SAE J2012 2바이트 인코딩 |
| 2 | 1 | `status` | `DTC_STS_*` 비트 |
| 3 | 1 | `occurrence` | 발생 횟수 (255 포화) |
| 4 | 4 | `first_ms` | 최초 확정 시각 |
| 8 | 4 | `last_ms` | 최근 고장 관측 시각 |
| 12 | 2 | `detect_ms` | **실측** 검출 지연 |
| 14 | 2 | reserved | |

**프리즈 프레임 24 B** (off 208..231)

| off | 크기 | 필드 |
|---|---|---|
| 0 | 2 | `code` — 이 프레임을 트리거한 DTC |
| 2 | 1 | `valid` — 0 = 캡처된 적 없음 |
| 4 | 4 | `uptime_ms` |
| 8 | 16 | `sig[8]` — 신호 스냅샷 (u16 × 8) |

OBD-II 와 같이 **존당 하나만** 유지하며 `Diag_Clear()` 전까지 덮어쓰지 않는다.
첫 고장이 근본 원인일 가능성이 높고, 이후 연쇄 고장이 원본을 지우면 안 되기 때문이다.

**상태 비트** (ISO 14229 statusOfDTC 부분 구현)

| 비트 | 이름 | 의미 |
|---|---|---|
| 0x01 | TEST_FAILED | 현재 고장 중 |
| 0x02 | PENDING | 디바운스 중 |
| 0x04 | **CONFIRMED** | 확정 — **한 번 서면 지워지지 않는다** |
| 0x08 | WARNING | 운전자 경고 대상 |

> 현재 상태를 보려면 `TEST_FAILED` 를 봐야 한다. `CONFIRMED` 는 이력이라
> 고장이 해소돼도 남는다. 이걸 현재 상태로 오해하면 안 된다.

**DTC 코드 인코딩** — `bit15..14` = 00 P(파워트레인) / 01 C(섀시) / 10 B(바디) / 11 U(네트워크)

```
예)  U0100  ->  0b11 << 14 | 0x0100 = 0xC100
```

| 코드 | 값 | 의미 |
|---|---|---|
| U0001 | `0xC001` | FDCAN1(500k) 버스오프 |
| U0002 | `0xC002` | FDCAN2(250k) 버스오프 |
| U0003 | `0xC003` | 하위 노드가 버스오프를 보고 |
| U0100 | `0xC100` | 존간 링크 상실 |
| U0200 | `0xC200` | 프레임 무결성 오류 급증 |
| U0300 | `0xC300` | Jetson 명령 링크 상실 |
| P0500 | `0x0500` | 초음파(0x312) 무응답 |
| P0501 | `0x0501` | IMU(0x301) 무응답 |
| P0502 | `0x0502` | 토글스위치(0x311) 무응답 |
| P0503 | `0x0503` | DHT11 온습도 무효 |
| P0510 | `0x0510` | 하위 노드 CAN 신호 두절 |
| U0101 | `0xC101` | Front 텔레메트리 두절 — **Jetson 이 판정**, STM32 는 보내지 않음 |

---

## 3-b. RTOS 태스크 구조

두 H723 모두 **FreeRTOS + lwIP `NO_SYS=0`** (CubeMX 정식 경로) 로 동작한다.
전환 근거와 실측은 `METRICS.md` §6-c(Rear) / §6-e(Front) 참조.

### Rear H723

| 태스크 | 우선순위 | 주기 | 역할 |
|---|---|---|---|
| `EthIf` | 실시간(48) | 이벤트 | ETH RX 세마포어 대기 (CubeMX) |
| `tcpip_thread` | 24 | 이벤트 | lwIP 코어 (CubeMX) |
| `wheel` | 27 | 10 ms | FDCAN1 500k — RearBody F407 핑/에코 |
| `ctrl` | 25 | 10 ms + 알림 | FDCAN2 250k, 하행 지령, CAN 진단·버스오프 복구 |
| `net` | 23 | 20 ms | `'SR'`·`'SP'`·`'SD'` 송신 |
| `diag` | 22 | 50 ms | DTC 판정 |
| `defaultTask` | 24 | 초기화 후 유휴 | PHY 지연 → `MX_LWIP_Init()` → UDP 개설 → 태스크 기동 |

### Front H723

| 태스크 | 우선순위 | 주기 | 역할 |
|---|---|---|---|
| `EthIf` | 실시간(48) | 이벤트 | ETH RX 세마포어 대기 |
| `tcpip_thread` | 24 | 이벤트 | lwIP 코어 |
| `ctrl` | 25 | 10 ms + 알림 | FDCAN2 하행 명령 7종, CAN 진단·버스오프 복구 |
| `net` | 23 | 20 ms | `'SV'`·`'SP'`·`'SD'` 송신, 존간 FRONT_STATE |
| `diag` | 22 | 50 ms | DTC 판정, 스택·힙 여유 기록 |
| `defaultTask` | 24 | 초기화 후 유휴 | 〃 |

Front 는 CAN 버스가 FDCAN2 하나뿐이라 `ctrl` 이 독점하면 되고, 그 결과
**CAN 락이 필요 없다.** Rear 는 두 버스를 쓰지만 각각 전용 태스크가 소유한다.

### 지켜야 할 규칙 둘

**① lwIP raw API 는 반드시 코어 락 안에서.**
`NO_SYS=0` 에서 lwIP 는 `tcpip_thread` 소유다. 다른 태스크에서 `udp_sendto()`
등을 부르려면 `LOCK_TCPIP_CORE()` / `UNLOCK_TCPIP_CORE()` 로 감싸야 한다.
없으면 ARP 큐의 pbuf 가 양쪽에서 해제돼 `mem_free()` 이중 해제로 죽는다.

**② 스레드 스택은 바이트 단위다.**
CMSIS-RTOS v2 의 `stack_size` 는 워드가 아니라 **바이트**다. CubeMX 기본값
`INTERFACE_THREAD_STACK_SIZE (350)` 은 350 B 이고 `EthIf` 실사용은 404 B 라
넘친다. 넘친 스택이 `heap_4` 인접 블록을 덮어써 엉뚱한 곳에서 죽는다.
현재 `EthIf` 1024 B, `tcpip_thread` 2048 B 로 조정했다.

| 스레드 | 할당 | 실사용 최대 | 여유 |
|---|---|---|---|
| EthIf | 1024 B | 404 B | 61 % |
| tcpip_thread | 2048 B | 844 B | 59 % |
| defaultTask | 4096 B | 608 B | 85 % |
| ctrl | 2048 B | 256 B | 88 % |
| net | 4096 B | 1004 B | 75 % |
| diag | 2048 B | 176 B | 91 % |

FreeRTOS 힙 48,000 B 중 최소 여유 29,760 B (62 %).

---

## 4. 타임아웃과 근거 — **DTC 판정 기준**

| 상수 | 값 | 감시 대상 | 정상 주기 | 배수 | 동작 |
|---|---|---|---|---|---|
| `FRONT_TO_REAR_TIMEOUT_MS` | 300 | CAN 신호 신선도 | 20~50 ms | 6~15× | 브레이크/방향지시 무효화 |
| `REAR_STATUS_TIMEOUT_MS` | 300 | Rear→Front 상태 | 50~100 ms | 3~6× | `front_rear_link_alive = 0` |
| `JETSON_COMMAND_TIMEOUT_MS` | 500 | Jetson 명령 | 이벤트 | — | **전 오버라이드 해제, 로컬 제어 복귀** |
| `RT_SENSOR_FRESH_MS` | 1000 | F446RE 센서 | 60~100 ms | 10~16× | 신선도 비트 0 |

**설계 원칙: 타임아웃 = 정상 주기 × 3배 이상.** 단발 패킷 손실로 오검출되지 않게 하기 위함.

### 페일세이프 계층

```
Jetson 끊김 (500 ms)  →  오버라이드 해제, Front 로컬 제어로 복귀
Front 끊김  (300 ms)  →  Rear 램프 전부 OFF
CAN 끊김    (300 ms)  →  브레이크 신호 무효, 방향지시 OFF
F446 명령끊김(500 ms) →  후미등 소등 (F446RE 자체 판정)
```

**모든 경로가 "무응답 → 안전 상태"로 수렴한다.** 어느 한 곳이 죽어도 액추에이터가 마지막 명령에 고착되지 않는다.

---

## 5. 게이트웨이 신호 매핑

### 5.1 Front H723

| 방향 | CAN | ↔ | UDP |
|---|---|---|---|
| 상행 | `0x210` steering ADC | → | `'SV'` [12] |
| 상행 | `0x211` brake ADC | → | `'SV'` [16], FRONT_STATE [8] |
| 상행 | `0x212` accel ADC | → | `'SV'` [14] |
| 상행 | `0x213/0x214` 방향지시 출력 | → | `'SV'` [24][25], FRONT_STATE [6] |
| 상행 | `0x216` 전조등 | → | `'SV'` [26] |
| 상행 | `0x217/0x218/0x219` 온습도 | → | `'SV'` [20][22] + flags |
| 상행 | `0x21A` 액추에이터 위치 | → | `'SV'` [18] |
| 상행 | `0x21E` 버스오프 | → | `'SV'` flags bit1 |
| 하행 | `0x310`~`0x316` | ← | FRONT_COMMAND [6][7~8][9] |

**브레이크 판정은 게이트웨이가 수행한다** — 원시 ADC를 받아 부팅 후 50샘플 평균으로 기준선을 잡고, 히스테리시스(ON 300 / OFF 150)로 눌림을 판정한다. F407은 원값만 보내고 해석은 게이트웨이 몫이다.

### 5.2 Rear H723

| 방향 | CAN | ↔ | UDP |
|---|---|---|---|
| 상행 | `0x301` IMU | → | `'SR'` [22~29] 쿼터니언 4×i16 분해 |
| 상행 | `0x311` 스위치 | → | `'SR'` sensor bit0 |
| 상행 | `0x312` 초음파 | → | `'SR'` [20] |
| 하행 | `0x320`~`0x323` | ← | **중재 결과** (아래) |

### 5.3 소스 중재 — 게이트웨이의 핵심 기능

Rear는 **두 도메인에서 명령을 받는다**. 우선순위 규칙:

```
신호별로 독립 판정:

  Jetson 링크 생존 AND 해당 오버라이드 비트 ON
      → Jetson 명령 채택          (source 비트 = 1)
  아니면
      Front 링크 생존             → Front 상태 반영
      아니면                       → OFF (안전 상태)
```

`rear_applied_*`가 최종 결과이며, **어느 출처를 썼는지 `'SR'` source 비트로 Jetson에 보고**한다. 상위에서 중재 결과를 감사(audit)할 수 있다.

---

## 6. 진단 후보 (DTC 설계 입력)

이미 구현된 감지 로직 → DTC 격상 대상.

| 감지 변수 | 위치 | 판정 | DTC 후보 |
|---|---|---|---|
| `front_rear_link_alive` | Front | 300 ms | U0100 Front↔Rear 링크 상실 |
| `rear_front_link_alive` | Rear | 300 ms | U0101 (반대 방향) |
| `front_jetson_command_alive` | Front | 500 ms | U0300 Jetson 링크 상실 |
| `rear_jetson_command_alive` | Rear | 500 ms | U0301 |
| `can500_protocol_status.BusOff` | Rear | 즉시 | U0001 CAN500 버스오프 |
| `can250_protocol_status.BusOff` | Rear/Front | 즉시 | U0002 CAN250 버스오프 |
| `f407_can_busoff` | Front (CAN 보고) | 즉시 | U0003 Front F407 버스오프 |
| `RST_SEN_ULTRA_FRESH` | Rear | 1000 ms | P0500 초음파 무응답 |
| `RST_SEN_IMU_FRESH` | Rear | 1000 ms | P0501 IMU 무응답 |
| `RST_SEN_SWITCH_FRESH` | Rear | 1000 ms | P0502 스위치 무응답 |
| `*_bad_count` 급증 | 전 노드 | — | U0200 프레임 무결성 오류 |

**현재 없는 것** — 저장, 이력(최초/최근 발생 시각, 발생 횟수), 디바운스, 조회 인터페이스, 삭제.
이것이 다음 작업 범위다.

---

## 7. 미해결 / 주의

- ~~`'SR'` 확장 시 Jetson 커널 드라이버 동시 수정 필수~~ → **해결.** DTC 를
  별도 패킷 `'SD'`(5105)로 분리했다 (§3.4).
- **`0x323` 고아 신호** — Rear 가 10.1 Hz 로 보내지만 F446 이 받지 않아 전량
  유실되고 `can250_rx_bad_count` 를 오염시킨다. 설계 판단 대기 (METRICS §5-h).
- **빌드 구성 불일치** — Rear 는 `Release`, Front 는 `Debug` 에만 FreeRTOS 빌드
  스크립트가 있다. 반대 구성을 고르면 빌드가 깨진다.
- **FreeRTOS 포트** — CubeMX 가 M7 인데 `ARM_CM4F` 를 골랐다. `ARM_CM7/r0p1`
  로 바꾸는 것이 맞다 (현재 동작에는 문제 없음).
- **Rear 잔여물** — `defaultTask` 가 8 KB 스택으로 유휴, `can2_lock` 뮤텍스가
  생성만 되고 미사용(NOTIFY 모드).
- **Rear 에 진단 펌웨어 탑재 중** — `CAN500_SNIFF 1` 로 FDCAN1 전역 필터가
  열려 있고 프로브/ID 스윕이 들어 있다. MD400T 작업 후 되돌릴 것.
- **젯슨 유선이 DHCP** 인데 펌웨어는 그 주소를 고정으로 안다. 공유기에서
  MAC `3C:6D:66:1E:A4:B2` 에 DHCP 예약 권장 (§0).
- `5104` 성능 계측 채널은 임시. RTOS 비교 완료 후 제거.
- Front는 HSI(내부 RC) 클럭 사용 → `uptime_ms` 정확도가 낮다(±1 % 수준).
  시간 상관 분석 시 Jetson 수신 시각을 기준으로 삼을 것.
- REAR_COMMAND 구형 4바이트 경로는 검증이 약하다(범위 검사만, 체크섬 없음). 폐기 검토 필요.

---

## 8. 변경 이력

**2026-08-01**
- 최초 작성. 포트 맵·패킷 4종·타임아웃 근거·게이트웨이 매핑·중재 규칙 정리
- `'SR'` v1(32 B) → v2(48 B) 반영 (후방 센서 + 신선도 비트 추가)
- `'SP'` 성능 계측 채널(5104) 신설

**2026-08-04**
- `'SD'` 진단 패킷(5105) 신설 — DTC 12 종, 프리즈 프레임, 상태 비트 4 종
- CAN 진단 기준 수정 — BusOff 만 감시하던 것을 RX 타임아웃 기준으로 (500 ms 검출)
- CAN 버스오프 자동 복구 추가 (`CCCR.INIT` 해제 + 1 s 재시도), Front·Rear 양쪽

**2026-08-05**
- 두 H723 을 **FreeRTOS + lwIP `NO_SYS=0`** 로 전환 (§3-b)
- 기능별 태스크 분리 — Rear wheel/ctrl/net/diag, Front ctrl/net/diag
- lwIP 코어 락(`LOCK_TCPIP_CORE`) 적용 — 앱 태스크의 raw API 호출 보호
- `EthIf` 1024 B / `tcpip_thread` 2048 B 로 스택 조정 (기본값 오버플로)
- 젯슨 통합 HMI(`zone_hmi.py`) — 진단 표시 + 액추에이터 제어 8 종

**2026-08-06**
- 공유기 교체(NEXTU)로 유선 대역 `172.30.1.0/24` → `192.168.1.0/24` (§0)
- 펌웨어 5 곳 갱신 + 두 보드 재플래시, ping 30/30 무손실 확인

---

## 9. 트러블슈팅 — 실제로 겪은 문제와 원인

> 2026-08-01 하루 종일 이더넷 불안정을 추적하며 확인한 것들.
> 증상이 전부 "리셋하면 되고 전원 인가로는 안 된다"로 같아서 구분이 어려웠다.

### 9.1 이더넷: 파워온만 실패하고 리셋은 정상 — 캐시 ECC ★ 근본 원인

**증상**
- 전원 인가로는 이더넷이 안 뜨고, 리셋 버튼을 누르면 동작
- 뜨더라도 수 초~수 분 뒤 HardFault 로 정지 (CAN ISR 포함 전부 멈춤)
- 간헐적 — 어떤 부팅은 5분, 어떤 부팅은 2초

**진단**
```
CFSR = 0x00000400   bit10 IMPRECISERR (부정확 버스 오류)
HFSR = 0x40000000   bit30 FORCED
BFAR                무효 (부정확이라 주소 미기록)
폴트 PC             HAL_ETH_ReadData -> HAL_ETH_RxLinkCallback 부근
```

**원인**
파워온 직후 Cortex-M7 캐시 RAM 에 임의의 노이즈가 남는다. M7 캐시는 ECC 를
갖고 있어서 "전체 무효화"가 아닌 **범위 지정 무효화**를 수행하면 노이즈가 든
라인에서 ECC 오류가 나고 BusFault 로 전파된다.
`HAL_ETH_RxLinkCallback()` 이 수신 프레임마다 `SCB_InvalidateDCache_by_Addr()`
를 호출하므로 정확히 이 경로에 걸린다. 리셋 버튼은 캐시를 정리하므로 재현되지 않는다.

**수정** — `main()` 최상단, 다른 어떤 초기화보다 먼저
```c
SCB_InvalidateDCache();
```

### 9.2 D2 SRAM 클럭 미활성

**증상** 위와 동일하게 나타나 구분이 어려움

**진단**
```
RCC_AHB2ENR (0x580244DC) = 0x00000000   ← SRAM1/SRAM2 클럭 전부 OFF
정상값                    = 0x60000000
```

**원인**
이더넷 DMA 디스크립터(0x30000000) · RX 버퍼 풀(0x30000100) · lwIP 힙(0x30005000)
이 전부 D2 SRAM 에 있는데 이 클럭은 리셋 후 기본 OFF 다.
CMSIS `SystemInit()` 에 활성화 코드가 있으나 `#if defined(DATA_IN_D2_SRAM)` 로
감싸여 있고 이 프로젝트는 그 매크로를 정의하지 않아 실행되지 않는다.

**수정** — `EthernetMPU_Config()` 앞
```c
__HAL_RCC_D2SRAM1_CLK_ENABLE();
__HAL_RCC_D2SRAM2_CLK_ENABLE();
```

### 9.3 파워온 시 PHY 열거 실패

**원인** LAN8742 는 전원 인가 후 MDIO 응답까지 시간이 필요하다
(데이터시트 최소 약 26 ms, 실사용 500 ms~2 s 보고 다수).
`low_level_init()` 은 `LAN8742_Init()` 을 한 번만 호출하고, 실패하면
`GetLinkState()` 가 `READ_ERROR(-5)` 를 돌려주는데
`ethernet_link_check_state()` 의 복구 조건이 `PHYLinkState > LINK_DOWN(1)`
이라 `-5 > 1` 이 거짓이 되어 복구 분기에 진입조차 못 한다.

**수정** `MX_LWIP_Init()` 앞에 `HAL_Delay(500U)`
보조로 `eth_phy_recover.c` (PHY 재초기화 재시도) 를 두었으나 현재 비활성.

### 9.4 RMII GPIO 속도

50 MHz 신호이므로 `GPIO_SPEED_FREQ_VERY_HIGH` 를 쓴다.
CubeMX 기본값이 `HIGH` 인 경우가 있어 확인 필요.

### 9.5 CAN: 버스오프 후 영구 정지

**증상** 노드가 한 번 죽으면 보드 리셋 전까지 복구 안 됨

**원인** `hcanX.Init.AutoBusOff = DISABLE`
전원 인가 시점에 버스에 트래픽이 있거나 배선이 순간 튀면 버스오프가 되고,
`DISABLE` 이면 소프트웨어가 `HAL_CAN_Start()` 를 다시 부를 때까지 죽어 있다.

**수정** `AutoBusOff = ENABLE` — 128 x 11 recessive bit 후 하드웨어 자동 복귀.
전 노드가 동일해야 한다.

### 9.6 CAN: 샘플점 불일치로 인한 프레임 유실

**증상** 통신은 되는데 프레임의 약 절반이 유실

**진단** (Rear CAN500 실측)
```
can500_tx_count = 17   송신
can500_rx_count =  9   수신     ← 절반 유실
can500_rx_bad   =  9
```

**원인** H723 FDCAN1 은 16 tq / 샘플점 87.5 %, F407 은 14 tq / 92.9 % 로
5.4 % 차이. (정상 동작하는 250 k 버스는 1.8 % 차이)

**수정** F407 을 `BS1=11, BS2=2` (14 tq, 85.7 %) 로 변경 → 유실 0.
42 MHz 에서는 16 tq 를 만들 수 없으므로 가능한 조합 중 가장 가까운 것을 쓴다.

---

## 10. 진단 방법 (도구)

CubeIDE 없이 ST-Link 로 직접 읽는 절차. 프로브가 여러 개여도 `sn=` 으로 지정한다.

```bash
PROG=".../STM32_Programmer_CLI.exe"

# 프로브 목록
$PROG -l

# 메모리 읽기 (HOTPLUG = 리셋 없이 현재 상태 그대로)
$PROG -c port=SWD sn=<SN> mode=HOTPLUG -r32 0xE000ED28 1    # CFSR
$PROG -c port=SWD sn=<SN> mode=HOTPLUG -r32 0x580244DC 1    # RCC_AHB2ENR

# 플래시 (SN 지정 필수 — 미지정 시 첫 프로브에 굽는다)
$PROG -c port=SWD sn=<SN> mode=UR reset=HWrst -w app.elf -v --start
```

**보드 식별** — 같은 MCU 가 여러 개면 리셋 벡터(0x08000004)를 읽어 ELF 와 대조한다.

**변수 읽기** — `arm-none-eabi-nm --defined-only app.elf` 로 주소를 얻어 `-r32` 로 읽는다.

**GDB** — `ST-LINK_gdbserver.exe -p <port> -d -s -cp <프로그래머경로> -i <SN> --attach`
후 `arm-none-eabi-gdb -x script.gdb app.elf`. `compare-sections` 로 플래시와 ELF 일치 확인 가능.

### CFSR 비트 해석

| 비트 | 값 | 이름 | 의미 |
|---|---|---|---|
| 1 | 0x0002 | DACCVIOL | MPU 데이터 접근 위반 |
| 9 | 0x0200 | PRECISERR | 정확한 버스 오류 (BFAR 유효) |
| 10 | 0x0400 | **IMPRECISERR** | 부정확 — 버퍼된 쓰기 실패, BFAR 무효 |
| 11 | 0x0800 | UNSTKERR | 예외 복귀 시 언스택 실패 |
| 12 | 0x1000 | STKERR | 예외 진입 시 스택 실패 |
| 24 | 0x0100_0000 | UNALIGNED | 정렬되지 않은 접근 |

> **부정확 폴트를 정확하게 만들려는 시도는 이 프로젝트에서 통하지 않는다.**
> MPU 를 Strongly-ordered(TEX=0)로 바꾸면 쓰기 버퍼링이 사라져 PRECISERR 이 되지만,
> lwIP 가 패킷 헤더에 비정렬 쓰기를 하므로 UNALIGNED UsageFault 가 대신 발생한다.
> (실측 확인함)
