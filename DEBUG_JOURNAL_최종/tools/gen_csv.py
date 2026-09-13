#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""보드별 송수신 신호 목록과 실행시간 측정값을 CSV 로 정리한다."""
import csv
import io
import os

OUT = r"c:\stm_workspace\tools"

# --------------------------------------------------------------------
# 1) 신호 목록
#    board, role, peer, channel, id_port, signal, bytes, period_ms,
#    trigger, source
# --------------------------------------------------------------------
SIG = []


def add(board, role, peer, ch, idp, sig, nbytes, period, trig, src):
    SIG.append([board, role, peer, ch, idp, sig, nbytes, period, trig, src])


# ---- Front H723 : CAN250 (FDCAN2) -> F407_Front ----
_f_cmd = [("0x310", "steering_override", 1), ("0x311", "actuator_enable", 1),
          ("0x312", "actuator_target", 2), ("0x313", "headlamp_override", 1),
          ("0x314", "headlamp_command", 1), ("0x315", "turn_override", 1),
          ("0x316", "turn_mode", 1)]
for i, s, n in _f_cmd:
    add("Front_H723", "TX", "F407_Front", "CAN 250k", i, s, n, 100,
        "이벤트+주기(최소간격10ms)", "FRONT_COMMAND_TX_PERIOD_MS")
add("Front_H723", "TX", "F407_Front", "CAN 250k", "0x201", "ping", 8, 500,
    "주기", "CAN250_TX_PERIOD_MS")

_f_rx = [("0x210", "steering_adc", 2, 20), ("0x211", "brake_adc", 2, 20),
         ("0x212", "accel_adc", 2, 20), ("0x213", "turn_left_output", 1, 50),
         ("0x214", "turn_right_output", 1, 50), ("0x215", "turn_switch", 1, 50),
         ("0x216", "headlamp_output", 1, 50), ("0x217", "temperature_x10", 2, 1000),
         ("0x218", "humidity_x10", 2, 1000), ("0x219", "dht11_valid", 1, 1000),
         ("0x21A", "actuator_position", 2, 20), ("0x21B", "actuator_enable_st", 1, 50),
         ("0x21C", "steering_override_st", 1, 50), ("0x21D", "headlamp_override_st", 1, 50),
         ("0x21E", "can_busoff", 1, 100), ("0x281", "ping_echo", 8, 500)]
for i, s, n, p in _f_rx:
    add("Front_H723", "RX", "F407_Front", "CAN 250k", i, s, n, p,
        "주기(ISR수신)", "CAN_PERIOD_* (F407측)")

# ---- Front H723 : UDP ----
add("Front_H723", "RX", "Jetson", "UDP", "5101", "FRONT_COMMAND", 12, 0,
    "이벤트", "zone_udp_protocol.h")
add("Front_H723", "TX", "Jetson", "UDP", "5101", "ACK 반향", 12, 0,
    "수신 시 즉시", "COMM_ICD 2.6")
add("Front_H723", "TX", "Jetson", "UDP", "5102", "'SV' 텔레메트리", 28, 100,
    "이벤트+주기(최소간격10ms)", "FT_PERIOD_MS")
add("Front_H723", "TX", "Jetson", "UDP", "5104", "'SP' perf", 104, 1000,
    "주기", "FT_PERF_PERIOD_MS")
add("Front_H723", "TX", "Jetson", "UDP", "5105", "'SD' DTC", 232, 1000,
    "상태변화 시 즉시+하트비트", "diagnostic.c")
add("Front_H723", "TX", "Rear_H723", "UDP", "5003", "FRONT_STATE", 12, 50,
    "주기", "FRONT_STATE_TX_PERIOD_MS")
add("Front_H723", "RX", "Rear_H723", "UDP", "5103", "REAR_STATUS", 12, 50,
    "주기", "REAR_STATUS_TIMEOUT_MS=300")

# ---- Rear H723 : CAN250 (FDCAN2) <-> F446 ----
for i, s in [("0x320", "led_left"), ("0x321", "led_right"), ("0x322", "led_brake")]:
    add("Rear_H723", "TX", "F446", "CAN 250k", i, s, 1, 100,
        "이벤트+주기(최소간격10ms)", "LED_CMD_TX_PERIOD_MS")
add("Rear_H723", "TX", "F446", "CAN 250k", "0x323", "window [수신자없음]", 1, 100,
    "주기", "WINDOW_CMD_TX_PERIOD_MS")
add("Rear_H723", "TX", "F446", "CAN 250k", "0x201", "ping", 8, 500,
    "주기", "CAN250_TX_PERIOD_MS")
add("Rear_H723", "RX", "F446", "CAN 250k", "0x301", "IMU 쿼터니언", 8, 0,
    "비주기(BNO085 리포트) 실측 약 47Hz", "busload 실측")
add("Rear_H723", "RX", "F446", "CAN 250k", "0x311", "switch_pressed", 1, 100,
    "주기", "SWITCH_TX_PERIOD_MS")
add("Rear_H723", "RX", "F446", "CAN 250k", "0x312", "ultrasonic_mm", 2, 60,
    "주기", "ULTRASONIC_PERIOD_MS")
add("Rear_H723", "RX", "F446", "CAN 250k", "0x281", "ping_echo", 8, 500,
    "요청 응답", "Node_F447_Real")

# ---- Rear H723 : CAN500 (FDCAN1) <-> RearBody ----
add("Rear_H723", "TX", "RearBody_F407", "CAN 500k", "0x101", "ping", 8, 500,
    "주기", "CAN500_TX_PERIOD_MS")
add("Rear_H723", "RX", "RearBody_F407", "CAN 500k", "0x181", "ping_echo", 8, 500,
    "요청 응답", "RearBody_F407_real")

# ---- Rear H723 : UDP ----
add("Rear_H723", "RX", "Jetson", "UDP", "5001", "REAR_COMMAND", 12, 0,
    "이벤트", "zone_udp_protocol.h")
add("Rear_H723", "TX", "Jetson", "UDP", "5001", "ACK 반향", 12, 0,
    "수신 시 즉시", "COMM_ICD 2.6")
add("Rear_H723", "TX", "Jetson", "UDP", "5002", "'SR' 텔레메트리", 48, 100,
    "이벤트+주기(최소간격10ms)", "RT_PERIOD_MS")
add("Rear_H723", "TX", "Jetson", "UDP", "5104", "'SP' perf", 104, 1000,
    "주기", "RT_PERF_PERIOD_MS")
add("Rear_H723", "TX", "Jetson", "UDP", "5105", "'SD' DTC", 232, 1000,
    "상태변화 시 즉시+하트비트", "diagnostic.c")
add("Rear_H723", "RX", "Front_H723", "UDP", "5003", "FRONT_STATE", 12, 50,
    "주기", "FRONT_TO_REAR_TIMEOUT_MS=300")
add("Rear_H723", "TX", "Front_H723", "UDP", "5103", "REAR_STATUS", 12, 50,
    "주기", "rear_zone_network.c")

# ---- F407_Front (하위) ----
for i, s, n, p in _f_rx[:-1]:
    add("F407_Front", "TX", "Front_H723", "CAN 250k", i, s, n, p,
        "주기", "CAN_PERIOD_*")
add("F407_Front", "TX", "Front_H723", "CAN 250k", "0x281", "ping_echo", 8, 500,
    "요청 응답", "main.c")
for i, s, n in _f_cmd:
    add("F407_Front", "RX", "Front_H723", "CAN 250k", i, s, n, 100,
        "이벤트+주기", "main.c")
add("F407_Front", "RX", "Front_H723", "CAN 250k", "0x201", "ping", 8, 500,
    "주기", "main.c")
add("F407_Front", "내부", "DHT11", "GPIO", "-", "온습도 판독", 5, 2000,
    "주기 (IRQ 4.1ms 차단)", "DHT11_READ_PERIOD_MS")
add("F407_Front", "내부", "액추에이터", "GPIO", "-", "제어 갱신", 0, 10,
    "주기", "CONTROL_UPDATE_PERIOD_MS")

# ---- F446 (Node_F447_Real) ----
add("F446", "TX", "Rear_H723", "CAN 250k", "0x301", "IMU 쿼터니언", 8, 0,
    "비주기(BNO085)", "main.c")
add("F446", "TX", "Rear_H723", "CAN 250k", "0x311", "switch_pressed", 1, 100,
    "주기", "SWITCH_TX_PERIOD_MS")
add("F446", "TX", "Rear_H723", "CAN 250k", "0x312", "ultrasonic_mm", 2, 60,
    "주기", "ULTRASONIC_PERIOD_MS")
add("F446", "TX", "Rear_H723", "CAN 250k", "0x281", "ping_echo", 8, 500,
    "요청 응답", "main.c")
for i, s in [("0x320", "led_left"), ("0x321", "led_right"), ("0x322", "led_brake")]:
    add("F446", "RX", "Rear_H723", "CAN 250k", i, s, 1, 100, "이벤트+주기", "main.c")
add("F446", "RX", "Rear_H723", "CAN 250k", "0x201", "ping", 8, 500, "주기", "main.c")

# ---- RearBody_F407 ----
add("RearBody_F407", "RX", "Rear_H723", "CAN 500k", "0x101", "ping", 8, 500,
    "주기", "REAR_PING_500_RX_ID")
add("RearBody_F407", "TX", "Rear_H723", "CAN 500k", "0x181", "ping_echo", 8, 500,
    "요청 응답", "REAR_PING_500_TX_ID")

# --------------------------------------------------------------------
# 2) 실행시간 / 지연 측정값
# --------------------------------------------------------------------
EXE = [
    # board, 구간, 종류, min_us, max_us, avg_us, 표본, 측정여부, 비고
    ["Front_H723", "메인루프 1바퀴", "주기", 53, 539, 72, 9272705, "실측",
     "슈퍼루프. 약 13,900 Hz"],
    ["Front_H723", "MX_LWIP_Process()", "실행시간", 4, 293, 20, 8478120, "실측",
     "RTOS 판에서 측정. 슈퍼루프도 유사"],
    ["Front_H723", "FDCAN RX ISR", "실행시간", 7, 13, "", 135606, "실측",
     "METRICS 6절 baseline"],
    ["Front_H723", "지령변화->CAN송신", "지연", 16, 79, 29, 302, "실측",
     "이벤트 트리거 적용 후"],
    ["Front_H723", "지령변화->종단반영", "지연", 662, 29791, 6911, 300, "실측",
     "CAN 왕복+텔레메트리 포함"],
    ["Front_H723", "ACK RTT (Jetson왕복)", "지연", 201, 1798, 456, 2000, "실측",
     "p50 251 / p95 903 / p99 975 us"],
    ["Rear_H723", "지령변화->CAN송신", "지연", 17, 18, 17, 300, "실측",
     "이벤트 트리거 적용 후"],
    ["Rear_H723", "지령변화->텔레메트리", "지연", 80, 99, 82, 300, "실측",
     "RearBody 피드백 없어 지령만 관측"],
    ["Rear_H723", "메인루프 1바퀴", "주기", "", "", "", "", "미측정",
     "perf_loop_period 존재. 읽기만 하면 됨"],
    ["F407_Front", "DHT11 IRQ 차단", "차단시간", "", 4098, "", "", "실측",
     "개선 전 23085 us"],
    ["Front_H723", "App_CanStep()", "실행시간", "", "", "", "", "미측정",
     "함수 단위 계측 미도입"],
    ["Front_H723", "App_NetStep()", "실행시간", "", "", "", "", "미측정",
     "함수 단위 계측 미도입"],
    ["Front_H723", "FrontTelemetry_Process()", "실행시간", "", "", "", "", "미측정",
     "UDP 송신 포함 구간"],
    ["Front_H723", "Front_ReportDiagnostics()", "실행시간", "", "", "", "", "미측정",
     "DTC 판정 5건"],
    ["Rear_H723", "Rear_ReportDiagnostics()", "실행시간", "", "", "", "", "미측정",
     "DTC 판정 7건"],
    ["Rear_H723", "FDCAN RX ISR", "실행시간", "", "", "", "", "미측정",
     "Front 는 7~13 us. 유사 예상"],
    ["F446", "초음파 측정", "실행시간", "", "", "", "", "미측정",
     "에코 대기 블로킹 가능성"],
    ["전 보드", "플래시 섹터 소거(128KB)", "차단시간", "", "", "", "", "미측정",
     "RTOS 필요성 판단의 핵심 값"],
]

os.makedirs(OUT, exist_ok=True)

p1 = os.path.join(OUT, "signals.csv")
with io.open(p1, "w", encoding="utf-8-sig", newline="") as f:
    w = csv.writer(f)
    w.writerow(["보드", "방향", "상대", "채널", "ID/포트", "신호",
                "바이트", "주기_ms", "트리거", "근거"])
    w.writerows(SIG)

p2 = os.path.join(OUT, "exec_time.csv")
with io.open(p2, "w", encoding="utf-8-sig", newline="") as f:
    w = csv.writer(f)
    w.writerow(["보드", "구간", "종류", "min_us", "max_us", "avg_us",
                "표본수", "측정여부", "비고"])
    w.writerows(EXE)

print("signals.csv    %d행" % len(SIG))
print("exec_time.csv  %d행  (실측 %d / 미측정 %d)"
      % (len(EXE),
         sum(1 for r in EXE if r[7] == "실측"),
         sum(1 for r in EXE if r[7] == "미측정")))
