#!/usr/bin/env python3
"""
busload.py - CAN 버스 부하율 실측

각 노드의 프레임 카운터를 ST-Link 로 두 번 읽어 프레임/초를 구하고,
DLC 별 프레임 비트수를 곱해 대역폭으로 나눈다.

프레임 비트수 (Standard CAN 2.0A, N 데이터바이트)
  고정부  SOF 1 + ID 11 + RTR 1 + IDE 1 + r0 1 + DLC 4
          + CRC 15 + CRCdel 1 + ACK 2 + EOF 7        = 44
  데이터  8N
  IFS                                                =  3
  -> 공칭 = 47 + 8N

  비트스터핑은 SOF~CRC 구간(34 + 8N 비트)에 적용되고
  최악으로 floor((34 + 8N - 1) / 4) 비트가 추가된다.
  -> 최악 = 공칭 + floor((33 + 8N) / 4)

    DLC1  공칭 55  최악 65
    DLC2  공칭 63  최악 75
    DLC8  공칭 111 최악 135
"""

import subprocess
import sys
import time

CLI = (r"C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins"
       r"\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304"
       r"\tools\bin\STM32_Programmer_CLI.exe")
NM = None   # 아래에서 채움

import glob, os
for c in glob.glob(r"C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins"
                   r"\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*"
                   r"\tools\bin\arm-none-eabi-nm.exe"):
    NM = c
    break

WS2 = r"C:\Users\PC\STM32CubeIDE\workspace_2.2.0"
WS1 = r"c:\stm_workspace"

BOARDS = {
    "front_h723": (WS1 + r"\ZoneController_Front_H723\Debug\ZoneController_Front_H723.elf",
                   "003F00343233510739363634"),
    "front_f407": (WS1 + r"\F407_Front\Debug\F407_Front.elf",
                   "066EFF575377524867034154"),
    "rear_h723":  (WS2 + r"\ZoneController_Rear_H723_real\Debug\ZoneController_Rear_H723_real.elf",
                   "004900343234510237333934"),
    "f446":       (WS2 + r"\Node_F447_Real\Debug\Node_F447_Real.elf",
                   "0668FF515086874967255335"),
    "rearbody":   (WS2 + r"\RearBody_F407_real\Debug\RearBody_F407_real.elf",
                   "066CFF525377524867034749"),
}

_symcache = {}


def sym(board, name):
    key = (board, name)
    if key in _symcache:
        return _symcache[key]
    elf = BOARDS[board][0]
    out = subprocess.run([NM, "--defined-only", elf],
                         capture_output=True, text=True).stdout
    addr = None
    for line in out.splitlines():
        f = line.split()
        if len(f) == 3 and f[2] == name:
            addr = "0x" + f[0]
            break
    _symcache[key] = addr
    return addr


def read32(board, addr):
    sn = BOARDS[board][1]
    out = subprocess.run(
        [CLI, "-c", "port=SWD", "sn=" + sn, "mode=HOTPLUG", "-r32", addr, "4"],
        capture_output=True, text=True).stdout
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("0x") and " : " in line:
            return int(line.split(":")[1].split()[0], 16)
    return None


def bits(dlc, worst):
    nominal = 47 + 8 * dlc
    if not worst:
        return nominal
    return nominal + (33 + 8 * dlc) // 4


# 버스 정의: (이름, 보레이트, [(설명, 보드, 심볼, DLC), ...])
BUSES = [
    ("FRONT 250k (H723 FDCAN2 <-> F407_Front)", 250000, [
        ("H723->F407 명령 0x310~0x316", "front_h723", "front_command_tx_count", 1),
        ("H723->F407 핑 0x201",         "front_h723", "can250_tx_count",        8),
        ("F407->H723 조향 0x210",       "front_f407", "can_steering_tx_count",  2),
        ("F407->H723 브레이크 0x211",   "front_f407", "can_brake_tx_count",     2),
        ("F407->H723 가속 0x212",       "front_f407", "can_accel_tx_count",     2),
        ("F407->H723 방향좌 0x213",     "front_f407", "can_turn_left_tx_count", 1),
        ("F407->H723 방향우 0x214",     "front_f407", "can_turn_right_tx_count", 1),
        ("F407->H723 스위치 0x215",     "front_f407", "can_turn_switch_tx_count", 1),
        ("F407->H723 전조등 0x216",     "front_f407", "can_headlamp_output_tx_count", 1),
        ("F407->H723 온도 0x217",       "front_f407", "can_temperature_tx_count", 2),
        ("F407->H723 습도 0x218",       "front_f407", "can_humidity_tx_count",  2),
        ("F407->H723 DHT유효 0x219",    "front_f407", "can_dht11_valid_tx_count", 1),
        ("F407->H723 액추위치 0x21A",   "front_f407", "can_actuator_position_tx_count", 2),
        ("F407->H723 액추허용 0x21B",   "front_f407", "can_actuator_enable_tx_count", 1),
        ("F407->H723 조향OR 0x21C",     "front_f407", "can_steering_override_tx_count", 1),
        ("F407->H723 전조OR 0x21D",     "front_f407", "can_headlamp_override_tx_count", 1),
        ("F407->H723 버스오프 0x21E",   "front_f407", "can_busoff_signal_tx_count", 1),
        ("F407->H723 핑에코 0x281",     "front_f407", "can_echo_tx_count",      8),
    ]),
    ("REAR 250k (H723 FDCAN2 <-> F446)", 250000, [
        ("H723->F446 LED 0x320~0x322",  "rear_h723", "led_cmd_tx_count",       1),
        ("H723->F446 창문 0x323 [고아]", "rear_h723", "window_cmd_tx_count",    1),
        ("H723->F446 핑 0x201",         "rear_h723", "can250_tx_count",        8),
        ("F446->H723 IMU 0x301",        "f446",      "imu_tx_count",           8),
        ("F446->H723 핑에코 0x281",     "f446",      "can250_tx_count",        8),
    ]),
    ("REAR 500k (H723 FDCAN1 <-> RearBody)", 500000, [
        ("H723->RearBody 핑 0x101",     "rear_h723", "can500_tx_count",        8),
        ("RearBody->H723 에코 0x181",   "rearbody",  "can250_reply_count",     8),
    ]),
]


def main():
    window = int(sys.argv[1]) if len(sys.argv) > 1 else 60

    plan = []
    for bus, baud, sigs in BUSES:
        for desc, board, name, dlc in sigs:
            a = sym(board, name)
            plan.append((bus, baud, desc, board, name, dlc, a))

    missing = [p for p in plan if p[6] is None]
    for m in missing:
        print("  심볼 없음: %s.%s (%s)" % (m[3], m[4], m[2]))
    plan = [p for p in plan if p[6] is not None]

    print("### CAN 버스 부하 실측  (창 %ds)" % window)
    print()
    t0 = time.time()
    first = {(p[3], p[4]): read32(p[3], p[6]) for p in plan}
    mid = time.time()
    time.sleep(max(0.0, window - (mid - t0)))
    second = {(p[3], p[4]): read32(p[3], p[6]) for p in plan}
    elapsed = time.time() - t0

    for bus, baud, sigs in BUSES:
        rows = []
        tot_nom = tot_worst = tot_fps = 0.0
        for desc, board, name, dlc in sigs:
            k = (board, name)
            if k not in first or first[k] is None or second[k] is None:
                continue
            d = second[k] - first[k]
            fps = d / elapsed
            bn = bits(dlc, False) * fps
            bw = bits(dlc, True) * fps
            rows.append((desc, dlc, d, fps, bn, bw))
            tot_nom += bn
            tot_worst += bw
            tot_fps += fps

        print("=" * 74)
        print("%s   %d kbps" % (bus, baud // 1000))
        print("-" * 74)
        print("  %-30s DLC %8s %8s %8s" % ("신호", "프레임", "Hz", "부하%"))
        for desc, dlc, d, fps, bn, bw in rows:
            print("  %-30s  %d  %8d %8.1f %8.3f" %
                  (desc, dlc, d, fps, bw * 100.0 / baud))
        print("-" * 74)
        print("  %-30s     %8d %8.1f" % ("합계", sum(r[2] for r in rows), tot_fps))
        print("  공칭 부하 (스터핑 없음)        %6.2f %%" % (tot_nom * 100.0 / baud))
        print("  최악 부하 (스터핑 최대)        %6.2f %%" % (tot_worst * 100.0 / baud))
        print()


if __name__ == "__main__":
    main()
