#!/usr/bin/env python3
"""
rear_toggle.py - Rear 하행 지령 값 변화를 생성한다.

Rear H723 의 LED/윈도우 지령 송신은 100 ms 주기다. 값 변화 후 실제로
CAN 으로 나가기까지, 그리고 SR 텔레메트리에 실리기까지를 재기 위해
스케줄러 위상에 고르게 분포하는 변화를 만든다.

브레이크등만 토글한다. 윈도우 모터는 건드리지 않는다(0 고정).

    python3 rear_toggle.py -n 300
"""

import argparse
import random
import socket
import time

REAR_IP = "192.168.1.200"
CMD_PORT = 5001

ZONE_MAGIC0 = 0x5A
ZONE_MAGIC_JETSON = 0x4A
ZONE_VERSION = 1
ZONE_MSG_REAR_COMMAND = 0x21

FLAG_TURN_OVERRIDE = 1 << 0
FLAG_BRAKE_OVERRIDE = 1 << 1
FLAG_WINDOW_OVERRIDE = 1 << 2


def build(seq, brake):
    d = bytearray(12)
    d[0] = ZONE_MAGIC0
    d[1] = ZONE_MAGIC_JETSON
    d[2] = ZONE_VERSION
    d[3] = ZONE_MSG_REAR_COMMAND
    d[4] = seq & 0xFF
    d[5] = (seq >> 8) & 0xFF
    d[6] = FLAG_BRAKE_OVERRIDE      # 브레이크만 오버라이드
    d[7] = 0x00                     # 방향지시 OFF
    d[8] = brake & 0x01
    d[9] = 0x00                     # 윈도우 OFF - 모터 건드리지 않음
    c = 0
    for i in range(11):
        c ^= d[i]
    d[11] = c
    return bytes(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--count", type=int, default=300)
    args = ap.parse_args()

    random.seed(20260801)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    brake = 0
    t0 = time.time()

    for _ in range(args.count):
        brake ^= 1
        s.sendto(build(seq, brake), (REAR_IP, CMD_PORT))
        seq = (seq + 1) & 0xFFFF
        # 100 ms 의 배수를 피하고 위상을 흩뜨린다
        time.sleep(0.137 + random.uniform(0.0, 0.05))

    # 오버라이드 해제 - 원상 복구
    for _ in range(5):
        d = bytearray(build(seq, 0))
        d[6] = 0x00
        c = 0
        for i in range(11):
            c ^= d[i]
        d[11] = c
        s.sendto(bytes(d), (REAR_IP, CMD_PORT))
        seq = (seq + 1) & 0xFFFF
        time.sleep(0.05)
    s.close()
    print("토글 %d회, %.1f초" % (args.count, time.time() - t0))


if __name__ == "__main__":
    main()
