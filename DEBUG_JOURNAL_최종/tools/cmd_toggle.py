#!/usr/bin/env python3
"""
cmd_toggle.py - 하행 명령 값 변화를 생성한다.

H723 의 CAN 명령 스케줄러는 신호당 100 ms 주기다. 값이 바뀐 뒤 실제로
CAN 으로 나가기까지 얼마나 기다리는지 재려면, 스케줄러 위상에 고르게
분포하는 변화를 만들어야 한다. 그래서 토글 간격을 100 ms 의 배수가 아닌
값으로 잡고 약간의 난수를 섞는다.

전조등만 토글한다. 액추에이터/조향은 건드리지 않는다.

    python3 cmd_toggle.py -n 300
"""

import argparse
import random
import socket
import time

FRONT_IP = "192.168.1.201"
CMD_PORT = 5101

ZONE_MAGIC0 = 0x5A
ZONE_MAGIC_JETSON = 0x4A
ZONE_VERSION = 1
ZONE_MSG_FRONT_COMMAND = 0x20

FLAG_HEADLAMP_OVERRIDE = 1 << 2
FLAG_HEADLAMP_ON = 1 << 3


def build(seq, flags):
    d = bytearray(12)
    d[0] = ZONE_MAGIC0
    d[1] = ZONE_MAGIC_JETSON
    d[2] = ZONE_VERSION
    d[3] = ZONE_MSG_FRONT_COMMAND
    d[4] = seq & 0xFF
    d[5] = (seq >> 8) & 0xFF
    d[6] = flags & 0xFF
    c = 0
    for i in range(11):
        c ^= d[i]
    d[11] = c
    return bytes(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--count", type=int, default=300)
    ap.add_argument("--hold", action="store_true",
                    help="토글 없이 명령만 유지 (대조군)")
    args = ap.parse_args()

    random.seed(20260801)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    on = 0
    t0 = time.time()

    for i in range(args.count):
        if not args.hold:
            on ^= 1
        flags = FLAG_HEADLAMP_OVERRIDE | (FLAG_HEADLAMP_ON if on else 0)
        s.sendto(build(seq, flags), (FRONT_IP, CMD_PORT))
        seq = (seq + 1) & 0xFFFF
        # 100ms 의 배수를 피하고 위상을 흩뜨린다
        time.sleep(0.137 + random.uniform(0.0, 0.05))

    # override 해제 - 원상 복구
    for _ in range(5):
        s.sendto(build(seq, 0), (FRONT_IP, CMD_PORT))
        seq = (seq + 1) & 0xFFFF
        time.sleep(0.05)
    s.close()
    print("토글 %d회, %.1f초" % (args.count, time.time() - t0))


if __name__ == "__main__":
    main()
