#!/usr/bin/env python3
"""
fake_zones.py — Front('SV'/5102) + Rear('SR'/5002) 동시 시뮬레이터

STM32 없이 7페이지 vlcd 전체를 검증한다.

    python3 fake_zones.py --demo             # 두 존 모두 송신
    python3 fake_zones.py --demo --only front
    python3 fake_zones.py --demo --only rear
"""

import argparse
import math
import socket
import struct
import sys
import time

VST_PORT, VST_MAGIC = 5102, 0x5653
RST_PORT, RST_MAGIC = 5002, 0x5253

VFMT = "<HBBIIHHHHhHBBBB"   # 28B
RFMT = "<HBBIIBBBBBBHHhhhhBBIIHHHH"  # 48B
assert struct.calcsize(VFMT) == 28
assert struct.calcsize(RFMT) == 48

# VST flags
F_DHT, F_BUSOFF, F_SW, F_AE, F_SOVR, F_HOVR = (1 << i for i in range(6))
# RST source
S_FLINK, S_TJET, S_BJET, S_JLINK = (1 << i for i in range(4))
# RST sensor
SEN_SW, SEN_ULTRA, SEN_IMU, SEN_SWFRESH = (1 << i for i in range(4))


def build_front(seq, t):
    steering = 2048 + int(1800 * math.sin(t * 0.7))
    accel = 1000 + int(900 * (0.5 + 0.5 * math.sin(t * 0.3)))
    brake = 3000 if (int(t) % 8) >= 5 else 400
    tl = 1 if steering < 1200 else 0
    tr = 1 if steering > 2900 else 0
    hl = 1 if (int(t) // 10) % 2 else 0
    flags = F_DHT | (F_SW if tl or tr else 0) | \
            (F_AE if (int(t) // 5) % 2 else 0)
    temp = int(235 + 30 * math.sin(t * 0.1))
    hum = int(520 + 80 * math.sin(t * 0.13))

    return struct.pack(VFMT, VST_MAGIC, 1, flags, seq,
                       int(t * 1000) & 0xFFFFFFFF,
                       steering, accel, brake, 2048,
                       temp, hum, tl, tr, hl, 0), \
           f"F STR={steering:4d} BRK={brake:4d} L={tl} R={tr}"


def build_rear(seq, t):
    import math as _m

    turn_mode = (int(t) // 5) % 3
    tl = 1 if turn_mode == 1 else 0
    tr = 1 if turn_mode == 2 else 0
    brk = 1 if (int(t) % 8) >= 5 else 0
    win = 1 if (int(t) // 7) % 2 else 0
    jet = (int(t) // 12) % 2
    source = S_FLINK | S_JLINK | \
             (S_TJET if jet else 0) | (S_BJET if jet else 0)
    fbrk = 3000 if brk else 400

    # 초음파: 20cm ~ 200cm 왕복
    dist_mm = int(1100 + 900 * _m.sin(t * 0.5))
    # 토글 스위치: 6초 주기
    sw = 1 if (int(t) // 6) % 2 else 0
    # IMU: yaw 만 천천히 회전하는 쿼터니언 (Q14)
    yaw = t * 0.4
    qr = int(_m.cos(yaw / 2) * 16384)
    qk = int(_m.sin(yaw / 2) * 16384)
    qi = int(200 * _m.sin(t * 0.9))
    qj = int(200 * _m.cos(t * 0.7))

    sensor = SEN_ULTRA | SEN_IMU | SEN_SWFRESH | (SEN_SW if sw else 0)

    return struct.pack(RFMT, RST_MAGIC, 2, source, seq,
                       int(t * 1000) & 0xFFFFFFFF,
                       turn_mode, tl, tr, brk, win,
                       0,                    # health 정상
                       fbrk,
                       dist_mm,
                       qi, qj, qk, qr,
                       sensor, 0,
                       seq * 2, seq * 2,     # can500/can250 rx
                       seq % 10000,          # cmd
                       seq % 10000,          # imu
                       seq % 10000,          # ultra
                       seq % 10000), \
           (f"R turn={turn_mode} brk={brk} dist={dist_mm/10:.0f}cm "
            f"sw={sw} src={'JET' if jet else 'FRT'}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--demo", action="store_true")
    p.add_argument("--only", choices=["front", "rear"])
    p.add_argument("--interval", type=float, default=0.1)
    p.add_argument("--verbose", "-v", action="store_true")
    args = p.parse_args()

    if not args.demo:
        p.print_help()
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    t0 = time.time()

    zones = []
    if args.only in (None, "front"):
        zones.append(("front", VST_PORT, build_front))
    if args.only in (None, "rear"):
        zones.append(("rear", RST_PORT, build_rear))

    print(f"{args.host} 로 송신: " +
          ", ".join(f"{n}:{pt}" for n, pt, _ in zones) +
          "  (Ctrl+C 종료)")

    try:
        while True:
            t = time.time() - t0
            lines = []

            for name, port, builder in zones:
                pkt, desc = builder(seq, t)
                sock.sendto(pkt, (args.host, port))
                lines.append(desc)

            seq += 1

            if args.verbose:
                print(f"seq={seq:5d}  " + "  |  ".join(lines))
            elif seq % 20 == 0:
                print(f"  {seq} 패킷 송신")

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print(f"\n종료. 총 {seq} 주기")

    return 0


if __name__ == "__main__":
    sys.exit(main())
