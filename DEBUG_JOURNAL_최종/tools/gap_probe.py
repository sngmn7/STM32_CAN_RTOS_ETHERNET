#!/usr/bin/env python3
"""
gap_probe.py - 통신 공백 길이를 제어하며 U0300 오검출 임계점을 찾는다.

무작위 손실로는 500 + confirm_ms 를 넘는 공백이 거의 안 생겨 표본이 모이지
않는다. 그래서 공백 길이 G 를 직접 만들어 넣고, 그 글리치가 확정(CONFIRMED)
까지 가는지 본다.

각 시행
  1) 손실 없이 WARMUP 초 송신 -> U0300 이 해제될 때까지 기다림
  2) G ms 동안 송신 중단        -> 인위적 글리치
  3) 재개 후 SETTLE 초 관찰      -> 그 사이 CONFIRMED 로 갔는지 기록

기대: 장치는 무송신 500 ms 후 고장으로 보고, 그게 confirm_ms 이상 지속돼야
확정한다. 즉 G > 500 + confirm_ms 일 때만 확정돼야 한다.

    python3 gap_probe.py -g 600,1000,1500,2000,2500,3000 -n 3
"""

import argparse
import socket
import struct
import sys
import threading
import time

FRONT_IP = "192.168.1.201"
CMD_PORT = 5101
DTC_PORT = 5105

ZONE_MAGIC0 = 0x5A
ZONE_MAGIC_JETSON = 0x4A
ZONE_VERSION = 1
ZONE_MSG_FRONT_COMMAND = 0x20

DTC_MAGIC = 0x4453
DTC_PACKET_SIZE = 232
HDR_FMT = "<HBBIIBBBB"
ENTRY_FMT = "<HBBIIHH"   # 16 B: code status occ first last detect reserved

STS_TEST_FAILED = 0x01   # bit0
STS_PENDING     = 0x02
STS_CONFIRMED   = 0x04   # bit2 (0x08 은 WARNING)
STS_WARNING     = 0x08

PERIOD = 0.05          # 20 Hz 송신 - 공백 경계를 또렷하게
WARMUP = 6.0
SETTLE = 4.0

running = True
sending = True


def build_command(seq):
    d = bytearray(12)
    d[0] = ZONE_MAGIC0
    d[1] = ZONE_MAGIC_JETSON
    d[2] = ZONE_VERSION
    d[3] = ZONE_MSG_FRONT_COMMAND
    d[4] = seq & 0xFF
    d[5] = (seq >> 8) & 0xFF
    c = 0
    for i in range(11):
        c ^= d[i]
    d[11] = c
    return bytes(d)


def sender():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    while running:
        if sending:
            try:
                s.sendto(build_command(seq), (FRONT_IP, CMD_PORT))
            except OSError:
                pass
            seq = (seq + 1) & 0xFFFF
        time.sleep(PERIOD)
    s.close()


def u0300(data):
    hs = struct.calcsize(HDR_FMT)
    es = struct.calcsize(ENTRY_FMT)
    magic, ver, zone, seq, uptime, cnt, active, conf, _ = \
        struct.unpack(HDR_FMT, data[:hs])
    if magic != DTC_MAGIC or zone != ord('F'):
        return None
    for i in range(min(cnt, 12)):
        off = hs + i * es
        code, status, occ, first, last, det, _p = \
            struct.unpack(ENTRY_FMT, data[off:off + es])
        if code == 0xC300:
            return status
    return None


def drain(rx, seconds):
    """seconds 동안 수신하며 마지막 status 를 반환."""
    end = time.time() + seconds
    last = None
    while time.time() < end:
        try:
            data, _ = rx.recvfrom(2048)
        except socket.timeout:
            continue
        if len(data) >= DTC_PACKET_SIZE:
            st = u0300(data)
            if st is not None:
                last = st
    return last


def watch(rx, seconds):
    """seconds 동안 관찰하며 CONFIRMED/TEST_FAILED 가 한 번이라도 섰는지."""
    end = time.time() + seconds
    saw_conf = False
    saw_fail = False
    while time.time() < end:
        try:
            data, _ = rx.recvfrom(2048)
        except socket.timeout:
            continue
        if len(data) < DTC_PACKET_SIZE:
            continue
        st = u0300(data)
        if st is None:
            continue
        if st & STS_CONFIRMED:
            saw_conf = True
        if st & STS_TEST_FAILED:
            saw_fail = True
    return saw_conf, saw_fail


def main():
    global running, sending
    ap = argparse.ArgumentParser()
    ap.add_argument("-g", "--gaps", default="600,1000,1500,2000,2500,3000")
    ap.add_argument("-n", "--repeats", type=int, default=3)
    args = ap.parse_args()
    gaps = [int(x) for x in args.gaps.split(",")]

    th = threading.Thread(target=sender, daemon=True)
    th.start()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", DTC_PORT))
    rx.settimeout(0.4)

    results = []
    for g in gaps:
        conf_hits = 0
        fail_hits = 0
        for _ in range(args.repeats):
            # 1) 워밍업 - U0300 이 해제될 때까지
            sending = True
            st = drain(rx, WARMUP)
            # 2) 공백 주입
            sending = False
            time.sleep(g / 1000.0)
            # 3) 재개 후 관찰
            sending = True
            c, f = watch(rx, SETTLE)
            conf_hits += 1 if c else 0
            fail_hits += 1 if f else 0
        results.append((g, conf_hits, fail_hits))
        print("GAP,%d,%d,%d,%d" % (g, conf_hits, fail_hits, args.repeats),
              flush=True)

    running = False
    sending = False
    rx.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        running = False
        sys.exit(1)
