#!/usr/bin/env python3
"""
rtt_probe.py - Front 명령 왕복 지연(RTT) 분포 측정

두 가지를 따로 잰다.

  A. ACK RTT       : 명령 송신 -> 에코 수신
                     순수 네트워크 + 파싱 지연. 텔레메트리 주기와 무관.
  B. 반영 RTT      : 전조등 토글 명령 -> SV 텔레메트리에 반영된 값이 도착
                     실제 제어 루프가 닫히는 시간. 텔레메트리 주기가 포함된다.

전조등만 조작한다. 액추에이터/조향은 건드리지 않는다(override 비트 off).
종료 시 override 를 해제해 원래 상태로 되돌린다.

    python3 rtt_probe.py -a 1000 -b 200 -o rtt.csv
"""

import argparse
import csv
import socket
import statistics
import struct
import sys
import time

FRONT_IP = "192.168.1.201"
CMD_PORT = 5101
SV_PORT = 5102

ZONE_MAGIC0 = 0x5A
ZONE_MAGIC_JETSON = 0x4A
ZONE_VERSION = 1
ZONE_MSG_FRONT_COMMAND = 0x20

FLAG_HEADLAMP_OVERRIDE = 1 << 2
FLAG_HEADLAMP_ON = 1 << 3

SV_MAGIC = 0x5653
SV_SIZE = 28
SV_FMT = "<HBBIIHHHHhHBBBB"   # 28 B


def build_command(seq, flags=0, actuator=0, turn=0):
    d = bytearray(12)
    d[0] = ZONE_MAGIC0
    d[1] = ZONE_MAGIC_JETSON
    d[2] = ZONE_VERSION
    d[3] = ZONE_MSG_FRONT_COMMAND
    d[4] = seq & 0xFF
    d[5] = (seq >> 8) & 0xFF
    d[6] = flags & 0xFF
    d[7] = actuator & 0xFF
    d[8] = (actuator >> 8) & 0xFF
    d[9] = turn & 0xFF
    d[10] = 0x00
    c = 0
    for i in range(11):
        c ^= d[i]
    d[11] = c
    return bytes(d)


def pct(vals, p):
    if not vals:
        return float("nan")
    v = sorted(vals)
    k = (len(v) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def summarize(name, vals, unit="ms"):
    if not vals:
        print("  %-14s 표본 없음" % name)
        return None
    row = {
        "name": name,
        "n": len(vals),
        "min": min(vals),
        "p50": pct(vals, 50),
        "p95": pct(vals, 95),
        "p99": pct(vals, 99),
        "max": max(vals),
        "mean": statistics.fmean(vals),
        "stdev": statistics.pstdev(vals),
    }
    print("  %-14s n=%-5d min %7.3f  p50 %7.3f  p95 %7.3f  p99 %7.3f  max %8.3f  sigma %6.3f  %s"
          % (name, row["n"], row["min"], row["p50"], row["p95"],
             row["p99"], row["max"], row["stdev"], unit))
    return row


def phase_a(n):
    """ACK RTT: 명령 -> 에코"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 0))
    s.settimeout(0.5)
    rtts = []
    lost = 0
    print("[A] ACK RTT %d회 측정 중..." % n)
    for i in range(n):
        pkt = build_command(i)
        t0 = time.perf_counter()
        s.sendto(pkt, (FRONT_IP, CMD_PORT))
        deadline = t0 + 0.5
        got = False
        while time.perf_counter() < deadline:
            try:
                data, _ = s.recvfrom(2048)
            except socket.timeout:
                break
            t1 = time.perf_counter()
            if len(data) >= 6 and data[4] == (i & 0xFF) and data[5] == ((i >> 8) & 0xFF):
                rtts.append((t1 - t0) * 1000.0)
                got = True
                break
        if not got:
            lost += 1
        time.sleep(0.01)
    s.close()
    return rtts, lost


def phase_b(n):
    """반영 RTT: 전조등 토글 -> SV 에 반영"""
    if n <= 0:
        return [], 0
    cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sv.bind(("0.0.0.0", SV_PORT))
    except OSError as e:
        print("[B] 포트 %d 점유 중(%s) - 반영 RTT 건너뜀" % (SV_PORT, e))
        sv.close()
        cmd.close()
        return [], 0
    sv.settimeout(1.0)

    def read_headlamp(timeout):
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            try:
                data, _ = sv.recvfrom(2048)
            except socket.timeout:
                return None, None
            if len(data) >= SV_SIZE:
                f = struct.unpack(SV_FMT, data[:SV_SIZE])
                if f[0] == SV_MAGIC:
                    return f[13], time.perf_counter()
        return None, None

    cur, _ = read_headlamp(3.0)
    if cur is None:
        print("[B] SV 텔레메트리 미수신 - 건너뜀")
        sv.close()
        cmd.close()
        return [], 0
    print("[B] 반영 RTT %d회 측정 중 (전조등 초기값=%d)..." % (n, cur))

    rtts = []
    lost = 0
    seq = 0
    for i in range(n):
        want = 0 if cur else 1
        flags = FLAG_HEADLAMP_OVERRIDE | (FLAG_HEADLAMP_ON if want else 0)
        # 수신 큐를 비워 직전 주기의 낡은 패킷을 배제
        sv.setblocking(False)
        try:
            while True:
                sv.recvfrom(2048)
        except (BlockingIOError, socket.error):
            pass
        sv.setblocking(True)
        sv.settimeout(1.0)

        t0 = time.perf_counter()
        cmd.sendto(build_command(seq, flags), (FRONT_IP, CMD_PORT))
        seq = (seq + 1) & 0xFFFF

        deadline = t0 + 1.0
        hit = False
        while time.perf_counter() < deadline:
            v, t1 = read_headlamp(max(0.0, deadline - time.perf_counter()))
            if v is None:
                break
            if v == want:
                rtts.append((t1 - t0) * 1000.0)
                cur = want
                hit = True
                break
        if not hit:
            lost += 1
            cur = want
        time.sleep(0.15)

    # override 해제 - 원상 복구
    for _ in range(5):
        cmd.sendto(build_command(seq, 0), (FRONT_IP, CMD_PORT))
        seq = (seq + 1) & 0xFFFF
        time.sleep(0.05)

    sv.close()
    cmd.close()
    return rtts, lost


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-a", "--ack", type=int, default=1000)
    ap.add_argument("-b", "--reflect", type=int, default=200)
    ap.add_argument("-o", "--out", default="rtt.csv")
    args = ap.parse_args()

    print("Front RTT 분포 측정  ->  %s:%d" % (FRONT_IP, CMD_PORT))
    print()

    a_rtts, a_lost = phase_a(args.ack)
    b_rtts, b_lost = phase_b(args.reflect)

    print()
    print("=== 집계 ===")
    rows = []
    r = summarize("ACK RTT", a_rtts)
    if r:
        rows.append(r)
    r = summarize("반영 RTT", b_rtts)
    if r:
        rows.append(r)
    print()
    print("  ACK  손실 %d/%d" % (a_lost, args.ack))
    print("  반영 손실 %d/%d" % (b_lost, args.reflect))

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "n", "min_ms", "p50_ms", "p95_ms",
                    "p99_ms", "max_ms", "mean_ms", "stdev_ms"])
        for row in rows:
            w.writerow([row["name"], row["n"], "%.3f" % row["min"],
                        "%.3f" % row["p50"], "%.3f" % row["p95"],
                        "%.3f" % row["p99"], "%.3f" % row["max"],
                        "%.3f" % row["mean"], "%.3f" % row["stdev"]])
    raw = args.out.replace(".csv", "_raw.csv")
    with open(raw, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "i", "rtt_ms"])
        for i, v in enumerate(a_rtts):
            w.writerow(["ack", i, "%.4f" % v])
        for i, v in enumerate(b_rtts):
            w.writerow(["reflect", i, "%.4f" % v])
    print()
    print("-> %s , %s" % (args.out, raw))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n중단됨")
        sys.exit(1)
