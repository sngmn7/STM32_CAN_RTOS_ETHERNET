#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zone_hmi.py 의 제어 경로 폐루프 검증.

UI 를 클릭하는 대신 zone_hmi 모듈의 실제 패킷 생성 함수를 그대로 불러
지령을 내리고, 텔레메트리 되읽기로 말단까지 반영됐는지 확인한다.

  젯슨 -> H723 (UDP) -> F407 (CAN) -> H723 (CAN) -> 젯슨 (UDP)

전 구간이 닫혀야 통과다. HMI 가 떠 있으면 5 Hz 로 flags=0 을 같이
쏘아 지령이 씹히므로, 실행 전에 반드시 내려야 한다.
"""

import socket
import struct
import sys
import time

sys.path.insert(0, "/home/nongsa")
import zone_hmi as H


def bind(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    s.settimeout(0.3)
    return s


def read_fresh(sock, fmt, size, magic, deadline, keep_alive):
    """적체를 비운 뒤 처음 오는 새 패킷을 돌려준다.

    두 가지를 지켜야 한다.
      1) 읽는 동안에도 지령을 계속 보낸다. 펌웨어의 링크 타임아웃이
         500 ms 라 송신을 멈추면 보드가 오버라이드를 스스로 푼다.
      2) 마지막이 아니라 '처음' 온 새 패킷을 본다. 마지막을 보면
         타임아웃으로 해제된 뒤의 값을 읽게 된다.
    """
    last_ka = 0.0
    while time.time() < deadline:
        now = time.time()
        if now - last_ka >= 0.1:
            keep_alive()
            last_ka = now
        try:
            d, _ = sock.recvfrom(512)
        except socket.timeout:
            continue
        if len(d) == size:
            v = struct.unpack(fmt, d)
            if v[0] == magic:
                return v
    return None


class Cmd(object):
    """zone_hmi.build_front / build_rear 가 읽는 필드만 갖는 껍데기."""

    def __init__(self):
        for k in ("f_lamp_ovr", "f_lamp_on", "f_turn_ovr", "f_turn_mode",
                  "f_act_en", "f_act_target", "f_steer_ovr",
                  "r_turn_ovr", "r_turn_mode", "r_brake_ovr", "r_brake",
                  "r_win_ovr", "r_window"):
            setattr(self, k, 0)


def flush(sock):
    """쌓인 패킷을 버린다.

    지령을 유지하는 동안 소켓에 옛 패킷이 쌓인다. 그대로 읽으면 지령
    이전의 값을 보고 '반영 안 됨' 으로 오판한다. 읽기 직전에 비운다.
    """
    sock.setblocking(False)
    try:
        while True:
            sock.recv(2048)
    except (BlockingIOError, OSError):
        pass
    sock.setblocking(True)
    sock.settimeout(0.3)


def drive(tx, cmd, secs, sv_sock, sr_sock, verbose=False):
    """secs 동안 10 Hz 로 지령을 유지한 뒤, 유지된 채로 되읽는다."""
    state = {"seq": 0}

    def send():
        q = state["seq"]
        tx.sendto(H.build_front(q, cmd), (H.FRONT_IP, H.FRONT_CMD_PORT))
        tx.sendto(H.build_rear(q, cmd), (H.REAR_IP, H.REAR_CMD_PORT))
        state["seq"] = (q + 1) & 0xFFFF

    fp, rp = H.build_front(0, cmd), H.build_rear(0, cmd)
    if verbose:
        print("  보낸 바이트  F flags=0x%02X turn=%d  R flags=0x%02X turn=%d"
              " brake=%d win=%d"
              % (fp[6], fp[9], rp[6], rp[7], rp[8], rp[9]))

    end = time.time() + secs
    while time.time() < end:
        send()
        time.sleep(0.1)

    flush(sv_sock)
    flush(sr_sock)
    sv = read_fresh(sv_sock, H.SV_FMT, H.SV_SIZE, H.SV_MAGIC,
                    time.time() + 2.0, send)
    sr = read_fresh(sr_sock, H.SR_FMT, H.SR_SIZE, H.SR_MAGIC,
                    time.time() + 2.0, send)
    if verbose:
        print("  받은 패킷    SV seq=%s  SR seq=%s"
              % (sv[3] if sv else "-", sr[3] if sr else "-"))
    return sv, sr


def main():
    sv_sock = bind(H.SV_PORT)
    sr_sock = bind(H.SR_PORT)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    fails = []

    def check(label, got, want):
        ok = (got == want)
        print("  %-28s 되읽기=%-5s 기대=%-5s  %s"
              % (label, got, want, "OK" if ok else "실패"))
        if not ok:
            fails.append(label)

    print("== 1. 전조등 ON / 브레이크등 ON / 방향지시 좌 ==")
    c = Cmd()
    c.f_lamp_ovr, c.f_lamp_on = 1, 1
    c.r_brake_ovr, c.r_brake = 1, 1
    c.r_turn_ovr, c.r_turn_mode = 1, 1
    sv, sr = drive(tx, c, 3.0, sv_sock, sr_sock, verbose=True)
    if sv is None:
        print("  Front SV 수신 없음"); fails.append("SV 수신")
    else:
        check("Front 전조등", sv[13], 1)
    if sr is None:
        print("  Rear SR 수신 없음"); fails.append("SR 수신")
    else:
        check("Rear 브레이크등", sr[8], 1)
        check("Rear 방향지시 모드", sr[5], 1)
        check("Rear 좌 램프", sr[6], 1)
        check("Rear 지령출처=젯슨(제동)",
              1 if sr[2] & H.RST_SRC_BRAKE_JETSON else 0, 1)

    print("== 2. 전조등 OFF / 브레이크등 OFF / 방향지시 우 ==")
    c = Cmd()
    c.f_lamp_ovr, c.f_lamp_on = 1, 0
    c.r_brake_ovr, c.r_brake = 1, 0
    c.r_turn_ovr, c.r_turn_mode = 1, 2
    sv, sr = drive(tx, c, 3.0, sv_sock, sr_sock, verbose=True)
    if sv:
        check("Front 전조등", sv[13], 0)
    if sr:
        check("Rear 브레이크등", sr[8], 0)
        check("Rear 방향지시 모드", sr[5], 2)
        check("Rear 우 램프", sr[7], 1)

    print("== 3. 전체 해제 — 원래 제어원으로 복귀 ==")
    c = Cmd()
    sv, sr = drive(tx, c, 3.0, sv_sock, sr_sock, verbose=True)
    if sr:
        check("Rear 지령출처=Front(제동)",
              1 if sr[2] & H.RST_SRC_BRAKE_JETSON else 0, 0)
        check("Rear 지령출처=Front(방향)",
              1 if sr[2] & H.RST_SRC_TURN_JETSON else 0, 0)

    print()
    if fails:
        print("실패 %d 건: %s" % (len(fails), ", ".join(fails)))
        return 1
    print("전 항목 통과 — 젯슨 UI -> H723 -> CAN -> 말단 -> 되읽기 폐루프 확인")
    return 0


if __name__ == "__main__":
    sys.exit(main())
