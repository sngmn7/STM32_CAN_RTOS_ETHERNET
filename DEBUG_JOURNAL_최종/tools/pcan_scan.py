#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pcan_scan.py — PCAN-USB 로 버스를 리슨온리로 훑는다.

모르는 노드가 어떤 비트레이트로 무슨 ID 를 쏘는지 알아내기 위한 도구다.

  * 리슨온리(listen-only) 로만 연다. 한 비트도 송신하지 않고 ACK 도 주지
    않는다. 비트레이트를 잘못 짚어도 버스를 망가뜨리지 않는다.
  * 에러 프레임도 받도록 열어둔다. 비트레이트가 틀리면 정상 프레임 대신
    에러가 쏟아지므로, 그 비율로 맞는 속도를 가려낼 수 있다.

python-can 을 쓰지 않고 PCANBasic.dll 을 직접 부른다. 설치할 것이 없다.

    python pcan_scan.py                 # 기본 속도 목록을 훑는다
    python pcan_scan.py -b 500          # 500k 만
    python pcan_scan.py -t 5            # 속도당 5초
"""

import argparse
import ctypes
import sys
import time
from ctypes import (POINTER, Structure, byref, c_char, c_ubyte, c_ulong,
                    c_ushort)

# ------------------------------------------------------------------ #
# PCANBasic 상수                                                      #
# ------------------------------------------------------------------ #
PCAN_USBBUS = [0x51 + i for i in range(8)]      # PCAN_USBBUS1..8

PCAN_ERROR_OK = 0x00000
PCAN_ERROR_QRCVEMPTY = 0x00020
PCAN_ERROR_BUSLIGHT = 0x00001
PCAN_ERROR_BUSHEAVY = 0x00002
PCAN_ERROR_BUSPASSIVE = 0x40000
PCAN_ERROR_BUSOFF = 0x00008

PCAN_LISTEN_ONLY = 0x08
PCAN_ALLOW_ERROR_FRAMES = 0x21
PCAN_RECEIVE_STATUS = 0x15
PCAN_PARAMETER_ON = 0x01
PCAN_CHANNEL_CONDITION = 0x0B
PCAN_CHANNEL_AVAILABLE = 0x01

MSG_STANDARD = 0x00
MSG_RTR = 0x01
MSG_EXTENDED = 0x02
MSG_ERRFRAME = 0x40
MSG_STATUS = 0x80

# Btr0Btr1 코드 (SJA1000 호환). PCANBasic.h 의 PCAN_BAUD_* 값.
BAUD = [
    ("1M",   0x0014),
    ("800k", 0x0016),
    ("500k", 0x001C),
    ("250k", 0x011C),
    ("125k", 0x031C),
    ("100k", 0x432F),
    ("83k",  0x852B),
    ("50k",  0x472F),
    ("33k",  0x8B2F),
    ("20k",  0x532F),
    ("10k",  0x672F),
]


class TPCANMsg(Structure):
    _fields_ = [("ID", c_ulong),
                ("MSGTYPE", c_ubyte),
                ("LEN", c_ubyte),
                ("DATA", c_ubyte * 8)]


class TPCANTimestamp(Structure):
    _fields_ = [("millis", c_ulong),
                ("millis_overflow", c_ushort),
                ("micros", c_ushort)]


def load_dll():
    for name in ("PCANBasic.dll", r"C:\Windows\System32\PCANBasic.dll"):
        try:
            return ctypes.windll.LoadLibrary(name)
        except OSError:
            continue
    print("PCANBasic.dll 을 못 찾았다. PEAK 드라이버가 설치돼 있는지 확인.")
    sys.exit(2)


def find_channel(dll):
    """붙어 있는 첫 PCAN-USB 채널을 돌려준다."""
    for ch in PCAN_USBBUS:
        cond = c_ulong(0)
        if dll.CAN_GetValue(c_ushort(ch), c_ubyte(PCAN_CHANNEL_CONDITION),
                            byref(cond), 4) == PCAN_ERROR_OK:
            if cond.value & PCAN_CHANNEL_AVAILABLE:
                return ch
    return None


def err_text(dll, code):
    buf = ctypes.create_string_buffer(256)
    if dll.CAN_GetErrorText(c_ulong(code), c_ushort(0x09), buf) == PCAN_ERROR_OK:
        return buf.value.decode("latin-1", "replace").strip()
    return "0x%X" % code


def scan_one(dll, ch, label, btr, secs):
    """한 비트레이트로 secs 초 듣는다. (프레임표, 통계) 반환."""
    st = dll.CAN_Initialize(c_ushort(ch), c_ushort(btr), c_ubyte(0),
                            c_ulong(0), c_ushort(0))
    if st != PCAN_ERROR_OK:
        return None, {"init 실패": err_text(dll, st)}

    on = c_ulong(PCAN_PARAMETER_ON)
    # 순서 중요 — 초기화 뒤에 설정해야 먹는다.
    dll.CAN_SetValue(c_ushort(ch), c_ubyte(PCAN_LISTEN_ONLY), byref(on), 4)
    dll.CAN_SetValue(c_ushort(ch), c_ubyte(PCAN_ALLOW_ERROR_FRAMES),
                     byref(on), 4)
    dll.CAN_SetValue(c_ushort(ch), c_ubyte(PCAN_RECEIVE_STATUS), byref(on), 4)

    frames = {}          # id -> [개수, ext, rtr, dlc, 마지막데이터, 최초t, 최종t]
    stats = {"정상": 0, "에러프레임": 0, "상태프레임": 0, "버스에러": 0}
    msg = TPCANMsg()
    ts = TPCANTimestamp()

    end = time.time() + secs
    while time.time() < end:
        rc = dll.CAN_Read(c_ushort(ch), byref(msg), byref(ts))
        if rc == PCAN_ERROR_QRCVEMPTY:
            time.sleep(0.001)
            continue
        if rc != PCAN_ERROR_OK:
            if rc & (PCAN_ERROR_BUSLIGHT | PCAN_ERROR_BUSHEAVY |
                     PCAN_ERROR_BUSPASSIVE | PCAN_ERROR_BUSOFF):
                stats["버스에러"] += 1
            continue

        t = time.time()
        if msg.MSGTYPE & MSG_ERRFRAME:
            stats["에러프레임"] += 1
            continue
        if msg.MSGTYPE & MSG_STATUS:
            stats["상태프레임"] += 1
            continue

        stats["정상"] += 1
        key = (msg.ID, bool(msg.MSGTYPE & MSG_EXTENDED))
        d = bytes(msg.DATA[:msg.LEN])
        if key in frames:
            f = frames[key]
            f[0] += 1
            f[3] = msg.LEN
            f[4] = d
            f[6] = t
        else:
            frames[key] = [1, key[1], bool(msg.MSGTYPE & MSG_RTR),
                           msg.LEN, d, t, t]

    dll.CAN_Uninitialize(c_ushort(ch))
    return frames, stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bitrate", help="특정 속도만 (예: 500 또는 500k)")
    ap.add_argument("-t", "--secs", type=float, default=3.0,
                    help="속도당 관측 시간(초). 기본 3")
    args = ap.parse_args()

    dll = load_dll()
    ch = find_channel(dll)
    if ch is None:
        print("PCAN-USB 를 못 찾았다. 어댑터를 꽂고 다시 실행.")
        sys.exit(1)
    print("채널 0x%02X 사용, 리슨온리 (송신·ACK 안 함)\n" % ch)

    table = BAUD
    if args.bitrate:
        want = args.bitrate.lower().rstrip("k") + "k"
        table = [b for b in BAUD if b[0] == want]
        if not table:
            print("모르는 속도:", args.bitrate)
            print("가능:", ", ".join(b[0] for b in BAUD))
            sys.exit(2)

    hits = []
    for label, btr in table:
        frames, stats = scan_one(dll, ch, label, btr, args.secs)
        if frames is None:
            print("%-6s %s" % (label, stats))
            continue
        n = stats["정상"]
        e = stats["에러프레임"] + stats["버스에러"]
        mark = "  <<< 유력" if (n > 0 and e == 0) else ""
        print("%-6s  정상 %-5d  에러 %-5d  ID %-3d%s"
              % (label, n, e, len(frames), mark))
        if frames:
            hits.append((label, frames))

    if not hits:
        print("\n아무것도 못 받았다. 다음을 의심한다.")
        print("  - 노드가 주기 송신을 안 하고 요청에만 응답한다")
        print("  - 통신 방식이 CAN 으로 설정돼 있지 않다")
        print("  - 배선(CANH/CANL) 또는 종단저항")
        return

    print("\n" + "=" * 62)
    for label, frames in hits:
        print("[%s]" % label)
        print("  ID        개수   DLC 주기ms  데이터")
        for (cid, ext), f in sorted(frames.items()):
            cnt, _e, rtr, dlc, data, t0, t1 = f
            per = ((t1 - t0) * 1000.0 / (cnt - 1)) if cnt > 1 else 0.0
            print("  %s%-8X %-6d %-3d %-7.1f %s%s"
                  % ("x" if ext else " ", cid, cnt, dlc, per,
                     " ".join("%02X" % b for b in data),
                     "  (RTR)" if rtr else ""))


if __name__ == "__main__":
    main()
