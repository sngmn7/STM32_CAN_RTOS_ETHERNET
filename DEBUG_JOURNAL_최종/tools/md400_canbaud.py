#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""md400_canbaud.py — MD400T 의 CAN 통신속도를 PCAN 으로 바꾼다.

근거: MDROBOT CAN통신사양 V2.1
  PID 137 (0x89) W  PID_ECAN_BITRATE   CAN통신속도 지정
      1:50k  2:100k  3:250k  4:500k  8:125k
      패킷  137, 0xaa, BIT_RATE, x, x, x, x, x
  0xaa(170) = Write check byte (쓰기 보안용 추가 데이터)
  0xfe(254) = ID_ALL, 모든 제어기에 동시 전송

패킷 구조 (같은 문서 §2.1)
  Standard  헤더 11bit = NC(3bit) + ID(1byte)
            데이터 8byte = PID, D0..D6
  Extended  헤더 29bit = NC(5bit) + RMID(183) + TMID(184) + ID
            데이터 8byte = PID, D0..D6

기본 동작은 '읽기만' 이다. 실제로 쓰려면 --write 를 준다.

    python md400_canbaud.py                    # 현재 50k 에서 응답 확인만
    python md400_canbaud.py --from 50k         # 다른 현재 속도로 확인
    python md400_canbaud.py --write 500k       # 500k 로 변경 (되돌리기 번거로움)
"""

import argparse
import ctypes
import sys
import time
from ctypes import Structure, byref, c_ubyte, c_ulong, c_ushort

PCAN_USBBUS = [0x51 + i for i in range(8)]
PCAN_ERROR_OK = 0x00000
PCAN_ERROR_QRCVEMPTY = 0x00020
PCAN_CHANNEL_CONDITION = 0x0B
PCAN_CHANNEL_AVAILABLE = 0x01
PCAN_ALLOW_ERROR_FRAMES = 0x21
PCAN_PARAMETER_ON = 0x01

MSG_STANDARD = 0x00
MSG_EXTENDED = 0x02
MSG_ERRFRAME = 0x40
MSG_STATUS = 0x80

BAUD = {"50k": 0x472F, "100k": 0x432F, "125k": 0x031C,
        "250k": 0x011C, "500k": 0x001C, "1m": 0x0014}

# PID_ECAN_BITRATE 의 데이터 값
BITRATE_CODE = {"50k": 1, "100k": 2, "250k": 3, "500k": 4, "125k": 8}

RMID = 183          # BLDC 모터제어기
TMID = 184          # 사용자제어기
ID_ALL = 0xFE       # 모든 제어기

PID_VER = 1
PID_REQ_PID_DATA = 4
PID_ECAN_BITRATE = 137
WRITE_CHECK = 0xAA


class TPCANMsg(Structure):
    _fields_ = [("ID", c_ulong), ("MSGTYPE", c_ubyte),
                ("LEN", c_ubyte), ("DATA", c_ubyte * 8)]


class TPCANTimestamp(Structure):
    _fields_ = [("millis", c_ulong), ("millis_overflow", c_ushort),
                ("micros", c_ushort)]


def load_dll():
    for n in ("PCANBasic.dll", r"C:\Windows\System32\PCANBasic.dll"):
        try:
            return ctypes.windll.LoadLibrary(n)
        except OSError:
            pass
    sys.exit("PCANBasic.dll 없음. PEAK 드라이버를 설치하라.")


def find_channel(dll):
    for ch in PCAN_USBBUS:
        c = c_ulong(0)
        if dll.CAN_GetValue(c_ushort(ch), c_ubyte(PCAN_CHANNEL_CONDITION),
                            byref(c), 4) == PCAN_ERROR_OK:
            if c.value & PCAN_CHANNEL_AVAILABLE:
                return ch
    return None


def open_bus(dll, ch, label):
    st = dll.CAN_Initialize(c_ushort(ch), c_ushort(BAUD[label]),
                            c_ubyte(0), c_ulong(0), c_ushort(0))
    if st != PCAN_ERROR_OK:
        sys.exit("CAN_Initialize 실패 (0x%X)" % st)
    on = c_ulong(PCAN_PARAMETER_ON)
    dll.CAN_SetValue(c_ushort(ch), c_ubyte(PCAN_ALLOW_ERROR_FRAMES),
                     byref(on), 4)


def send(dll, ch, node_id, payload, extended):
    """payload = [PID, D0..D6] 8바이트."""
    m = TPCANMsg()
    if extended:
        m.ID = (RMID << 16) | (TMID << 8) | node_id
        m.MSGTYPE = MSG_EXTENDED
    else:
        m.ID = node_id
        m.MSGTYPE = MSG_STANDARD
    m.LEN = 8
    for i in range(8):
        m.DATA[i] = payload[i] if i < len(payload) else 0
    st = dll.CAN_Write(c_ushort(ch), byref(m))
    if st != PCAN_ERROR_OK:
        print("  송신 실패 0x%X" % st)
        return False
    return True


def drain(dll, ch, secs):
    """secs 동안 받은 정상 프레임을 리스트로."""
    got, errs = [], 0
    m, ts = TPCANMsg(), TPCANTimestamp()
    end = time.time() + secs
    while time.time() < end:
        rc = dll.CAN_Read(c_ushort(ch), byref(m), byref(ts))
        if rc == PCAN_ERROR_QRCVEMPTY:
            time.sleep(0.001)
            continue
        if rc != PCAN_ERROR_OK:
            errs += 1
            continue
        if m.MSGTYPE & (MSG_ERRFRAME | MSG_STATUS):
            errs += 1
            continue
        got.append((m.ID, bool(m.MSGTYPE & MSG_EXTENDED),
                    bytes(m.DATA[:m.LEN])))
    return got, errs


def probe(dll, ch, node_id, extended, label):
    """PID_VER 를 요청해 응답이 오는지 본다."""
    drain(dll, ch, 0.2)
    send(dll, ch, node_id, [PID_REQ_PID_DATA, PID_VER], extended)
    got, errs = drain(dll, ch, 0.6)
    print("  [%s] 응답 %d개, 에러 %d" % (label, len(got), errs))
    for cid, ext, d in got[:4]:
        print("      %s%X  %s" % ("x" if ext else "", cid,
                                  " ".join("%02X" % b for b in d)))
    return len(got) > 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="cur", default="50k",
                    help="MD400T 의 현재 CAN 속도 (기본 50k)")
    ap.add_argument("--write", metavar="속도",
                    help="변경할 속도 (50k/100k/125k/250k/500k)")
    ap.add_argument("--id", type=lambda x: int(x, 0), default=ID_ALL,
                    help="제어기 ID. 기본 254(ID_ALL, 전체)")
    ap.add_argument("--std", action="store_true",
                    help="Standard(11bit) 로 보낸다. 기본은 Extended(29bit)")
    a = ap.parse_args()

    cur = a.cur.lower()
    if cur not in BAUD:
        sys.exit("모르는 속도: %s (가능: %s)" % (a.cur, ", ".join(BAUD)))
    ext = not a.std

    dll = load_dll()
    ch = find_channel(dll)
    if ch is None:
        sys.exit("PCAN-USB 를 못 찾았다. 어댑터를 꽂아라.")
    print("채널 0x%02X, %s 모드, 대상 ID %d(0x%02X)"
          % (ch, "Extended" if ext else "Standard", a.id, a.id))

    # ---------------- 현재 속도에서 응답 확인 ---------------- #
    print("\n1) 현재 속도 %s 에서 응답 확인" % cur)
    open_bus(dll, ch, cur)
    alive = probe(dll, ch, a.id, ext, cur)
    if not alive:
        print("  응답 없음. 다른 모드/속도도 시도한다.")
        for lbl in ("50k", "100k", "125k", "250k", "500k"):
            if lbl == cur:
                continue
            dll.CAN_Uninitialize(c_ushort(ch))
            open_bus(dll, ch, lbl)
            if probe(dll, ch, a.id, ext, lbl):
                cur, alive = lbl, True
                print("  -> 실제 속도는 %s 로 보인다." % lbl)
                break
    if not alive:
        dll.CAN_Uninitialize(c_ushort(ch))
        print("\n어느 속도에서도 응답이 없다. 확인할 것:")
        print("  - 배선(CAN_H/CAN_L), 종단저항 60옴")
        print("  - MD400T 전원")
        print("  - Standard 모드도 시도: --std")
        return 1

    # ---------------- 변경 ---------------- #
    if not a.write:
        dll.CAN_Uninitialize(c_ushort(ch))
        print("\n읽기만 했다. 변경하려면 --write 500k 를 준다.")
        return 0

    tgt = a.write.lower()
    if tgt not in BITRATE_CODE:
        dll.CAN_Uninitialize(c_ushort(ch))
        sys.exit("설정 불가 속도: %s (가능: %s)"
                 % (a.write, ", ".join(BITRATE_CODE)))

    code = BITRATE_CODE[tgt]
    print("\n2) PID_ECAN_BITRATE(137) <- %d (%s) 쓰기" % (code, tgt))
    print("   데이터: %02X %02X %02X 00 00 00 00 00"
          % (PID_ECAN_BITRATE, WRITE_CHECK, code))
    send(dll, ch, a.id, [PID_ECAN_BITRATE, WRITE_CHECK, code], ext)
    time.sleep(0.3)
    dll.CAN_Uninitialize(c_ushort(ch))

    # ---------------- 새 속도에서 확인 ---------------- #
    print("\n3) 새 속도 %s 에서 응답 확인" % tgt)
    time.sleep(1.0)
    open_bus(dll, ch, tgt)
    ok = probe(dll, ch, a.id, ext, tgt)
    dll.CAN_Uninitialize(c_ushort(ch))

    if ok:
        print("\n변경 성공. MD400T 는 이제 %s 로 통신한다." % tgt)
        return 0
    print("\n새 속도에서 응답이 없다. 전원을 껐다 켠 뒤 다시 확인하라.")
    print("  python %s --from %s" % (sys.argv[0], tgt))
    return 1


if __name__ == "__main__":
    sys.exit(main())
