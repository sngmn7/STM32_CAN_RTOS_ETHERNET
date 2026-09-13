#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""md400_rs485.py — RS485 로 MD400T 의 CAN 통신속도를 읽고 바꾼다.

PCAN 이 필요 없다. USB-RS485 컨버터 하나면 된다.

근거: MDROBOT RS485/232 통신사양 V6.4h, CAN통신사양 V2.1

  패킷      RMID, TMID, ID, PID, DataNumber, DATA.., CHK
  보낼 때   RMID=183(제어기), TMID=184(사용자)
  받을 때   RMID=184, TMID=183  (뒤바뀐다)
  체크섬    byChkSend = 앞의 모든 바이트 합
            byCHK     = (~byChkSend) + 1      (2의 보수)
            수신 검증 = CHK 포함 전부 더해서 0 이면 정상

  PID 4   (0x04) R   PID_REQ_PID_DATA   원하는 PID 의 값을 요청
  PID 137 (0x89) R/W PID_ECAN_BITRATE   CAN 통신속도
        1:50k  2:100k  3:250k  4:500k  8:125k
        쓰기 → 183, TMID, ID, 137, 2, 0xaa, BIT_RATE, CHK
        0xaa 는 Write check byte(쓰기 보안용)

기본 동작은 읽기만 한다. 바꾸려면 --write 를 준다.

    python md400_rs485.py --port COM7                 # ID 찾고 현재값 읽기
    python md400_rs485.py --port COM7 --write 500k    # 500k 로 변경
"""

import argparse
import sys
import time

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    sys.exit("pyserial 이 필요하다:  pip install pyserial")

RMID_CTRL = 183      # 모터제어기
TMID_USER = 184      # 사용자(PC)
ID_ALL = 0xFE

PID_VER = 1
PID_REQ_PID_DATA = 4
PID_ECAN_BITRATE = 137
WRITE_CHECK = 0xAA

BITRATE_CODE = {"50k": 1, "100k": 2, "250k": 3, "500k": 4, "125k": 8}
CODE_NAME = {1: "50k", 2: "100k", 3: "250k", 4: "500k", 8: "125k"}


def build(node_id, pid, data):
    """RMID, TMID, ID, PID, DataNumber, DATA.., CHK"""
    body = [RMID_CTRL, TMID_USER, node_id, pid, len(data)] + list(data)
    chk = ((~(sum(body) & 0xFF)) + 1) & 0xFF
    return bytes(body + [chk])


def parse(buf):
    """받은 스트림에서 온전한 응답 패킷들을 뽑는다."""
    out = []
    i = 0
    while i + 6 <= len(buf):
        # 응답은 RMID=184, TMID=183 으로 뒤바뀌어 온다
        if buf[i] != TMID_USER or buf[i + 1] != RMID_CTRL:
            i += 1
            continue
        n = buf[i + 4]
        total = 5 + n + 1
        if i + total > len(buf):
            break
        pkt = buf[i:i + total]
        if (sum(pkt) & 0xFF) == 0:          # CHK 포함 합이 0 이면 정상
            out.append({"id": pkt[2], "pid": pkt[3],
                        "data": list(pkt[5:5 + n])})
            i += total
        else:
            i += 1
    return out


def xfer(ser, node_id, pid, data, wait=0.25):
    ser.reset_input_buffer()
    ser.write(build(node_id, pid, data))
    ser.flush()
    time.sleep(wait)
    return parse(ser.read(ser.in_waiting or 1))


def read_pid(ser, node_id, target_pid):
    """PID_REQ_PID_DATA 로 특정 PID 값을 요청한다."""
    for r in xfer(ser, node_id, PID_REQ_PID_DATA, [target_pid]):
        if r["pid"] == target_pid:
            return r["data"]
    return None


def find_id(ser, hint=None):
    """응답하는 제어기 ID 를 찾는다."""
    order = []
    if hint is not None:
        order.append(hint)
    order += [1, 2, 3, 0] + [i for i in range(4, 254)]
    seen = set()
    for i in order:
        if i in seen:
            continue
        seen.add(i)
        if read_pid(ser, i, PID_VER) is not None:
            return i
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="COM 포트 (생략하면 목록만 보여준다)")
    ap.add_argument("--baud", type=int, default=19200,
                    help="RS485 통신속도. 공장 기본 19200")
    ap.add_argument("--id", type=lambda x: int(x, 0),
                    help="제어기 ID. 생략하면 찾는다")
    ap.add_argument("--write", metavar="속도",
                    help="CAN 속도 변경 (50k/100k/125k/250k/500k)")
    a = ap.parse_args()

    ports = list(list_ports.comports())
    if not a.port:
        print("COM 포트 목록:")
        for p in ports:
            print("  %-8s %s" % (p.device, p.description))
        print("\nUSB-RS485 컨버터의 포트를 --port 로 지정하라.")
        return 0

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.3)
    except serial.SerialException as e:
        sys.exit("포트 열기 실패: %s" % e)

    with ser:
        print("%s @ %d bps" % (a.port, a.baud))

        node = a.id
        if node is None:
            print("\n1) 제어기 ID 탐색")
            node = find_id(ser)
            if node is None:
                print("  응답 없음. 확인할 것:")
                print("   - COM 포트가 맞는가 (--port)")
                print("   - RS485 A/B(485+/485-) 결선")
                print("   - RS485 속도 (--baud 9600/19200/38400/57600/115200)")
                print("   - MD400T 전원")
                return 1
            print("  ID %d 응답" % node)
        else:
            if read_pid(ser, node, PID_VER) is None:
                print("  ID %d 가 응답하지 않는다." % node)
                return 1
            print("  ID %d 응답" % node)

        ver = read_pid(ser, node, PID_VER)
        if ver:
            print("  PID_VER = %s" % " ".join("%d" % b for b in ver))

        print("\n2) 현재 CAN 속도 읽기 (PID 137)")
        cur = read_pid(ser, node, PID_ECAN_BITRATE)
        if cur is None:
            print("  읽기 실패. 이 모델이 PID 137 을 지원하는지 확인 필요.")
        else:
            code = cur[0]
            print("  BIT_RATE = %d  (%s)" % (code, CODE_NAME.get(code, "?")))

        if not a.write:
            print("\n읽기만 했다. 바꾸려면  --write 500k")
            return 0

        tgt = a.write.lower()
        if tgt not in BITRATE_CODE:
            sys.exit("설정 불가: %s (가능: %s)"
                     % (a.write, ", ".join(BITRATE_CODE)))
        code = BITRATE_CODE[tgt]

        print("\n3) CAN 속도 -> %s (BIT_RATE=%d) 쓰기" % (tgt, code))
        print("   패킷: 183 %d %d 137 2 0xAA %d CHK"
              % (TMID_USER, node, code))
        xfer(ser, node, PID_ECAN_BITRATE, [WRITE_CHECK, code], wait=0.4)

        time.sleep(0.5)
        print("\n4) 되읽기 확인")
        back = read_pid(ser, node, PID_ECAN_BITRATE)
        if back is None:
            print("  되읽기 실패. 전원을 껐다 켠 뒤 다시 확인하라.")
            return 1
        got = back[0]
        print("  BIT_RATE = %d  (%s)" % (got, CODE_NAME.get(got, "?")))
        if got == code:
            print("\n변경 성공. MD400T 의 CAN 은 이제 %s 다." % tgt)
            print("전원을 껐다 켜서 확실히 적용한 뒤 버스에 연결하라.")
            return 0
        print("\n값이 반영되지 않았다. 제조사에 문의가 필요하다.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
