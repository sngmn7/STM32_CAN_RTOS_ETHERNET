#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zone_hmi.py — 존 아키텍처 통합 HMI (진단 + 제어)

기존 zone_ui.py(고장 표시)에 액추에이터 제어를 합쳤다.

  왼쪽   차량 존 구조도 — 어느 노드/링크가 고장인지 색으로
  가운데 제어판 — FRONT / REAR 액추에이터 오버라이드
  오른쪽 반영 상태 — 보드가 실제로 내리고 있는 값(텔레메트리 되읽기)
  아래   DTC 목록

수신
  5105  'SD'  DTC          (Front / Rear)
  5102  'SV'  Front 상태   28 B
  5002  'SR'  Rear 상태    48 B
송신
  5101        Front 명령   12 B  (ZONE_MSG_FRONT_COMMAND 0x20)
  5001        Rear  명령   12 B  (ZONE_MSG_REAR_COMMAND  0x21)

펌웨어의 JETSON_COMMAND_TIMEOUT_MS 가 500 ms 다. 명령을 한 번만 보내면
0.5 초 뒤 보드가 링크를 죽은 것으로 보고 오버라이드를 스스로 푼다.
그래서 이 프로그램은 현재 지령을 200 ms 마다 되풀이해 보낸다.
체크를 풀면 해당 override 비트가 0 이 되어 보드는 원래 제어원(Front
페달/스위치)으로 되돌아간다. 즉 '놓으면 원상복구' 가 기본 동작이다.

표준 라이브러리만 쓴다. 젯슨에 아무것도 설치하지 않는다.

    DISPLAY=:0 python3 zone_hmi.py
"""

import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import font as tkfont

# ------------------------------------------------------------------ #
# 프로토콜 상수 — 펌웨어 zone_udp_protocol.h / vstatus_proto.h 와 일치  #
# ------------------------------------------------------------------ #
FRONT_IP = "192.168.1.201"
REAR_IP = "192.168.1.200"

FRONT_CMD_PORT = 5101
REAR_CMD_PORT = 5001
SV_PORT = 5102          # Front 상태
SR_PORT = 5002          # Rear 상태
DTC_PORT = 5105

ZONE_MAGIC_0 = 0x5A
ZONE_MAGIC_JETSON = 0x4A
ZONE_VERSION = 1
ZONE_MSG_FRONT_COMMAND = 0x20
ZONE_MSG_REAR_COMMAND = 0x21

# Front 명령 플래그
F_STEER_OVR = 1 << 0
F_ACT_EN = 1 << 1
F_LAMP_OVR = 1 << 2
F_LAMP_ON = 1 << 3
F_TURN_OVR = 1 << 4

# Rear 명령 플래그
R_TURN_OVR = 1 << 0
R_BRAKE_OVR = 1 << 1
R_WINDOW_OVR = 1 << 2

TX_PERIOD = 0.2         # 초. 펌웨어 타임아웃 500 ms 의 절반 이하

# 텔레메트리
SV_MAGIC = 0x5653
SV_SIZE = 28
SV_FMT = "<HBBIIHHHHhHBBBB"
SR_MAGIC = 0x5253
SR_SIZE = 48
SR_FMT = "<HBBII" + "BBBBBBH" + "HhhhhBB" + "IIHHHH"

RST_SRC_FRONT_LINK = 1 << 0
RST_SRC_TURN_JETSON = 1 << 1
RST_SRC_BRAKE_JETSON = 1 << 2
RST_SRC_JETSON_LINK = 1 << 3
RST_HLT_CAN500_ERR = 1 << 0
RST_HLT_CAN250_ERR = 1 << 1

# DTC
DTC_MAGIC = 0x4453
DTC_PACKET_SIZE = 232
DTC_MAX_ENTRIES = 12
HDR_FMT = "<HBBIIBBBB"          # 16 B
ENTRY_FMT = "<HBBIIHH"          # 16 B
FREEZE_FMT = "<HBBI" + "8H"     # 24 B

STS_TEST_FAILED = 0x01
STS_PENDING = 0x02
STS_CONFIRMED = 0x04
STS_WARNING = 0x08

STALE_SEC = 3.0

# ------------------------------------------------------------------ #
# 색                                                                  #
# ------------------------------------------------------------------ #
BG = "#12161c"
PANEL = "#0d1117"
FG = "#e6edf3"
DIM = "#7d8590"
EDGE = "#30363d"
C_GRAY = "#3d444d"
C_GREEN = "#2ea043"
C_YELLOW = "#d29922"
C_RED = "#da3633"
C_HIST = "#e08c3b"
C_ACT = "#316dca"       # 오버라이드 활성

MAP = {
    ("F", 0xC300): "link_jet_front",
    ("R", 0xC300): "link_jet_rear",
    ("F", 0xC100): "link_zone",
    ("R", 0xC100): "link_zone",
    ("F", 0xC003): "link_f_can",
    ("F", 0x0510): "link_f_can",
    ("F", 0x0503): "node_f407",
    ("R", 0xC002): "link_r_can250",
    ("R", 0xC001): "link_r_can500",
    ("R", 0x0500): "node_f446",
    ("R", 0x0501): "node_f446",
    ("R", 0x0502): "node_f446",
}

NAME = {
    0xC001: "CAN500 링크",
    0xC002: "CAN250 링크",
    0xC003: "하위노드 버스오프",
    0xC100: "존간 링크",
    0xC300: "호스트 링크",
    0x0500: "초음파",
    0x0501: "IMU",
    0x0502: "토글스위치",
    0x0503: "DHT11 온습도",
    0x0510: "하위노드 신호",
}


def dtc_str(code):
    return "%c%X%03X" % ("PCBU"[(code >> 14) & 3], (code >> 12) & 3, code & 0xFFF)


def sts_str(s):
    if s == 0:
        return "정상"
    parts = []
    if s & STS_TEST_FAILED:
        parts.append("고장")
    if s & STS_PENDING:
        parts.append("보류")
    if s & STS_CONFIRMED:
        parts.append("확정")
    if s & STS_WARNING:
        parts.append("경고")
    return "|".join(parts)


def xor_checksum(d):
    c = 0
    for i in range(11):
        c ^= d[i]
    return c


# ================================================================== #
# 공유 상태                                                           #
# ================================================================== #
class State(object):
    def __init__(self):
        self.lock = threading.Lock()
        # 진단
        self.zones = {}
        self.freeze = {}
        self.packets = 0
        self.bad = 0
        # 텔레메트리 되읽기
        self.sv = None
        self.sv_t = 0.0
        self.sr = None
        self.sr_t = 0.0
        # 명령 (UI 스레드가 쓰고 송신 스레드가 읽는다)
        self.f_lamp_ovr = 0
        self.f_lamp_on = 0
        self.f_turn_ovr = 0
        self.f_turn_mode = 0
        self.f_act_en = 0
        self.f_act_target = 0
        self.f_steer_ovr = 0
        self.r_turn_ovr = 0
        self.r_turn_mode = 0
        self.r_brake_ovr = 0
        self.r_brake = 0
        self.r_win_ovr = 0
        self.r_window = 0
        # 송신 통계
        self.tx_count = 0
        self.f_ack = None       # 마지막 ACK RTT (ms)
        self.r_ack = None
        self.f_ack_t = 0.0
        self.r_ack_t = 0.0


# ================================================================== #
# 수신 스레드                                                         #
# ================================================================== #
def bind_udp(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    return s


def rx_dtc(st):
    s = bind_udp(DTC_PORT)
    hs = struct.calcsize(HDR_FMT)
    es = struct.calcsize(ENTRY_FMT)
    while True:
        try:
            data, _ = s.recvfrom(2048)
        except OSError:
            continue
        if len(data) != DTC_PACKET_SIZE:
            with st.lock:
                st.bad += 1
            continue
        magic, ver, zone, seq, uptime, cnt, active, conf, _p = \
            struct.unpack(HDR_FMT, data[:hs])
        if magic != DTC_MAGIC:
            with st.lock:
                st.bad += 1
            continue
        z = chr(zone)
        entries = []
        for i in range(min(cnt, DTC_MAX_ENTRIES)):
            off = hs + i * es
            code, status, occ, first, last, det, _r = \
                struct.unpack(ENTRY_FMT, data[off:off + es])
            if code != 0:
                entries.append((code, status, occ, det))
        fz = struct.unpack(FREEZE_FMT,
                           data[hs + DTC_MAX_ENTRIES * es:
                                hs + DTC_MAX_ENTRIES * es + 24])
        with st.lock:
            st.packets += 1
            st.zones[z] = {"t": time.time(), "uptime": uptime,
                           "dtc": entries, "active": active, "conf": conf}
            st.freeze[z] = (fz[0], list(fz[4:]))


def rx_sv(st):
    """Front 상태 28 B — 지령이 실제로 F407 까지 갔는지 확인용."""
    s = bind_udp(SV_PORT)
    while True:
        try:
            data, _ = s.recvfrom(512)
        except OSError:
            continue
        if len(data) != SV_SIZE:
            continue
        v = struct.unpack(SV_FMT, data)
        if v[0] != SV_MAGIC:
            continue
        with st.lock:
            st.sv = v
            st.sv_t = time.time()


def rx_sr(st):
    """Rear 상태 48 B."""
    s = bind_udp(SR_PORT)
    while True:
        try:
            data, _ = s.recvfrom(512)
        except OSError:
            continue
        if len(data) != SR_SIZE:
            continue
        v = struct.unpack(SR_FMT, data)
        if v[0] != SR_MAGIC:
            continue
        with st.lock:
            st.sr = v
            st.sr_t = time.time()


# ================================================================== #
# 송신 스레드 — 현재 지령을 200 ms 마다 되풀이한다                     #
# ================================================================== #
def build_front(seq, st):
    d = bytearray(12)
    d[0] = ZONE_MAGIC_0
    d[1] = ZONE_MAGIC_JETSON
    d[2] = ZONE_VERSION
    d[3] = ZONE_MSG_FRONT_COMMAND
    d[4] = seq & 0xFF
    d[5] = (seq >> 8) & 0xFF
    flags = 0
    if st.f_steer_ovr:
        flags |= F_STEER_OVR
    if st.f_act_en:
        flags |= F_ACT_EN
    if st.f_lamp_ovr:
        flags |= F_LAMP_OVR
        if st.f_lamp_on:
            flags |= F_LAMP_ON
    if st.f_turn_ovr:
        flags |= F_TURN_OVR
    d[6] = flags
    t = st.f_act_target & 0x0FFF          # 펌웨어가 4095 초과를 버린다
    d[7] = t & 0xFF
    d[8] = (t >> 8) & 0xFF
    d[9] = st.f_turn_mode & 0x03          # 0..2
    d[11] = xor_checksum(d)
    return bytes(d)


def build_rear(seq, st):
    d = bytearray(12)
    d[0] = ZONE_MAGIC_0
    d[1] = ZONE_MAGIC_JETSON
    d[2] = ZONE_VERSION
    d[3] = ZONE_MSG_REAR_COMMAND
    d[4] = seq & 0xFF
    d[5] = (seq >> 8) & 0xFF
    flags = 0
    if st.r_turn_ovr:
        flags |= R_TURN_OVR
    if st.r_brake_ovr:
        flags |= R_BRAKE_OVR
    if st.r_win_ovr:
        flags |= R_WINDOW_OVR
    d[6] = flags
    d[7] = st.r_turn_mode & 0x03          # 펌웨어 검증: <= 2
    d[8] = st.r_brake & 0x01
    d[9] = st.r_window & 0x01
    d[11] = xor_checksum(d)
    return bytes(d)


def tx_thread(st, stop):
    """양쪽 보드에 현재 지령을 주기 송신하고 ACK 왕복시간을 잰다.

    ACK 는 보드가 명령을 보낸 소켓 주소로 되돌려준다. 따라서 같은
    소켓에서 recv 하면 왕복이 잡힌다 — 링크 생존의 직접 증거다.
    """
    fs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fs.settimeout(0.03)
    rs.settimeout(0.03)
    seq = 0
    while not stop.is_set():
        with st.lock:
            fp = build_front(seq, st)
            rp = build_rear(seq, st)
        t0 = time.time()
        try:
            fs.sendto(fp, (FRONT_IP, FRONT_CMD_PORT))
        except OSError:
            pass
        try:
            rs.sendto(rp, (REAR_IP, REAR_CMD_PORT))
        except OSError:
            pass

        for sock, which in ((fs, "f"), (rs, "r")):
            try:
                sock.recv(64)
                rtt = (time.time() - t0) * 1000.0
                with st.lock:
                    if which == "f":
                        st.f_ack, st.f_ack_t = rtt, time.time()
                    else:
                        st.r_ack, st.r_ack_t = rtt, time.time()
            except socket.timeout:
                pass
            except OSError:
                pass

        with st.lock:
            st.tx_count += 1
        seq = (seq + 1) & 0xFFFF
        time.sleep(TX_PERIOD)

    # 종료 시 오버라이드 해제 — 보드를 원래 제어원으로 돌려놓는다
    with st.lock:
        st.f_lamp_ovr = st.f_turn_ovr = st.f_act_en = st.f_steer_ovr = 0
        st.r_turn_ovr = st.r_brake_ovr = st.r_win_ovr = 0
    for _ in range(5):
        with st.lock:
            fp, rp = build_front(seq, st), build_rear(seq, st)
        try:
            fs.sendto(fp, (FRONT_IP, FRONT_CMD_PORT))
            rs.sendto(rp, (REAR_IP, REAR_CMD_PORT))
        except OSError:
            pass
        seq = (seq + 1) & 0xFFFF
        time.sleep(0.05)
    fs.close()
    rs.close()


# ================================================================== #
# UI                                                                  #
# ================================================================== #
class UI(object):
    def __init__(self, root, st, stop):
        self.st = st
        self.root = root
        self.stop = stop
        root.title("Zone Controller — 진단 · 제어 통합 HMI")
        root.configure(bg=BG)

        self.f_big = tkfont.Font(family="DejaVu Sans", size=13, weight="bold")
        self.f_mid = tkfont.Font(family="DejaVu Sans", size=10)
        self.f_sml = tkfont.Font(family="DejaVu Sans Mono", size=9)
        self.f_hdr = tkfont.Font(family="DejaVu Sans", size=11, weight="bold")

        upper = tk.Frame(root, bg=BG)
        upper.pack(fill="both", expand=True, padx=10, pady=(8, 4))

        self.cv = tk.Canvas(upper, width=760, height=430, bg=BG,
                            highlightthickness=0)
        self.cv.pack(side="left")

        right = tk.Frame(upper, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(12, 0))
        tk.Label(right, text="고장 코드 (DTC)", bg=BG, fg=DIM,
                 font=self.f_hdr, anchor="w").pack(fill="x")
        self.txt = tk.Text(right, width=48, bg=PANEL, fg=FG,
                           font=self.f_sml, relief="flat",
                           highlightthickness=1, highlightbackground=EDGE)
        self.txt.pack(fill="both", expand=True, pady=(4, 0))

        lower = tk.Frame(root, bg=BG)
        lower.pack(fill="x", padx=10, pady=(4, 4))
        self.build_front_panel(lower)
        self.build_rear_panel(lower)
        self.build_feedback_panel(lower)

        bar = tk.Frame(root, bg=BG)
        bar.pack(fill="x", padx=12, pady=(0, 8))
        tk.Button(bar, text="전체 해제 (원상복구)", command=self.release_all,
                  bg="#21262d", fg=FG, activebackground=C_RED,
                  activeforeground="#ffffff", relief="flat",
                  font=self.f_mid, padx=14, pady=4).pack(side="left")
        self.status = tk.Label(bar, text="", bg=BG, fg=DIM,
                               font=self.f_sml, anchor="w")
        self.status.pack(side="left", fill="x", expand=True, padx=(14, 0))

        self.items = {}
        self.build_diagram()
        self.tick()

    # -------------------- 제어판 -------------------- #
    def group(self, parent, title, color):
        f = tk.LabelFrame(parent, text=" " + title + " ", bg=PANEL, fg=color,
                          font=self.f_hdr, relief="flat", bd=2,
                          highlightthickness=1, highlightbackground=EDGE,
                          padx=10, pady=6)
        f.pack(side="left", fill="both", expand=True, padx=(0, 8))
        return f

    def chk(self, parent, text, var, cmd):
        return tk.Checkbutton(parent, text=text, variable=var, command=cmd,
                              bg=PANEL, fg=FG, selectcolor="#21262d",
                              activebackground=PANEL, activeforeground=C_ACT,
                              font=self.f_mid, anchor="w",
                              highlightthickness=0)

    def radios(self, parent, var, opts, cmd):
        row = tk.Frame(parent, bg=PANEL)
        for label, val in opts:
            tk.Radiobutton(row, text=label, variable=var, value=val,
                           command=cmd, bg=PANEL, fg=FG,
                           selectcolor="#21262d", activebackground=PANEL,
                           activeforeground=C_ACT, font=self.f_mid,
                           highlightthickness=0).pack(side="left")
        return row

    def build_front_panel(self, parent):
        g = self.group(parent, "FRONT H723 → F407 제어", "#58a6ff")

        self.v_f_lamp_ovr = tk.IntVar()
        self.v_f_lamp_on = tk.IntVar()
        self.v_f_turn_ovr = tk.IntVar()
        self.v_f_turn = tk.IntVar()
        self.v_f_act_en = tk.IntVar()
        self.v_f_act = tk.IntVar()
        self.v_f_steer = tk.IntVar()

        self.chk(g, "전조등 오버라이드", self.v_f_lamp_ovr,
                 self.push).pack(fill="x")
        self.radios(g, self.v_f_lamp_on,
                    [("끄기", 0), ("켜기", 1)], self.push).pack(fill="x",
                                                             padx=(20, 0))

        self.chk(g, "방향지시등 오버라이드", self.v_f_turn_ovr,
                 self.push).pack(fill="x", pady=(6, 0))
        self.radios(g, self.v_f_turn,
                    [("끄기", 0), ("좌", 1), ("우", 2)],
                    self.push).pack(fill="x", padx=(20, 0))

        self.chk(g, "액추에이터 ENABLE", self.v_f_act_en,
                 self.push).pack(fill="x", pady=(6, 0))
        sc = tk.Scale(g, from_=0, to=4095, orient="horizontal",
                      variable=self.v_f_act, command=lambda _e: self.push(),
                      bg=PANEL, fg=FG, troughcolor="#21262d",
                      highlightthickness=0, font=self.f_sml,
                      label="목표 위치 (0~4095)", length=210)
        sc.pack(fill="x", padx=(20, 0))

        self.chk(g, "조향 오버라이드", self.v_f_steer,
                 self.push).pack(fill="x", pady=(6, 0))

    def build_rear_panel(self, parent):
        g = self.group(parent, "REAR H723 → RearBody F407 제어", "#a371f7")

        self.v_r_turn_ovr = tk.IntVar()
        self.v_r_turn = tk.IntVar()
        self.v_r_brk_ovr = tk.IntVar()
        self.v_r_brk = tk.IntVar()
        self.v_r_win_ovr = tk.IntVar()
        self.v_r_win = tk.IntVar()

        self.chk(g, "방향지시등 오버라이드", self.v_r_turn_ovr,
                 self.push).pack(fill="x")
        self.radios(g, self.v_r_turn,
                    [("끄기", 0), ("좌", 1), ("우", 2)],
                    self.push).pack(fill="x", padx=(20, 0))

        self.chk(g, "브레이크등 오버라이드", self.v_r_brk_ovr,
                 self.push).pack(fill="x", pady=(6, 0))
        self.radios(g, self.v_r_brk,
                    [("끄기", 0), ("켜기", 1)], self.push).pack(fill="x",
                                                             padx=(20, 0))

        self.chk(g, "윈도우 오버라이드", self.v_r_win_ovr,
                 self.push).pack(fill="x", pady=(6, 0))
        self.radios(g, self.v_r_win,
                    [("정지", 0), ("동작", 1)], self.push).pack(fill="x",
                                                             padx=(20, 0))

        tk.Label(g, text="F446 은 센서 전용 — 내릴 지령 없음",
                 bg=PANEL, fg=DIM, font=self.f_sml,
                 anchor="w").pack(fill="x", pady=(10, 0))

    def build_feedback_panel(self, parent):
        g = self.group(parent, "반영 상태 (보드 되읽기)", C_GREEN)
        self.fb = tk.Text(g, width=36, height=15, bg=PANEL, fg=FG,
                          font=self.f_sml, relief="flat",
                          highlightthickness=0)
        self.fb.pack(fill="both", expand=True)

    # -------------------- 명령 반영 -------------------- #
    def push(self):
        """UI 위젯 -> 공유 상태. 송신 스레드가 200 ms 마다 집어간다."""
        st = self.st
        with st.lock:
            st.f_lamp_ovr = self.v_f_lamp_ovr.get()
            st.f_lamp_on = self.v_f_lamp_on.get()
            st.f_turn_ovr = self.v_f_turn_ovr.get()
            st.f_turn_mode = self.v_f_turn.get()
            st.f_act_en = self.v_f_act_en.get()
            st.f_act_target = self.v_f_act.get()
            st.f_steer_ovr = self.v_f_steer.get()
            st.r_turn_ovr = self.v_r_turn_ovr.get()
            st.r_turn_mode = self.v_r_turn.get()
            st.r_brake_ovr = self.v_r_brk_ovr.get()
            st.r_brake = self.v_r_brk.get()
            st.r_win_ovr = self.v_r_win_ovr.get()
            st.r_window = self.v_r_win.get()

    def release_all(self):
        """모든 오버라이드 해제. 보드는 Front 페달/스위치 제어로 돌아간다."""
        for v in (self.v_f_lamp_ovr, self.v_f_turn_ovr, self.v_f_act_en,
                  self.v_f_steer, self.v_r_turn_ovr, self.v_r_brk_ovr,
                  self.v_r_win_ovr):
            v.set(0)
        self.push()

    # -------------------- 구조도 -------------------- #
    def node(self, key, x, y, w, h, label, sub):
        r = self.cv.create_rectangle(x, y, x + w, y + h, fill=C_GRAY,
                                     outline="", width=3)
        self.cv.create_text(x + w / 2, y + h / 2 - 9, text=label,
                            fill="#ffffff", font=self.f_big)
        self.cv.create_text(x + w / 2, y + h / 2 + 11, text=sub,
                            fill="#c9d1d9", font=self.f_sml)
        self.items[key] = ("rect", r)

    def link(self, key, x1, y1, x2, y2, label):
        ln = self.cv.create_line(x1, y1, x2, y2, fill=C_GRAY, width=6)
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        self.cv.create_text(mx, my - 12, text=label, fill=DIM,
                            font=self.f_sml)
        self.items[key] = ("line", ln)

    def build_diagram(self):
        c = self.cv
        c.create_text(380, 16, text="차량 존 아키텍처", fill=DIM,
                      font=self.f_mid)

        self.node("node_jetson", 300, 32, 160, 54, "Jetson", "게이트웨이")

        self.link("link_jet_front", 330, 86, 150, 150, "UDP 5101/5102")
        self.link("link_jet_rear", 430, 86, 610, 150, "UDP 5001/5002")

        self.node("node_front", 60, 150, 180, 60, "FRONT H723", "192.168.1.201")
        self.node("node_rear", 520, 150, 180, 60, "REAR H723", "192.168.1.200")

        self.link("link_zone", 240, 180, 520, 180, "존간 UDP 5003/5103")

        self.link("link_f_can", 150, 210, 150, 300, "CAN 250k")
        self.node("node_f407", 60, 300, 180, 60, "F407 Front", "조향·브레이크·램프")

        self.link("link_r_can250", 580, 210, 500, 300, "CAN 250k")
        self.link("link_r_can500", 640, 210, 700, 300, "CAN 500k")
        self.node("node_f446", 410, 300, 180, 60, "F446", "초음파·IMU·스위치")
        self.node("node_rearbody", 610, 300, 150, 60, "RearBody", "F407")

        y = 392
        for i, (col, lab) in enumerate([(C_GREEN, "정상"), (C_YELLOW, "이상"),
                                        (C_RED, "경고"), (C_GRAY, "무응답"),
                                        (C_HIST, "이력")]):
            x = 60 + i * 140
            c.create_rectangle(x, y, x + 16, y + 12, fill=col, outline="")
            c.create_text(x + 24, y + 6, text=lab, fill=DIM, anchor="w",
                          font=self.f_sml)

    def paint(self, key, color, hist=False):
        kind, obj = self.items[key]
        if kind == "rect":
            self.cv.itemconfig(obj, fill=color,
                               outline=(C_HIST if hist else ""), width=3)
        else:
            self.cv.itemconfig(obj, fill=color)

    # -------------------- 주기 갱신 -------------------- #
    def tick(self):
        now = time.time()
        with self.st.lock:
            zones = dict(self.st.zones)
            freeze = dict(self.st.freeze)
            packets, bad = self.st.packets, self.st.bad
            sv, sv_t = self.st.sv, self.st.sv_t
            sr, sr_t = self.st.sr, self.st.sr_t
            txc = self.st.tx_count
            f_ack, f_ack_t = self.st.f_ack, self.st.f_ack_t
            r_ack, r_ack_t = self.st.r_ack, self.st.r_ack_t

        live = {z: v for z, v in zones.items() if now - v["t"] <= STALE_SEC}

        base = C_GREEN if live else C_GRAY
        for k in self.items:
            self.paint(k, base)
        self.paint("node_jetson", C_GREEN)

        if "F" not in live:
            for k in ("node_front", "link_jet_front", "link_f_can",
                      "node_f407"):
                self.paint(k, C_GRAY)
        if "R" not in live:
            for k in ("node_rear", "link_jet_rear", "link_r_can250",
                      "link_r_can500", "node_f446", "node_rearbody"):
                self.paint(k, C_GRAY)
        if not live:
            self.paint("link_zone", C_GRAY)

        worst = {}
        for z, v in live.items():
            for code, status, occ, det in v["dtc"]:
                key = MAP.get((z, code))
                if key is None:
                    continue
                if status & STS_TEST_FAILED:
                    lvl = 3 if (status & STS_WARNING) else 2
                elif status & STS_PENDING:
                    lvl = 2
                elif status & STS_CONFIRMED:
                    lvl = 1
                else:
                    lvl = 0
                if lvl > worst.get(key, 0):
                    worst[key] = lvl
        for key, lvl in worst.items():
            if lvl == 3:
                self.paint(key, C_RED)
            elif lvl == 2:
                self.paint(key, C_YELLOW)
            elif lvl == 1:
                self.paint(key, C_GREEN, hist=True)

        self.update_dtc_text(live, freeze)
        self.update_feedback(now, sv, sv_t, sr, sr_t,
                             f_ack, f_ack_t, r_ack, r_ack_t)

        self.status.config(
            text="DTC 수신 %d (불량 %d)   |   명령 송신 %d 회 @ %.0f Hz   |   %s"
                 % (packets, bad, txc, 1.0 / TX_PERIOD,
                    time.strftime("%H:%M:%S")))
        self.root.after(300, self.tick)

    def update_dtc_text(self, live, freeze):
        self.txt.config(state="normal")
        self.txt.delete("1.0", "end")
        for z in ("F", "R"):
            title = "FRONT" if z == "F" else "REAR"
            if z not in live:
                self.txt.insert("end", "[%s]  무응답\n\n" % title)
                continue
            v = live[z]
            self.txt.insert("end", "[%s]  가동 %.1f s\n"
                            % (title, v["uptime"] / 1000.0))
            for code, status, occ, det in v["dtc"]:
                mark = "  " if status == 0 else \
                       ("!!" if status & STS_WARNING else " *")
                self.txt.insert("end", "%s %-6s %-14s %-14s occ=%-3d %sms\n" %
                                (mark, dtc_str(code), NAME.get(code, ""),
                                 sts_str(status), occ, det))
            fz = freeze.get(z)
            if fz and fz[0]:
                self.txt.insert("end", "   프리즈(%s): %s\n" %
                                (dtc_str(fz[0]),
                                 " ".join(str(x) for x in fz[1])))
            self.txt.insert("end", "\n")
        self.txt.config(state="disabled")

    def update_feedback(self, now, sv, sv_t, sr, sr_t,
                        f_ack, f_ack_t, r_ack, r_ack_t):
        """명령이 실제로 말단까지 갔는지 되읽기로 보여준다.

        여기 나오는 값은 UI 가 보낸 지령이 아니라 H723 이 CAN 으로
        받아온 F407 / RearBody 의 실제 출력이다. 둘이 다르면 하행
        경로 어딘가가 끊긴 것이다.
        """
        t = self.fb
        t.config(state="normal")
        t.delete("1.0", "end")

        def onoff(x):
            return "ON " if x else "off"

        t.insert("end", "── FRONT (SV 5102) ──\n")
        if sv is None or now - sv_t > STALE_SEC:
            t.insert("end", "  수신 없음\n")
        else:
            (_m, _v, flags, seq, up, steer, accel, brake, actpos,
             temp, hum, tl, tr, lamp, _r) = sv
            t.insert("end", "  전조등    %s   (지령 %s)\n"
                     % (onoff(lamp), onoff(self.v_f_lamp_ovr.get()
                                           and self.v_f_lamp_on.get())))
            t.insert("end", "  방향지시  좌%s 우%s\n" % (onoff(tl), onoff(tr)))
            t.insert("end", "  액추에이터 위치 %4d / 목표 %4d\n"
                     % (actpos, self.v_f_act.get()))
            t.insert("end", "  조향 %4d  가속 %4d  제동 %4d\n"
                     % (steer, accel, brake))
            t.insert("end", "  온도 %.1f C  습도 %.1f %%%s\n"
                     % (temp / 10.0, hum / 10.0,
                        "" if flags & 1 else " (무효)"))
            if flags & 2:
                t.insert("end", "  !! F407 CAN 버스오프\n")

        t.insert("end", "\n── REAR (SR 5002) ──\n")
        if sr is None or now - sr_t > STALE_SEC:
            t.insert("end", "  수신 없음\n")
        else:
            (_m, _v, src, seq, up, turn, ll, lr, lb, win, health,
             fbadc, ultra, qi, qj, qk, qr, sen, _rsv,
             c5, c2, cmd, imu, ul, sw) = sr
            t.insert("end", "  방향지시  %s (좌%s 우%s)\n"
                     % (("끄기", "좌", "우")[turn if turn < 3 else 0],
                        onoff(ll), onoff(lr)))
            t.insert("end", "  브레이크등 %s   윈도우 %s\n"
                     % (onoff(lb), onoff(win)))
            t.insert("end", "  지령 출처  방향%s 제동%s\n"
                     % ("젯슨" if src & RST_SRC_TURN_JETSON else "Front",
                        "젯슨" if src & RST_SRC_BRAKE_JETSON else "Front"))
            t.insert("end", "  링크  Front%s  젯슨%s\n"
                     % ("O" if src & RST_SRC_FRONT_LINK else "X",
                        "O" if src & RST_SRC_JETSON_LINK else "X"))
            t.insert("end", "  후방거리 %d mm\n" % ultra)
            if health:
                t.insert("end", "  !! CAN%s 버스오프\n"
                         % ("500" if health & RST_HLT_CAN500_ERR else "250"))

        t.insert("end", "\n── 명령 ACK 왕복 ──\n")
        fa = "%.2f ms" % f_ack if (f_ack is not None and
                                   now - f_ack_t < 2.0) else "무응답"
        ra = "%.2f ms" % r_ack if (r_ack is not None and
                                   now - r_ack_t < 2.0) else "무응답"
        t.insert("end", "  FRONT %s\n  REAR  %s\n" % (fa, ra))
        t.config(state="disabled")


def main():
    st = State()
    stop = threading.Event()
    for fn in (rx_dtc, rx_sv, rx_sr):
        threading.Thread(target=fn, args=(st,), daemon=True).start()
    tx = threading.Thread(target=tx_thread, args=(st, stop))
    tx.daemon = False
    tx.start()

    root = tk.Tk()
    ui = UI(root, st, stop)

    def on_close():
        stop.set()
        tx.join(timeout=2.0)      # 오버라이드 해제 패킷이 나갈 시간
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    finally:
        stop.set()


if __name__ == "__main__":
    main()
