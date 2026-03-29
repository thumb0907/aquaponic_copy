#!/usr/bin/env python3
"""
monitor_node.py  ─  터미널 대시보드
=========================================================
실행 모드:
  python monitor_node.py           # 전체 상태 요약 (기본)
  python monitor_node.py stm       # STM1 로그 화면
  python monitor_node.py stm2      # STM2 시퀀스 화면
"""

from __future__ import annotations

import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class C:
    RESET  = '\033[0m'
    RED    = '\033[91m'
    GREEN  = '\033[92m'
    YELLOW = '\033[93m'
    BLUE   = '\033[94m'
    CYAN   = '\033[96m'
    BOLD   = '\033[1m'
    GRAY   = '\033[90m'


class MonitorNode(Node):

    def __init__(self):
        super().__init__('monitor_node')
        self.mode = sys.argv[1] if len(sys.argv) > 1 else 'flags'

        # ── 상태 ──────────────────────────────────────
        self.stm_state  = 'idle'
        self.stm2_state = 'idle'
        self.stm1_log   = []
        self.stm2_log   = []

        self.flags = {
            'start_flag': False,
            'stm_state': 'idle',
            'stm2_state': 'idle',
            'pi1_alive': False,
            'pi2_alive': False,
            'emergency': False,
            'cam1_connected': False,
            'cam1_status': 'disconnected',
            'cam1_detect_label': 'none',
            'cam1_last_conf': '0.00',
        }

        self.pi1_last_hb = 0.0
        self.pi2_last_hb = 0.0

        # ── 구독 ──────────────────────────────────────
        self.create_subscription(String, '/pi1/uart_response', self._on_uart1, 10)
        self.create_subscription(String, '/pi2/uart_response', self._on_uart2, 10)
        self.create_subscription(String, '/monitor/state',     self._on_state, 10)
        self.create_subscription(String, '/system/heartbeat',  self._on_hb,    10)

        self.create_timer(0.5, self._refresh)

    # ── 수신 콜백 ─────────────────────────────────────
    def _on_uart1(self, msg: String):
        line = msg.data.strip()
        ts   = time.strftime('%H:%M:%S')

        color_map = {
            'STM1:PC:STATE:HOMING':             C.YELLOW,
            'STM1:PC:STATE:RUN_CONVEYOR1':      C.GREEN,
            'STM1:PC:STATE:SEEDING':            C.CYAN,
            'STM1:PC:STATE:EJECTING':           C.BLUE,
            'STM1:PC:STATE:WAIT_SCARA_PICK':    C.YELLOW,
            'STM1:PC:STATE:SCARA_PICK_STARTED': C.YELLOW,
            'STM1:PC:DONE:CYCLE1':              C.GREEN,
            'STM1:PC:STATE:ESTOP':              C.RED,
        }
        color = C.RED if 'ERR' in line else color_map.get(line, C.GRAY)
        self.stm1_log.append(f'{C.GRAY}[{ts}]{C.RESET} {color}{line}{C.RESET}')
        if len(self.stm1_log) > 15:
            self.stm1_log.pop(0)

    def _on_uart2(self, msg: String):
        line = msg.data.strip()
        ts   = time.strftime('%H:%M:%S')

        color_map = {
            'STM2:PC:STATE:CONVEY_RUN':   C.GREEN,
            'STM2:PC:STATE:IR_DETECTED':  C.CYAN,
            'STM2:PC:STATE:Z_FIX':        C.YELLOW,
            'STM2:PC:STATE:HARVESTING':   C.BLUE,
            'STM2:PC:STATE:HARVEST_DONE': C.GREEN,
            'STM2:PC:STATE:EJECTING':     C.BLUE,
            'STM2:PC:STATE:EJECT_DONE':   C.GREEN,
            'STM2:PC:DONE:CYCLE':         C.GREEN,
            'STM2:PC:STATE:RESET_DONE':   C.GREEN,
            'STM2:PC:STATE:ESTOP':        C.RED,
            'STM2:PC:ERR:TIMEOUT':        C.RED,
        }
        color = C.RED if 'ERR' in line else color_map.get(line, C.GRAY)
        self.stm2_log.append(f'{C.GRAY}[{ts}]{C.RESET} {color}{line}{C.RESET}')
        if len(self.stm2_log) > 15:
            self.stm2_log.pop(0)

    def _on_state(self, msg: String):
        for item in msg.data.split(','):
            if ':' in item:
                k, v = item.split(':', 1)
                k = k.strip()
                v = v.strip()
                if k in self.flags:
                    self.flags[k] = (v == 'True') if v in ('True', 'False') else v

    def _on_hb(self, msg: String):
        now = time.time()
        if msg.data == 'pi1':
            self.pi1_last_hb = now
        elif msg.data == 'pi2':
            self.pi2_last_hb = now

    # ── 화면 갱신 ─────────────────────────────────────
    def _refresh(self):
        now = time.time()
        self.flags['pi1_alive'] = (now - self.pi1_last_hb) < 3.0
        self.flags['pi2_alive'] = (now - self.pi2_last_hb) < 3.0

        if self.mode == 'stm':
            self._draw_stm1()
        elif self.mode == 'stm2':
            self._draw_stm2()
        else:
            self._draw_flags()

    # ── 전체 상태 요약 (기본) ─────────────────────────
    def _draw_flags(self):
        os.system('clear')
        f = self.flags

        p1 = f.get('pi1_alive', False)
        p2 = f.get('pi2_alive', False)
        emg = f.get('emergency', False)
        sf = f.get('start_flag', False)
        s1 = f.get('stm_state', 'idle')
        s2 = f.get('stm2_state', 'idle')

        cam_conn = f.get('cam1_connected', False)
        cam_stat = f.get('cam1_status', 'disconnected')
        cam_det  = f.get('cam1_detect_label', 'none')
        cam_conf = f.get('cam1_last_conf', '0.00')

        p1dot = f'{C.GREEN}● 연결됨{C.RESET}' if p1 else f'{C.RED}● 미연결{C.RESET}'
        p2dot = f'{C.GREEN}● 연결됨{C.RESET}' if p2 else f'{C.RED}● 미연결{C.RESET}'
        emgdot = f'{C.RED}{C.BOLD}● ON{C.RESET}' if emg else f'{C.GRAY}○ OFF{C.RESET}'
        sf_dot = f'{C.GREEN}● True{C.RESET} ← CAM1 감지 완료' if sf else f'{C.GRAY}○ False{C.RESET}'

        s1col = {
            'idle': C.GRAY,
            'homing': C.YELLOW,
            'running': C.GREEN,
            'seeding': C.CYAN,
            'ejecting': C.BLUE,
            'waiting_scara': C.YELLOW,
            'error': C.RED,
        }.get(s1, C.YELLOW)

        s2col = {
            'idle': C.GRAY,
            'convey_run': C.GREEN,
            'ir_detected': C.CYAN,
            'z_fix': C.YELLOW,
            'harvesting': C.BLUE,
            'ejecting': C.BLUE,
            'error': C.RED,
        }.get(s2, C.YELLOW)

        cam_conn_txt = f'{C.GREEN}● 연결됨{C.RESET}' if cam_conn else f'{C.RED}● 미연결{C.RESET}'

        if cam_stat == 'normal':
            cam_stat_txt = f'{C.GREEN}정상{C.RESET}'
        elif cam_stat == 'delay':
            cam_stat_txt = f'{C.YELLOW}지연{C.RESET}'
        else:
            cam_stat_txt = f'{C.RED}끊김{C.RESET}'

        print(f'{C.BOLD}━━━ 상태표시 ━━━━━━━━━━━━━━━━━━━━━━━━━━━{C.RESET}')
        print()

        print(f'  Pi1 연결        : {p1dot}')
        print(f'  Pi2 연결        : {p2dot}')
        print(f'  긴급정지        : {emgdot}')
        print()

        print(f'{C.BOLD}  ── 첫번째 컨베이어 (STM1) ──{C.RESET}')
        print(f'  start_flag      : {sf_dot}')
        print(f'  STM1 상태       : {s1col}{C.BOLD}{s1}{C.RESET}')
        print()

        print(f'{C.BOLD}  ── 두번째 컨베이어 (STM2) ──{C.RESET}')
        print(f'  STM2 상태       : {s2col}{C.BOLD}{s2}{C.RESET}')
        print()

        print(f'{C.BOLD}  ── 카메라 (CAM1) ──{C.RESET}')
        print(f'  CAM1 장치       : {cam_conn_txt}')
        print(f'  CAM1 프레임     : {cam_stat_txt}')
        print(f'  마지막 검출     : {cam_det}')
        print(f'  마지막 conf     : {cam_conf}')

        print(f'\n{C.GRAY}모드: 기본(status) / stm / stm2 | 0.5초마다 갱신 | Ctrl+C 종료{C.RESET}')

    # ── STM1 로그 화면 ───────────────────────────────
    def _draw_stm1(self):
        os.system('clear')
        stm = self.flags.get('stm_state', 'idle')
        print(f'{C.BOLD}━━━ STM32 #1 (첫번째 컨베이어) ━━━━━━━━━━{C.RESET}')
        print()

        steps = [
            ('idle',          '① 대기'),
            ('homing',        '② 홈잉'),
            ('running',       '③ 컨베이어 구동'),
            ('seeding',       '④ 파종'),
            ('ejecting',      '⑤ 배출'),
            ('waiting_scara', '⑥ 스카라 대기'),
            ('error',         '에러/정지'),
        ]
        order = [s[0] for s in steps]
        cur = order.index(stm) if stm in order else 0

        for i, (key, label) in enumerate(steps):
            if i == cur:
                print(f'  {C.GREEN}▶ {label}{C.RESET}')
            elif i < cur:
                print(f'  {C.GRAY}✓ {label}{C.RESET}')
            else:
                print(f'  {C.GRAY}  {label}{C.RESET}')

        print()
        print(f'{C.BOLD}━━━ STM32 #1 로그 ━━━━━━━━━━━━━━━━━━━━━━━{C.RESET}')
        print()
        if not self.stm1_log:
            print(f'  {C.GRAY}(수신 없음){C.RESET}')
        else:
            for line in self.stm1_log:
                print(f'  {line}')
        print(f'\n{C.GRAY}0.5초마다 갱신 | Ctrl+C 종료{C.RESET}')

    # ── STM2 시퀀스 화면 ─────────────────────────────
    def _draw_stm2(self):
        os.system('clear')
        stm2 = self.flags.get('stm2_state', 'idle')
        p2   = self.flags.get('pi2_alive', False)
        emg  = self.flags.get('emergency', False)

        p2dot  = f'{C.GREEN}● 연결됨{C.RESET}' if p2 else f'{C.RED}● 미연결{C.RESET}'
        emgdot = f'{C.RED}{C.BOLD}⛔ ON{C.RESET}' if emg else f'{C.GRAY}○ OFF{C.RESET}'

        print(f'{C.BOLD}━━━ STM32 #2 (두번째 컨베이어) ━━━━━━━━━━{C.RESET}')
        print()
        print(f'  Pi2 연결   : {p2dot}')
        print(f'  긴급정지   : {emgdot}')
        print()

        steps = [
            ('idle',        '① 대기'),
            ('convey_run',  '② 컨베이어 구동'),
            ('ir_detected', '③ IR 감지'),
            ('z_fix',       '④ Z축 고정'),
            ('harvesting',  '⑤ 수확'),
            ('ejecting',    '⑥ 배출'),
            ('error',       '에러/정지'),
        ]
        order = [s[0] for s in steps]
        cur = order.index(stm2) if stm2 in order else 0

        for i, (key, label) in enumerate(steps):
            if i == cur:
                print(f'  {C.GREEN}▶ {label}{C.RESET}')
            elif i < cur:
                print(f'  {C.GRAY}✓ {label}{C.RESET}')
            else:
                print(f'  {C.GRAY}  {label}{C.RESET}')

        print()
        print(f'{C.BOLD}━━━ STM32 #2 로그 ━━━━━━━━━━━━━━━━━━━━━━━{C.RESET}')
        print()
        if not self.stm2_log:
            print(f'  {C.GRAY}(수신 없음){C.RESET}')
        else:
            for line in self.stm2_log:
                print(f'  {line}')
        print(f'\n{C.GRAY}0.5초마다 갱신 | Ctrl+C 종료{C.RESET}')


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()