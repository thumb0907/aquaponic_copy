#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import sys
import time


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

        self.stm_state = 'idle'
        self.stm_log = []
        self.flags = {
            'start_flag': False,
            'stm_state':  'idle',
            'pi1_alive':  False,
            'emergency':  False,
        }
        self.pi1_last_hb = 0.0
        self.mode = sys.argv[1] if len(sys.argv) > 1 else 'flags'

        self.create_subscription(String, '/pi1/uart_response', self._on_uart, 10)
        self.create_subscription(String, '/monitor/state', self._on_state, 10)
        self.create_subscription(String, '/system/heartbeat', self._on_hb, 10)

        self.create_timer(0.5, self._refresh)

    def _on_uart(self, msg: String):
        line = msg.data.strip()
        ts = time.strftime('%H:%M:%S')

        if line == 'STM1:PC:STATE:HOMING':
            color = C.YELLOW
            self.stm_state = 'homing'

        elif line == 'STM1:PC:STATE:RUN_CONVEYOR1':
            color = C.GREEN
            self.stm_state = 'running'

        elif line == 'STM1:PC:STATE:SEEDING':
            color = C.CYAN
            self.stm_state = 'seeding'

        elif line == 'STM1:PC:STATE:EJECTING':
            color = C.BLUE
            self.stm_state = 'ejecting'

        elif line == 'STM1:PC:STATE:WAIT_SCARA_PICK':
            color = C.YELLOW
            self.stm_state = 'waiting_scara'

        elif line == 'STM1:PC:STATE:SCARA_PICK_STARTED':
            color = C.YELLOW
            self.stm_state = 'waiting_scara'
        
        elif line == 'STM1:PC:STATE:ESTOP':
            color = C.RED
            self.stm_state = 'error'

        elif line == 'STM1:PC:DONE:CYCLE1':
            color = C.GREEN
            self.stm_state = 'idle'

        elif line.startswith('STM1:PC:ERR:'):
            color = C.RED
            self.stm_state = 'error'

        elif line.startswith('STM1:PI1:ACK:'):
            color = C.GRAY

        else:
            color = C.GRAY

        self.stm_log.append(
            f'{C.GRAY}[{ts}]{C.RESET} {color}{line}{C.RESET}'
        )
        if len(self.stm_log) > 15:
            self.stm_log.pop(0)

    def _on_state(self, msg: String):
        for item in msg.data.split(','):
            if ':' in item:
                k, v = item.split(':', 1)
                k = k.strip()
                v = v.strip()
                if k in self.flags:
                    if v in ('True', 'False'):
                        self.flags[k] = (v == 'True')
                    else:
                        self.flags[k] = v

    def _on_hb(self, msg: String):
        if msg.data == 'pi1':
            self.pi1_last_hb = time.time()

    def _refresh(self):
        alive = (time.time() - self.pi1_last_hb) < 3.0
        self.flags['pi1_alive'] = alive

        if self.mode == 'stm':
            self._draw_stm()
        else:
            self._draw_flags()

    def _draw_stm(self):
        os.system('clear')
        print(f'{C.BOLD}━━━ STM32 동작 상태 ━━━━━━━━━━━━━━━━━━━━━{C.RESET}')
        print()

        state_color = {
            'idle':          C.GRAY,
            'homing':        C.YELLOW,
            'running':       C.GREEN,
            'seeding':       C.CYAN,
            'ejecting':      C.BLUE,
            'waiting_scara': C.YELLOW,
            'error':         C.RED,
        }.get(self.stm_state, C.GRAY)

        state_label = {
            'idle':          '대기 중',
            'homing':        '홈잉 진행 중...',
            'running':       '컨베이어 구동 중...',
            'seeding':       '파종 진행 중...',
            'ejecting':      '배출 진행 중...',
            'waiting_scara': '스카라 픽업 대기',
            'error':         '에러/정지',
        }.get(self.stm_state, self.stm_state)

        print(f'  현재 상태: {state_color}{C.BOLD}{state_label}{C.RESET}')
        print()

        steps = [
            ('idle',          '① 대기'),
            ('homing',        '② 홈잉'),
            ('running',       '③ 컨베이어 구동'),
            ('seeding',       '④ 파종'),
            ('ejecting',      '⑤ 배출'),
            ('waiting_scara', '⑥ 스카라 대기'),
            ('error',         '⑦ 에러/정지'),
        ]

        order = [s[0] for s in steps]
        cur = order.index(self.stm_state) if self.stm_state in order else 0

        for i, (key, label) in enumerate(steps):
            if i == cur:
                print(f'  {C.GREEN}▶ {label}{C.RESET}')
            elif i < cur:
                print(f'  {C.GRAY}✓ {label}{C.RESET}')
            else:
                print(f'  {C.GRAY}  {label}{C.RESET}')

        print()
        print(f'{C.BOLD}━━━ STM32 로그 ━━━━━━━━━━━━━━━━━━━━━━━━━{C.RESET}')
        print()

        if not self.stm_log:
            print(f'  {C.GRAY}(수신 없음){C.RESET}')
        else:
            for line in self.stm_log:
                print(f'  {line}')

        print()
        print(f'{C.GRAY}0.5초마다 갱신 | Ctrl+C 종료{C.RESET}')

    def _draw_flags(self):
        os.system('clear')
        print(f'{C.BOLD}━━━ 플래그 상태 (Phase 1) ━━━━━━━━━━━━━━{C.RESET}')
        print()

        alive = self.flags.get('pi1_alive', False)
        dot = f'{C.GREEN}● 연결됨{C.RESET}' if alive else f'{C.RED}● 끊김{C.RESET}'
        print(f'  Pi1 연결    :  {dot}')

        emg = self.flags.get('emergency', False)
        dot = f'{C.RED}{C.BOLD}● ON{C.RESET}' if emg else f'{C.GRAY}○ OFF{C.RESET}'
        print(f'  긴급정지    :  {dot}')

        print()
        print(f'{C.BOLD}  ── Phase 1 플래그 ──{C.RESET}')
        print()

        sf = self.flags.get('start_flag', False)
        dot = f'{C.GREEN}● True {C.RESET} ← CAM1 이벤트 전송됨' if sf else f'{C.GRAY}○ False{C.RESET}'
        print(f'  start_flag  :  {dot}')

        st = self.flags.get('stm_state', 'idle')
        color = {
            'idle':          C.GRAY,
            'homing':        C.YELLOW,
            'running':       C.GREEN,
            'seeding':       C.CYAN,
            'ejecting':      C.BLUE,
            'waiting_scara': C.YELLOW,
            'error':         C.RED,
        }.get(st, C.GRAY)
        print(f'  stm_state   :  {color}{C.BOLD}{st}{C.RESET}')

        print()
        print(f'{C.GRAY}0.5초마다 갱신 | Ctrl+C 종료{C.RESET}')

def main():
    rclpy.init()
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()