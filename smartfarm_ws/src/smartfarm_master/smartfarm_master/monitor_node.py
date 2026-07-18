#!/usr/bin/env python3
"""
monitor_node.py — 터미널 대시보드

실행:
  ros2 run smartfarm_master monitor_node stm    # STM32 상태
  ros2 run smartfarm_master monitor_node flags  # 전체 플래그
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

        # STM32 로그 (최근 15줄)
        self.stm_log = []

        # 전체 플래그 (master_node와 동일한 구조)
        self.status = {
            'start_flag': False,
            'stm_state':  'idle',
            'stm2_state': 'idle',
            'pi1_alive':  False,
            'pi2_alive':  False,
            'pi3_alive':  False,
            'emergency':  False,
            'estop_stm1': False,
            'estop_scara': False,
        }

        # 수경재배실 센서 캐시 — None이면 아직 수신 안 된 것
        self.sensor_cache: dict[str, float | None] = {
            'TDS':        None,   # TDS (ppm)
            'PH':         None,   # pH
            'WATER_TEMP': None,   # DS18B20 수온 (℃)
            'AIR_TEMP':   None,   # DHT22 기온 (℃)
            'HUMIDITY':   None,   # DHT22 습도 (%)
        }
        self.flags = {
            'ssf':  0,
            'smf':  0,
            'crf':  0,
            'uv':   0,
            'ulf':  0,
            'urf':  0,
            'wcnt': 0,
            'wlf':  0,
            'wrf':  0,
            'ff':   0,
            'uef':  0,
            'wef':  0,
            'hf':   0,
            'c1f':  0,
            'c2f':  0,
            'hmf':  0,
            's2f':       0,
            's3f':       0,
            'scara_src': 0,
            'scara_dst': 0,
        }

        self.pi1_last_hb = 0.0
        self.pi2_last_hb = 0.0
        self.pi3_last_hb = 0.0

        self.create_subscription(
            String, '/pi1/uart_response',
            self._on_uart, 10)
        self.create_subscription(
            String, '/monitor/state',
            self._on_state, 10)
        self.create_subscription(
            String, '/system/heartbeat',
            self._on_hb, 10)

        self.create_timer(0.5, self._refresh)

    def _on_uart(self, msg: String):
        line = msg.data.strip()
        ts   = time.strftime('%H:%M:%S')

        # 색상 결정
        if 'STATE:HOMING' in line:
            color = C.YELLOW
        elif 'STATE:RUN_CONVEYOR1' in line:
            color = C.GREEN
        elif 'STATE:SEEDING' in line:
            color = C.CYAN
        elif 'STATE:EJECTING' in line:
            color = C.BLUE
        elif 'STATE:WAIT_SCARA' in line:
            color = C.YELLOW
        elif 'DONE:CYCLE1' in line:
            color = C.GREEN
        elif 'STATE:ESTOP' in line or 'ERR:' in line:
            color = C.RED
        elif 'FLAG:' in line:
            color = C.CYAN
        else:
            color = C.GRAY

        self.stm_log.append(
            f'{C.GRAY}[{ts}]{C.RESET} {color}{line}{C.RESET}')
        if len(self.stm_log) > 15:
            self.stm_log.pop(0)

    def _on_state(self, msg: String):
        """master_node가 0.5초마다 보내는 전체 상태 수신"""
        for item in msg.data.split(','):
            if ':' in item:
                k, v = item.split(':', 1)
                k = k.strip()
                v = v.strip()

                if k in self.status:
                    if v in ('True', 'False'):
                        self.status[k] = (v == 'True')
                    else:
                        self.status[k] = v

                elif k in self.flags:
                    try:
                        self.flags[k] = int(v)
                    except ValueError:
                        pass

                # sensor_tds, sensor_ph 등 센서값 파싱
                # master_node가 "sensor_tds:512.00" 형식으로 보냄
                elif k.startswith('sensor_'):
                    sensor_key = k[7:].upper()   # "sensor_tds" → "TDS"
                    if sensor_key in self.sensor_cache:
                        try:
                            self.sensor_cache[sensor_key] = float(v)
                        except ValueError:
                            pass

    def _on_hb(self, msg: String):
        if msg.data == 'pi1':
            self.pi1_last_hb = time.time()
        elif msg.data == 'pi2':
            self.pi2_last_hb = time.time()
        elif msg.data == 'pi3':
            self.pi3_last_hb = time.time()

    def _refresh(self):
        self.status['pi1_alive'] = (time.time() - self.pi1_last_hb) < 3.0
        self.status['pi2_alive'] = (time.time() - self.pi2_last_hb) < 3.0
        self.status['pi3_alive'] = (time.time() - self.pi3_last_hb) < 3.0

        if self.mode == 'stm':
            self._draw_stm()
        else:
            self._draw_flags()

    # ── STM32 동작 상태 화면 ──────────────────
    def _draw_stm(self):
        os.system('clear')
        st = self.status.get('stm_state', 'idle')

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
        }.get(st, C.GRAY)

        state_label = {
            'idle':          '대기 중',
            'homing':        '홈잉 진행 중...',
            'running':       '컨베이어 구동 중...',
            'seeding':       '파종 진행 중...',
            'ejecting':      '배출 진행 중...',
            'waiting_scara': '스카라 픽업 대기',
            'error':         '에러/정지',
        }.get(st, st)

        print(f'  현재 상태: {state_color}{C.BOLD}{state_label}{C.RESET}')

        stm1_estop = self.status.get('estop_stm1', False)
        estop_label = f'{C.RED}{C.BOLD}ESTOP{C.RESET}' if stm1_estop else f'{C.GRAY}NORMAL{C.RESET}'
        print(f'  STM1 ESTOP: {estop_label}')
        print()

        steps = [
            ('idle',          '① 대기'),
            ('homing',        '② 홈잉'),
            ('running',       '③ 컨베이어 구동'),
            ('seeding',       '④ 파종'),
            ('ejecting',      '⑤ 배출'),
            ('waiting_scara', '⑥ 스카라 대기'),
            ('error',         '   에러/정지'),
        ]
        order = [s[0] for s in steps]
        cur   = order.index(st) if st in order else 0

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

    # ── 전체 플래그 화면 ──────────────────────
    def _draw_flags(self):
        os.system('clear')

        alive  = self.status.get('pi1_alive', False)
        alive2 = self.status.get('pi2_alive', False)
        alive3 = self.status.get('pi3_alive', False)
        emg    = self.status.get('emergency', False)
        sf     = self.status.get('start_flag', False)
        st     = self.status.get('stm_state', 'idle')

        print(f'{C.BOLD}━━━ 전체 플래그 상태 ━━━━━━━━━━━━━━━━━━━{C.RESET}')
        print()

        # 연결 상태
        p1dot = f'{C.GREEN}● 연결됨{C.RESET}' if alive  else f'{C.RED}● 끊김{C.RESET}'
        p2dot = f'{C.GREEN}● 연결됨{C.RESET}' if alive2 else f'{C.RED}● 끊김{C.RESET}'
        p3dot = f'{C.GREEN}● 연결됨{C.RESET}' if alive3 else f'{C.RED}● 끊김{C.RESET}'
        print(f'  Pi1 연결    :  {p1dot}')
        print(f'  Pi2 연결    :  {p2dot}')
        print(f'  Pi3 연결    :  {p3dot}')

        emgdot = f'{C.RED}{C.BOLD}● ON{C.RESET}' if emg else f'{C.GRAY}○ OFF{C.RESET}'
        print(f'  긴급정지    :  {emgdot}')

        stm1_estop = self.status.get('estop_stm1', False)
        scara_estop = self.status.get('estop_scara', False)
        stm1dot = f'{C.RED}{C.BOLD}● ESTOP{C.RESET}' if stm1_estop else f'{C.GRAY}○ 정상{C.RESET}'
        scaradot = f'{C.RED}{C.BOLD}● ESTOP{C.RESET}' if scara_estop else f'{C.GRAY}○ 정상{C.RESET}'
        print(f'  STM1 정지상태 :  {stm1dot}')
        print(f'  SCARA 정지상태:  {scaradot}')
        print()

        # STM1 상태
        state_color = {
            'idle':          C.GRAY,
            'homing':        C.YELLOW,
            'running':       C.GREEN,
            'seeding':       C.CYAN,
            'ejecting':      C.BLUE,
            'waiting_scara': C.YELLOW,
            'error':         C.RED,
        }.get(st, C.GRAY)
        sfdot = f'{C.GREEN}● True{C.RESET}' \
                if sf else f'{C.GRAY}○ False{C.RESET}'

        print(f'{C.BOLD}  ── STM1 상태 ──{C.RESET}')
        print(f'  start_flag  :  {sfdot}')
        print(f'  stm_state   :  {state_color}{C.BOLD}{st}{C.RESET}')
        print()

        # 스카라 관련 플래그
        print(f'{C.BOLD}  ── 스카라 ──{C.RESET}')
        self._print_flag('ssf',  '스카라 시작 명령')
        self._print_flag('smf',  '스카라 동작 중')
        self._print_flag('crf',  '직교로봇 리셋 명령')
        self._print_flag('hmf',  '스카라 사전 홈잉')
        self._print_flag('s2f', '스카라 sect2 시작')
        self._print_flag('s3f', '스카라 sect3 시작')
        self._print_flag('scara_src', '스카라 출발 슬롯')
        self._print_flag('scara_dst', '스카라 도착 슬롯')
        print()

        # UV실
        print(f'{C.BOLD}  ── UV실 ──{C.RESET}')
        self._print_flag('uv',   f'트레이 개수 (최대2)')
        self._print_flag('ulf',  '왼쪽 발아 완료')
        self._print_flag('urf',  '오른쪽 발아 완료')
        print()

        # 수경재배실
        print(f'{C.BOLD}  ── 수경재배실 ──{C.RESET}')
        self._print_flag('wcnt', '트레이 개수 (최대2)')
        self._print_flag('wlf',  '왼쪽 성장 완료')
        self._print_flag('wrf',  '오른쪽 성장 완료')
        print()

        # 수경재배실 센서 (STM3)
        print(f'{C.BOLD}  ── 수경재배실 센서 (STM3) ──{C.RESET}')
        self._print_sensor('TDS',        'TDS',      'ppm')
        self._print_sensor('PH',         'pH',       '')
        self._print_sensor('WATER_TEMP', '수온',     '℃')
        self._print_sensor('AIR_TEMP',   '기온',     '℃')
        self._print_sensor('HUMIDITY',   '습도',     '%')
        print()

        # 수확
        print(f'{C.BOLD}  ── 수확 ──{C.RESET}')
        self._print_flag('ff',   '트레이 고정')
        self._print_flag('hf',   '수확 완료 응답')
        self._print_flag('uef',  '수확 중 파종 완료 이벤트')
        self._print_flag('wef',  '수확 중 발아 완료 이벤트')
        print()

        # 컨베이어
        print(f'{C.BOLD}  ── 컨베이어 ──{C.RESET}')
        self._print_flag('c1f',  '1번 컨베이어')
        self._print_flag('c2f',  '2번 컨베이어')

        print()
        print(f'{C.GRAY}0.5초마다 갱신 | Ctrl+C 종료{C.RESET}')

    def _print_flag(self, key: str, desc: str):
        """플래그 한 줄 출력"""
        val = self.flags.get(key, 0)

        # 1이면 초록, 0이면 회색
        if val:
            dot = f'{C.GREEN}● {val}{C.RESET}'
        else:
            dot = f'{C.GRAY}○ {val}{C.RESET}'

        print(f'  {key:<6}  ({desc:<22}) :  {dot}')

    def _print_sensor(self, key: str, label: str, unit: str):
        """센서값 한 줄 출력 — None이면 '수신 대기' 표시"""
        val = self.sensor_cache.get(key)
        if val is None:
            dot = f'{C.GRAY}○ 수신 대기{C.RESET}'
        else:
            val_str = f'{val:.2f} {unit}'.strip()
            dot     = f'{C.CYAN}● {val_str}{C.RESET}'
        print(f'  {label:<8}  ({key:<12}) :  {dot}')


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