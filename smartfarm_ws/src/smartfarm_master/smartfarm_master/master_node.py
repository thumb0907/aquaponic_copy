#!/usr/bin/env python3
"""
master_node.py — PC 마스터 노드

역할:
  1. Pi1 카메라 영상 수신 → YOLO 추론 → 트레이 감지
  2. 트레이 감지 시 SSF=1 바이너리 프레임 → Pi1 → STM1
  3. STM1/STM2 상태 수신 → 플래그 관리
  4. 전체 플래그를 모니터 노드에 publish
  5. 긴급정지/리셋 명령 처리
  6. Pi3(수경재배실) 센서값 수신 → sensor_cache 저장 → 모니터 publish
"""
from __future__ import annotations

import time
import threading
import socket
import struct
import queue

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

frame_queue = queue.Queue(maxsize=2)
nursery_left_frame_queue = queue.Queue(maxsize=2)
nursery_right_frame_queue = queue.Queue(maxsize=2)

# ── 설정 ──────────────────────────────────────
# 컨베이어 트레이인식 카메라
MODEL_PATH    = '/home/thumb/aquaponic_copy/tray2/best.pt'
STREAM_PORT   = 5000
TRAY_CLASS_ID = 0
MIN_CONF      = 0.60
STABLE_FRAMES = 5
COOLDOWN_SEC  = 2.0

# 컨베이어 ROI (트레이 감지 유효 영역)
ROI_X_MIN = 0.25
ROI_X_MAX = 0.85
ROI_Y_MIN = 0.05
ROI_Y_MAX = 0.95
# 발아실 ROI
NURSERY_ROI_X_MIN = 0.20
NURSERY_ROI_X_MAX = 0.80
NURSERY_ROI_Y_MIN = 0.15
NURSERY_ROI_Y_MAX = 0.90
NURSERY_TRAY_MIN_BOX_RATIO = 0.02
NURSERY_SPROUT_MIN_BOX_RATIO = 0.005

# 박스 최소 크기 (화면 대비 비율)
MIN_BOX_RATIO = 0.20

# 캘리브레이션 파일
CALIB_PATH = '/home/thumb/aquaponic_copy/smartfarm_ws/camera_calib.npz'
NURSERY_LEFT_CALIB_PATH = '/home/thumb/aquaponic_copy/smartfarm_ws/uv_left_calib.npz'
NURSERY_RIGHT_CALIB_PATH = '/home/thumb/aquaponic_copy/smartfarm_ws/uv_right_calib.npz'

# 발아실 카메라 / 모델
NURSERY_MODEL_PATH = '/home/thumb/aquaponic_copy/best.pt'
NURSERY_LEFT_STREAM_PORT = 5001
NURSERY_RIGHT_STREAM_PORT = 5002

NURSERY_MIN_CONF = 0.05
NURSERY_STABLE_FRAMES = 5
NURSERY_COOLDOWN_SEC = 5.0

TRAY_OCCUPY_FRAMES = 3
TRAY_LOST_FRAMES = 10
# ── 프로토콜 상수 (comm.h / Serial.h 와 동일) ──
SOF = 0xAA

PID_SSF   = 0x01
PID_SMF   = 0x02
PID_CRF   = 0x03
PID_UV    = 0x04
PID_ULF   = 0x05
PID_URF   = 0x06
PID_WCNT  = 0x07
PID_WLF   = 0x08
PID_WRF   = 0x09
PID_FF    = 0x0A
PID_UEF   = 0x0B
PID_WEF   = 0x0C
PID_HF    = 0x0D
PID_C1F   = 0x0E
PID_C2F   = 0x0F
PID_ESTOP = 0x10
PID_RESET = 0x11


def make_frame(pid: int, data: bytes = b'') -> list:
    """바이너리 프레임 생성 [SOF, ID, LEN, DATA..., CHK]"""
    length = len(data)
    chk = (pid + length + sum(data)) & 0xFF
    return [SOF, pid, length] + list(data) + [chk]


def make_flag_u8(pid: int, val: int) -> list:
    """1바이트 플래그 프레임"""
    return make_frame(pid, bytes([val & 0xFF]))


def make_flag_u16(pid: int, val: int) -> list:
    """2바이트 플래그 프레임 (UV, WCNT용)"""
    return make_frame(pid, bytes([(val >> 8) & 0xFF, val & 0xFF]))


def make_estop() -> list:
    """긴급정지 프레임"""
    return make_frame(PID_ESTOP)


def make_reset() -> list:
    """리셋 프레임"""
    return make_frame(PID_RESET)


class MasterNode(Node):

    def __init__(self):
        super().__init__('master_node')

        self.state_lock = threading.Lock()

        # ── 공유 플래그 ───────────────────────
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
        }

        # ── STM1 상태 ─────────────────────────
        self.start_flag      = False
        self.stm_state       = 'idle'
        self.stable_cnt      = 0
        self.no_tray_cnt     = 0
        self.last_tx         = 0.0
        self.pi1_alive       = False
        self.pi1_last_hb     = 0.0
        self.pi1_alive_prev  = False  # 연결/끊김 변화 감지용

        # ── STM2 상태 ─────────────────────────
        self.stm2_state      = 'idle'
        self.pi2_alive       = False
        self.pi2_last_hb     = 0.0
        self.pi2_alive_prev  = False  # 연결/끊김 변화 감지용

        # ── Pi3 / STM3 (수경재배실 센서허브) ──
        self.pi3_alive       = False
        self.pi3_last_hb     = 0.0
        self.pi3_alive_prev  = False

        # 센서 캐시 — None이면 아직 수신 안 된 것, 모니터 publish에 포함
        self.sensor_cache: dict[str, float | None] = {
            'TDS':        None,   # TDS (ppm)
            'PH':         None,   # pH
            'WATER_TEMP': None,   # DS18B20 수온 (℃)
            'AIR_TEMP':   None,   # DHT22 기온 (℃)
            'HUMIDITY':   None,   # DHT22 습도 (%)
        }

        # ── 카메라 상태 ───────────────────────
        # 컨베이어 트레이인식 카메라
        self.cam1_connected      = False
        self.cam1_last_frame_ts  = 0.0
        self.cam1_last_detect_ts = 0.0
        self.cam1_last_conf      = 0.0
        self.cam1_status         = 'disconnected'
        self.cam1_detect_label   = 'none'
        self.cam1_last_bbox      = None
        self.cam1_last_bbox_ts   = 0.0
        self.cam1_overlay_hold_sec = 0.7
        self.cam1_last_conf_hold = 0.0
        # 발아실 카메라
        self.nursery_state = {
            'left': {
                'stable_cnt': 0,
                'last_tx': 0.0,
                'last_label': 'none',
                'last_conf': 0.0,
                'occupied': 0,
                'tray_seen_cnt': 0,
                'tray_lost_cnt': 0,
            },
            'right': {
                'stable_cnt': 0,
                'last_tx': 0.0,
                'last_label': 'none',
                'last_conf': 0.0,
                'occupied': 0,
                'tray_seen_cnt': 0,
                'tray_lost_cnt': 0,
            },  
        }   

        # ── 공통 ──────────────────────────────
        self.emergency = False

        # ── YOLO 모델 ─────────────────────────
        print('[YOLO] 모델 로딩 중...')
        self.model = YOLO(MODEL_PATH)
        print('[YOLO] 로딩 완료')

        print('[YOLO] 발아실 모델 로딩 중...')
        self.nursery_model = YOLO(NURSERY_MODEL_PATH)
        print('[YOLO] 발아실 모델 로딩 완료')
        print('[Nursery model names]', self.nursery_model.names)


        # ── Publisher ─────────────────────────
        # Pi1 → STM1 명령 (바이너리)
        self.pub_pi1     = self.create_publisher(
            UInt8MultiArray, '/pi1/uart_cmd', 10)
        # Pi2 → STM2/스카라/매니퓰 명령 (바이너리)
        self.pub_pi2     = self.create_publisher(
            UInt8MultiArray, '/pi2/uart_cmd', 10)
        # Pi3 → STM3 명령 (바이너리)
        self.pub_pi3     = self.create_publisher(
            UInt8MultiArray, '/pi3/uart_cmd', 10)
        # 모니터 노드용
        self.pub_monitor = self.create_publisher(
            String, '/monitor/state', 10)

        # ── Subscriber ────────────────────────
        self.create_subscription(
            String, '/pi1/uart_response',
            self._on_uart1, 10)
        self.create_subscription(
            String, '/pi2/uart_response',
            self._on_uart2, 10)
        self.create_subscription(
            String, '/pi2/flag_update',
            self._on_pi2_flag, 10)
        # Pi3 — STM3 상태 수신
        self.create_subscription(
            String, '/pi3/uart_response',
            self._on_uart3, 10)
        # Pi3 — 센서값 수신 (즉시 publish 분 + 1초 주기 캐시 publish 분 모두 여기로)
        self.create_subscription(
            String, '/pi3/sensor_data',
            self._on_sensor_data, 10)
        self.create_subscription(
            String, '/system/heartbeat',
            self._on_heartbeat, 10)
        self.create_subscription(
            String, '/system/command',
            self._on_command, 10)

        self.create_timer(1.0, self._check_heartbeat)
        self.create_timer(0.5, self._publish_monitor)

        self.get_logger().info('Master 노드 시작')


    # 플래그 조작 함수
    def _set_flag(self, name: str, value: int):
        if name in self.flags:
            self.flags[name] = int(value)

    def _inc_flag(self, name: str, delta: int, min_v: int = 0, max_v: int | None = None):
        if name not in self.flags:
            return

        v = int(self.flags[name]) + int(delta)

        if v < min_v:
            v = min_v
        if max_v is not None and v > max_v:
            v = max_v

        self.flags[name] = v
    
    # 플래그 공유 및 재배포
    def _broadcast_flags_to_scara(self):
        self._send_scara(make_flag_u8(PID_SSF, self.flags['ssf']))
        self._send_scara(make_flag_u8(PID_SMF, self.flags['smf']))
        self._send_scara(make_flag_u8(PID_CRF, self.flags['crf']))
        self._send_scara(make_flag_u16(PID_UV, self.flags['uv']))
        self._send_scara(make_flag_u16(PID_WCNT, self.flags['wcnt']))
        self._send_scara(make_flag_u8(PID_ULF, self.flags['ulf']))
        self._send_scara(make_flag_u8(PID_URF, self.flags['urf']))
        self._send_scara(make_flag_u8(PID_WLF, self.flags['wlf']))
        self._send_scara(make_flag_u8(PID_WRF, self.flags['wrf']))
        self._send_scara(make_flag_u8(PID_UEF, self.flags['uef']))
        self._send_scara(make_flag_u8(PID_WEF, self.flags['wef']))

    def _broadcast_flags_to_stm2(self):
        self._send_binary_pi2(make_flag_u8(PID_FF, self.flags['ff']))
        self._send_binary_pi2(make_flag_u16(PID_WCNT, self.flags['wcnt']))
        self._send_binary_pi2(make_flag_u8(PID_WLF, self.flags['wlf']))
        self._send_binary_pi2(make_flag_u8(PID_WRF, self.flags['wrf']))
        self._send_binary_pi2(make_flag_u8(PID_UEF, self.flags['uef']))
        self._send_binary_pi2(make_flag_u8(PID_WEF, self.flags['wef']))

    def _broadcast_all_flags(self):
        self._broadcast_flags_to_scara()
        self._broadcast_flags_to_stm2()

    # ══════════════════════════════════════════
    # STM1 상태 수신
    # ══════════════════════════════════════════
    def _on_uart1(self, msg: String):
        line = msg.data.strip()
        self.get_logger().info(f'[STM1] {line}')

        with self.state_lock:
            if line == 'STM1:PC:STATE:HOMING':
                self.stm_state = 'homing'
            elif line == 'STM1:PC:STATE:RUN_CONVEYOR1':
                self.stm_state = 'running'
            elif line == 'STM1:PC:STATE:SEEDING':
                self.stm_state = 'seeding'
            elif line == 'STM1:PC:STATE:EJECTING':
                self.stm_state = 'ejecting'
            elif line == 'STM1:PC:STATE:WAIT_SCARA_PICK':
                self.stm_state = 'waiting_scara'
            elif line == 'STM1:PC:STATE:ESTOP':
                self.stm_state  = 'error'
                self.start_flag = False
                self.get_logger().warn('[STM1] ESTOP')
            elif line == 'STM1:PC:DONE:CYCLE1':
                self.stm_state = 'waiting_scara'

                # 스카라 유휴 + UV실 자리가 있을 때만 시작 허용
                if self.flags['smf'] == 0 and self.flags['uv'] < 2:
                    self._set_flag('ssf', 1)
                    self._set_flag('crf', 1)

                    # 스카라 시작 지시
                    self._send_scara(make_flag_u8(PID_SSF, 1))
                    self._send_scara(make_flag_u8(PID_CRF, 1))
                    self._send_binary(make_flag_u8(PID_CRF, 1))
                    
                    self.get_logger().info(
                        f'파종 완료 → SSF=1, CRF=1 승인 (smf={self.flags["smf"]}, uv={self.flags["uv"]})'
                    )
                else:
                    self.get_logger().warn(
                        f'파종 완료지만 스카라 busy 또는 UV full: smf={self.flags["smf"]}, uv={self.flags["uv"]}'
                    )
            elif line == 'STM1:PC:STATE:IDLE':
                self.stm_state  = 'idle'
                self.start_flag = False
                self._set_flag('c1f', 0)   
            elif line.startswith('STM1:PC:ERR:'):
                self.stm_state  = 'error'
                self.start_flag = False
                self.get_logger().error(line)
           
            elif line.startswith('STM1:PC:FLAG:'):
                self.get_logger().info(f'STM1 플래그 보고 수신: {line}')
            elif line.startswith('STM1:PI1:ACK:'):
                self.get_logger().info(f'ACK: {line}')

    # ══════════════════════════════════════════
    # STM2 상태 수신
    # ══════════════════════════════════════════
    def _on_uart2(self, msg: String):
        line = msg.data.strip()
        self.get_logger().info(f'[STM2] {line}')

        # 스카라가 CRF=0 전송 → STM1 초기화 트리거
        if line == 'SCARA:PC:FLAG:CRF:0':
            self.get_logger().info('스카라 CRF=0 → STM1에 CRF=0 전달')
            self._send_binary(make_flag_u8(PID_CRF, 0))
            with self.state_lock:
                self._set_flag('crf', 0)
            return

        if line.startswith('SCARA:PC:FLAG:'):
            parts = line.split(':')   # ['SCARA','PC','FLAG','UV','2']
            if len(parts) == 5:
                k = parts[3].lower()  # 'uv', 'smf' 등
                v = int(parts[4])
                with self.state_lock:
                    if k in self.flags:
                        self.flags[k] = v
                        self.get_logger().info(f'스카라 플래그 동기화: {k}={v}')
            return
        
        with self.state_lock:
            # ── 스카라 작업 완료 이벤트 ─────────────────────
            if line == 'SCARA:PC:EVENT:PUT_TO_UV_DONE':
                self._set_flag('smf', 0)
                self._set_flag('ssf', 0)
                self._set_flag('crf', 0)

                
                self._broadcast_all_flags()
                self.get_logger().info(
                    f'SCARA UV 적재 완료 → uv={self.flags["uv"]}, smf=0, ssf=0, crf=0'
                )
                return

            if line == 'SCARA:PC:EVENT:MOVE_UV_TO_WATER_DONE':
                self._set_flag('smf', 0)
                
                self._inc_flag('wcnt', +1, 0, 2)
                self._broadcast_all_flags()
                self.get_logger().info(
                    f'SCARA 수경 이동 완료 → uv는 카메라 기준 유지, '
                    f'wcnt={self.flags["wcnt"]}, smf=0'
                )
                return

            if line == 'SCARA:PC:EVENT:MOVE_WATER_TO_CONVEY2_DONE':
                self._set_flag('smf', 0)
                self._inc_flag('wcnt', -1, 0, 2)
                self._set_flag('ff', 1)
                self._broadcast_all_flags()
                self.get_logger().info(
                    f'SCARA 2번 컨베이어 적재 완료 → wcnt={self.flags["wcnt"]}, ff=1, smf=0'
                )
                return
            if line == 'SCARA:PC:EVENT:HARVEST_DONE':  # 스카라 프로토콜에 맞게 조정
                self._set_flag('ff', 0)
                self._send_stm2(make_flag_u8(PID_FF, 0))
                self.get_logger().info('수확 완료 → STM2에 FF=0 전송')
                return

            # ── STM2 이벤트 ────────────────────────────────
            if line == 'STM2:PC:EVENT:FIX_DONE':
                self._set_flag('ff', 1)
                self._broadcast_all_flags()
                self.get_logger().info('STM2 고정 완료 → ff=1')
                return

            if line == 'STM2:PC:EVENT:HARVEST_DONE':
                self._set_flag('ff', 0)
                self._broadcast_all_flags()
                self.get_logger().info('STM2 수확 완료 → ff=0')
                return
            

            # ── STM2 상태 수신 ────────────────────────────
            if line == 'STM2:PC:STATE:CONVEY_RUN':
                self.stm2_state = 'convey_run'
            elif line == 'STM2:PC:STATE:IR_DETECTED':
                self.stm2_state = 'ir_detected'
            elif line == 'STM2:PC:STATE:Z_FIX':
                self.stm2_state = 'z_fix'
            elif line == 'STM2:PC:STATE:HARVESTING':
                self.stm2_state = 'harvesting'
                # 수확 시작 → 스카라/매니퓰에 지시
                # (현재는 SKIP_COMM=1로 STM2가 자체 타이머로 처리하므로 일단 로그만)
                self.get_logger().info('STM2 수확 중 — 스카라/매니퓰 수확 지시 필요')

            elif line == 'STM2:PC:STATE:EJECTING':
                self.stm2_state = 'ejecting'
            elif line == 'STM2:PC:STATE:EJECT_DONE':   
                self.stm2_state = 'eject_done'
            elif line == 'STM2:PC:DONE:CYCLE2':
                self.stm2_state = 'idle'
                self.get_logger().info('[STM2] 사이클 완료')
            elif line == 'STM2:PC:STATE:RESET_DONE':
                self.stm2_state = 'idle'
            elif line == 'STM2:PC:STATE:ESTOP':
                self.stm2_state = 'error'
                self.get_logger().warn('[STM2] ESTOP')
            elif line.startswith('STM2:PC:ERR:'):
                self.stm2_state = 'error'
                self.get_logger().error(line)
            elif line.startswith('STM2:PI2:ACK:'):
                self.get_logger().info(f'ACK: {line}')
            
            

    # ══════════════════════════════════════════
    # Pi2 플래그 업데이트
    # TODO:
    # 최종 구조에서는 /pi2/flag_update 로 전역 플래그를 직접 덮어쓰지 않고,
    # 작업 완료 이벤트(EVENT) 기반으로만 PC가 flags를 갱신하도록 정리할 것.
    # ══════════════════════════════════════════
    def _on_pi2_flag(self, msg: String):
        """Pi2에서 오는 플래그 업데이트 — 형식: FLAG:SMF:1"""
        line  = msg.data.strip()
        parts = line.split(':')

        with self.state_lock:
            if len(parts) == 3 and parts[0] == 'FLAG':
                k = parts[1].lower()
                v = int(parts[2])
                if k in self.flags:
                    self.flags[k] = v
                    self.get_logger().info(f'Pi2 플래그: {k}={v}')

    # ══════════════════════════════════════════
    # Heartbeat + 카메라 상태 판정
    # ══════════════════════════════════════════
    def _on_heartbeat(self, msg: String):
        now = time.time()
        with self.state_lock:
            if msg.data == 'pi1':
                self.pi1_last_hb = now
            elif msg.data == 'pi2':
                self.pi2_last_hb = now
            elif msg.data == 'pi3':
                self.pi3_last_hb = now

    def _check_heartbeat(self):
        now = time.time()
        with self.state_lock:
            pi1_now = (now - self.pi1_last_hb) < 3.0
            pi2_now = (now - self.pi2_last_hb) < 3.0
            pi3_now = (now - self.pi3_last_hb) < 3.0

            # Pi1 연결 상태 변화 감지
            if pi1_now and not self.pi1_alive_prev:
                self.get_logger().info('[Pi1] 연결됨')
            elif not pi1_now and self.pi1_alive_prev:
                self.get_logger().warn('[Pi1] 연결 끊김')

            # Pi2 연결 상태 변화 감지
            if pi2_now and not self.pi2_alive_prev:
                self.get_logger().info('[Pi2] 연결됨')
            elif not pi2_now and self.pi2_alive_prev:
                self.get_logger().warn('[Pi2] 연결 끊김')

            # Pi3 연결 상태 변화 감지
            if pi3_now and not self.pi3_alive_prev:
                self.get_logger().info('[Pi3] 연결됨')
            elif not pi3_now and self.pi3_alive_prev:
                self.get_logger().warn('[Pi3] 연결 끊김')

            self.pi1_alive      = pi1_now
            self.pi2_alive      = pi2_now
            self.pi3_alive      = pi3_now
            self.pi1_alive_prev = pi1_now
            self.pi2_alive_prev = pi2_now
            self.pi3_alive_prev = pi3_now

            # 카메라 상태 판정
            if not self.cam1_connected:
                self.cam1_status = 'disconnected'
            else:
                dt = now - self.cam1_last_frame_ts
                if dt < 1.0:
                    self.cam1_status = 'normal'
                elif dt < 3.0:
                    self.cam1_status = 'delay'
                else:
                    self.cam1_status = 'disconnected'

            # 감지 라벨 타임아웃
            if (now - self.cam1_last_detect_ts) > 3.0:
                self.cam1_detect_label = 'none'

    # ══════════════════════════════════════════
    # 외부 명령
    # ══════════════════════════════════════════
    def _on_command(self, msg: String):
        cmd = msg.data.strip()

        if cmd == 'EMERGENCY':
            with self.state_lock:
                self.emergency  = True
                self.start_flag = False
                self._broadcast_all_flags()
            self._send_binary(make_estop())           # STM1
            self._send_binary_pi2(make_estop())       # STM2/스카라
            self._send_binary_pi3(make_estop())       # STM3
            self.get_logger().warn('EMERGENCY!')

        elif cmd == 'RESET':
            with self.state_lock:
                self.emergency         = False
                self.start_flag        = False
                self.stm_state         = 'idle'
                self.stm2_state        = 'idle'
                self.stable_cnt        = 0
                self.no_tray_cnt       = 0
                self.cam1_detect_label = 'none'
                self.cam1_last_conf    = 0.0
                for k in self.flags:
                    self.flags[k] = 0
                # 센서 캐시는 리셋해도 None으로 초기화하지 않음
                # (센서값은 STM3가 계속 보내므로 곧 갱신됨)
                self._broadcast_all_flags()
            self._send_binary(make_reset())           # STM1
            self._send_binary_pi2(make_reset())       # STM2/스카라
            self._send_binary_pi3(make_reset())       # STM3
            self.get_logger().info('RESET')

        elif cmd == 'START_2':
            with self.state_lock:
                if self.emergency:
                    self.get_logger().warn('긴급정지 중 — START_2 무시')
                    return
                if self.stm2_state != 'idle':
                    self.get_logger().warn(
                        f'STM2 동작 중({self.stm2_state}) — 무시')
                    return
            self._send_binary_pi2(make_flag_u8(PID_FF, 1))
            self.get_logger().info('STM2 시작 → FF=1 전송')

        else:
            self.get_logger().warn(f'알 수 없는 명령: {cmd}')

    # ══════════════════════════════════════════
    # 송신
    # ══════════════════════════════════════════
    def _send_binary(self, frame: list):
        """Pi1 → STM1 바이너리 전송"""
        msg      = UInt8MultiArray()
        msg.data = frame
        self.pub_pi1.publish(msg)

    def _send_binary_pi2(self, frame):
        """Pi2 → STM2 바이너리 전송 (식별자 없음 → STM2 fallback)"""
        msg      = UInt8MultiArray()
        msg.data = list(frame)
        self.pub_pi2.publish(msg)

    def _send_scara(self, frame):
        """PC → 스카라 바이너리 전송 (식별자 0x01)"""
        msg      = UInt8MultiArray()
        msg.data = [0x01] + list(frame)
        self.pub_pi2.publish(msg)

    def _send_manip(self, frame):
        """PC → 매니퓰레이터 바이너리 전송 (식별자 0x02)"""
        msg      = UInt8MultiArray()
        msg.data = [0x02] + list(frame)
        self.pub_pi2.publish(msg)

    def _send_stm2(self, frame):
        """PC → STM2 바이너리 전송 (식별자 0x03)"""
        msg      = UInt8MultiArray()
        msg.data = [0x03] + list(frame)
        self.pub_pi2.publish(msg)

    def _send_binary_pi3(self, frame):
        """Pi3 → STM3 바이너리 전송 (식별자 없음 → STM3 fallback, ESTOP/RESET용)"""
        msg      = UInt8MultiArray()
        msg.data = list(frame)
        self.pub_pi3.publish(msg)

    def _send_stm3(self, frame):
        """PC → STM3 바이너리 전송 (식별자 0x01)"""
        msg      = UInt8MultiArray()
        msg.data = [0x01] + list(frame)
        self.pub_pi3.publish(msg)

    # ══════════════════════════════════════════
    # STM3 상태 수신 (Pi3 경유)
    # ══════════════════════════════════════════
    def _on_uart3(self, msg: String):
        """
        Pi3 → PC : STM3 상태/완료/에러 수신.
        형식: "STM3:PC:STATE:IDLE" / "STM3:PC:DONE:SENSE_DONE" 등
        현재는 로그만 기록. 필요 시 stm3_state 변수 추가하여 상태 관리 가능.
        """
        line = msg.data.strip()
        self.get_logger().info(f'[STM3] {line}')

        with self.state_lock:
            if line.startswith('STM3:PC:ERR:'):
                self.get_logger().error(f'STM3 에러: {line}')
            elif line.startswith('STM3:PC:RAW:'):
                self.get_logger().debug(f'STM3 raw: {line}')

    # ══════════════════════════════════════════
    # 센서값 수신 (Pi3 /pi3/sensor_data)
    # ══════════════════════════════════════════
    def _on_sensor_data(self, msg: String):
        """
        Pi3에서 오는 센서값 수신 → sensor_cache 갱신.
        형식: "TDS:512.0,PH:7.20"  또는  "WATER_TEMP:25.23"
              "TDS:512.0,PH:7.20,WATER_TEMP:25.23,AIR_TEMP:25.1,HUMIDITY:60.3"

        sensor_cache는 _publish_monitor에서 모니터 토픽에 포함됨.
        """
        line = msg.data.strip()
        if not line:
            return

        updated = {}
        try:
            for pair in line.split(','):
                pair = pair.strip()
                if ':' not in pair:
                    continue
                key, val_str = pair.split(':', 1)
                updated[key.strip()] = float(val_str.strip())
        except ValueError as e:
            self.get_logger().warn(f'센서 데이터 파싱 실패: {line} ({e})')
            return

        with self.state_lock:
            for k, v in updated.items():
                if k in self.sensor_cache:
                    self.sensor_cache[k] = v

        self.get_logger().debug(f'[Sensor] {updated}')

    # ══════════════════════════════════════════
    # 모니터 상태 publish
    # ══════════════════════════════════════════
    def _publish_monitor(self):
        with self.state_lock:
            status = (
                f'start_flag:{self.start_flag},'
                f'stm_state:{self.stm_state},'
                f'stm2_state:{self.stm2_state},'
                f'pi1_alive:{self.pi1_alive},'
                f'pi2_alive:{self.pi2_alive},'
                f'pi3_alive:{self.pi3_alive},'
                f'emergency:{self.emergency},'
                f'cam1_connected:{self.cam1_connected},'
                f'cam1_status:{self.cam1_status},'
                f'cam1_detect_label:{self.cam1_detect_label},'
                f'cam1_last_conf:{self.cam1_last_conf:.2f},'
                f'nursery_left_label:{self.nursery_state["left"]["last_label"]},'
                f'nursery_left_conf:{self.nursery_state["left"]["last_conf"]:.2f},'
                f'nursery_left_stable:{self.nursery_state["left"]["stable_cnt"]},'
                f'nursery_right_label:{self.nursery_state["right"]["last_label"]},'
                f'nursery_right_conf:{self.nursery_state["right"]["last_conf"]:.2f},'
                f'nursery_right_stable:{self.nursery_state["right"]["stable_cnt"]}'
            )
            flags_str = ','.join(
                f'{k}:{v}' for k, v in self.flags.items()
            )
            # 수신된 센서값만 포함 (None은 제외)
            sensor_str = ','.join(
                f'sensor_{k.lower()}:{v:.2f}'
                for k, v in self.sensor_cache.items()
                if v is not None
            )
        msg = String()
        parts = [status, flags_str]
        if sensor_str:
            parts.append(sensor_str)
        msg.data = ','.join(parts)
        self.pub_monitor.publish(msg)


# ══════════════════════════════════════════════
# YOLO 프레임 처리
# ══════════════════════════════════════════════
def process_frame(node: MasterNode, frame: np.ndarray):
    results = node.model(
        frame,
        imgsz=960,
        conf=0.25,
        verbose=False
    )[0]
    disp = frame.copy()
    best = None
    for box in results.boxes:
        cls_id = int(box.cls[0].item())
        conf   = float(box.conf[0].item())
        if cls_id == TRAY_CLASS_ID:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            if best is None or conf > best[4]:
                best = (x1, y1, x2, y2, conf)

    disp     = frame.copy()
    send_now = False
    now      = time.time()

    if best is None:
        with node.state_lock:
            node.stable_cnt   = 0
            node.no_tray_cnt += 1
            if node.no_tray_cnt >= 3:
                node.cam1_detect_label = 'none'
                node.cam1_last_conf    = 0.0
    else:
        x1, y1, x2, y2, conf = best

        if conf < MIN_CONF:
            with node.state_lock:
                node.stable_cnt        = 0
                node.no_tray_cnt       = 0
                node.cam1_detect_label = 'none'
                node.cam1_last_conf    = conf
        else:
            h, w = frame.shape[:2]
            cx = (x1 + x2) / 2 / w
            cy = (y1 + y2) / 2 / h
            box_w = x2 - x1
            box_h = y2 - y1

            # ROI + 박스 크기 조건
            in_roi = (
                ROI_X_MIN < cx < ROI_X_MAX
                and ROI_Y_MIN < cy < ROI_Y_MAX
                and box_w / w > MIN_BOX_RATIO
                and box_h / h > MIN_BOX_RATIO
            )

            with node.state_lock:
                if in_roi:
                    node.cam1_last_detect_ts = now
                    node.cam1_last_conf      = conf
                    node.cam1_detect_label   = 'tray_detected'
                    node.stable_cnt         += 1
                    node.no_tray_cnt         = 0
                    node.cam1_last_bbox      = (x1, y1, x2, y2)
                    node.cam1_last_bbox_ts   = now
                    node.cam1_last_conf_hold = conf

                    # 조건 충족 시 SSF=1 전송
                    if (
                        node.stable_cnt  >= STABLE_FRAMES
                        and not node.start_flag
                        and node.stm_state == 'idle'
                        and (now - node.last_tx) > COOLDOWN_SEC
                        and not node.emergency
                        and node.pi1_alive
                    ):
                        node.start_flag   = True
                        node.last_tx      = now
                        node.stable_cnt   = 0
                        node._set_flag('c1f', 1)
                        send_now = True
                else:
                    node.stable_cnt   = 0
                    node.no_tray_cnt += 1

    if send_now:
        msg      = UInt8MultiArray()
        msg.data = make_flag_u8(PID_C1F, 1)
        node.pub_pi1.publish(msg)
        node.get_logger().info('트레이 감지 → C1F=1 전송')

    # ── 화면 오버레이 ─────────────────────────
    with node.state_lock:
        start_flag     = node.start_flag
        stm_state      = node.stm_state
        stable_cnt     = node.stable_cnt
        last_bbox      = node.cam1_last_bbox
        last_bbox_ts   = node.cam1_last_bbox_ts
        hold_sec       = node.cam1_overlay_hold_sec
        last_conf_hold = node.cam1_last_conf_hold
        emergency      = node.emergency
        pi1_ok         = node.pi1_alive

    h, w = disp.shape[:2]

    cv2.rectangle(disp,
        (int(w * ROI_X_MIN), int(h * ROI_Y_MIN)),
        (int(w * ROI_X_MAX), int(h * ROI_Y_MAX)),
        (255, 255, 0), 2)

    cv2.putText(disp, f'start_flag={int(start_flag)}',
        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,255), 2)
    cv2.putText(disp, f'stm_state={stm_state}',
        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)
    cv2.putText(disp, f'stable={stable_cnt}  pi1={"OK" if pi1_ok else "NG"}',
        (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

    if last_bbox is not None and (now - last_bbox_ts) < hold_sec:
        bx1, by1, bx2, by2 = last_bbox
        cv2.rectangle(disp, (bx1,by1), (bx2,by2), (0,255,0), 2)
        cv2.putText(disp, f'tray {last_conf_hold:.2f}',
            (bx1, by1-8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    else:
        cv2.putText(disp, 'tray: none',
            (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

    if emergency:
        cv2.putText(disp, 'EMERGENCY',
            (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)

    try:
        frame_queue.put_nowait(disp)
    except queue.Full:
        pass

def process_nursery_frame(node: MasterNode, frame: np.ndarray, position: str):
    results = node.nursery_model.predict(
        source=frame,
        imgsz=640,  
        conf=NURSERY_MIN_CONF,
        device='cpu',
        verbose=False
    )[0]
    disp = frame.copy()
    h, w = frame.shape[:2]

    cv2.rectangle(
        disp,
        (int(w * NURSERY_ROI_X_MIN), int(h * NURSERY_ROI_Y_MIN)),
        (int(w * NURSERY_ROI_X_MAX), int(h * NURSERY_ROI_Y_MAX)),
        (255, 255, 0),
        2
    )
    node.get_logger().info(f'[Nursery {position}] boxes={len(results.boxes)}')

    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cls_id = int(box.cls[0].item())
        conf = float(box.conf[0].item())
        class_name = node.nursery_model.names[cls_id]

        cx = (x1 + x2) / 2 / w
        cy = (y1 + y2) / 2 / h
        box_w = x2 - x1
        box_h = y2 - y1

        if class_name == 'tray':
            min_box_ratio = NURSERY_TRAY_MIN_BOX_RATIO
        else:
            min_box_ratio = NURSERY_SPROUT_MIN_BOX_RATIO

        in_roi = (
            NURSERY_ROI_X_MIN < cx < NURSERY_ROI_X_MAX
            and NURSERY_ROI_Y_MIN < cy < NURSERY_ROI_Y_MAX
            and box_w / w > min_box_ratio
            and box_h / h > min_box_ratio
        )

        color = (0, 255, 0) if in_roi else (0, 0, 255)

        node.get_logger().info(
            f'[Nursery {position}] DETECT class={class_name}, conf={conf:.2f}, in_roi={in_roi}'
        )

        cv2.rectangle(disp, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            disp,
            f'{class_name} {conf:.2f}',
            (x1, max(20, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2
        )

    counts = {
        'tray': 0,
        'sprout1': 0,
        'sprout2': 0,
        'sprout3': 0,
    }

    best_sprout3_conf = 0.0
    new_uv = None
    
    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cls_id = int(box.cls[0].item())
        conf = float(box.conf[0].item())
        class_name = node.nursery_model.names[cls_id]

        if class_name not in counts:
            continue

        cx = (x1 + x2) / 2 / w
        cy = (y1 + y2) / 2 / h
        box_w = x2 - x1
        box_h = y2 - y1

        if class_name == 'tray':
            min_box_ratio = NURSERY_TRAY_MIN_BOX_RATIO
        else:
            min_box_ratio = NURSERY_SPROUT_MIN_BOX_RATIO

        in_roi = (
            NURSERY_ROI_X_MIN < cx < NURSERY_ROI_X_MAX
            and NURSERY_ROI_Y_MIN < cy < NURSERY_ROI_Y_MAX
            and box_w / w > min_box_ratio
            and box_h / h > min_box_ratio
        )

        if not in_roi:
            continue

        if class_name == 'tray':
            counts['tray'] = 1
        else:
            counts[class_name] += 1


        if class_name == 'sprout3':
            best_sprout3_conf = max(best_sprout3_conf, conf)

    tray_detected = counts['tray'] > 0
    sprout3_detected = counts['sprout3'] > 0

    now = time.time()
    send_event = False
    flag_name = None

    with node.state_lock:
        st = node.nursery_state[position]

        # ── 트레이 점유 판단 ─────────────────────
        if tray_detected:
            st['tray_seen_cnt'] += 1
            st['tray_lost_cnt'] = 0
        else:
            st['tray_lost_cnt'] += 1
            st['tray_seen_cnt'] = 0

        if st['tray_seen_cnt'] >= TRAY_OCCUPY_FRAMES:
            st['occupied'] = 1

        if st['tray_lost_cnt'] >= TRAY_LOST_FRAMES:
            st['occupied'] = 0
            if position == 'left':
                node._set_flag('ulf', 0)
            else:
                node._set_flag('urf', 0)

        # ── 발아 완료 판단 ─────────────────────
        if tray_detected and sprout3_detected:
            st['stable_cnt'] += 1
            st['last_label'] = 'sprout3'
            st['last_conf'] = best_sprout3_conf
        else:
            st['stable_cnt'] = 0
            st['last_label'] = 'tray_only' if tray_detected else 'none'
            st['last_conf'] = 0.0

        if (
            st['stable_cnt'] >= NURSERY_STABLE_FRAMES
            and now - st['last_tx'] > NURSERY_COOLDOWN_SEC
        ):
            st['last_tx'] = now
            st['stable_cnt'] = 0

            if position == 'left':
                if node.flags['ulf'] == 0:
                    node._set_flag('ulf', 1)
                    flag_name = 'ULF'
                    send_event = True
            else:
                if node.flags['urf'] == 0:
                    node._set_flag('urf', 1)
                    flag_name = 'URF'
                    send_event = True

        # ── 좌/우 점유 상태로 uv 재계산 ─────────
        new_uv = (
            int(node.nursery_state['left']['occupied'])
            + int(node.nursery_state['right']['occupied'])
        )

        if node.flags['uv'] != new_uv:
            node._set_flag('uv', new_uv)
            send_event = True

        stable = st['stable_cnt']
        label = st['last_label']

        if send_event:
            # node._broadcast_all_flags()  # 통합 테스트 전까지 잠시 비활성화
            if flag_name is not None:
                node.get_logger().info(
                    f'[TEST] 발아 완료 감지 → {flag_name}=1 예정, conf={best_sprout3_conf:.2f}'
                )
            else:
                node.get_logger().info(
                    f'[TEST] UV 점유 상태 변경 → uv={new_uv} 예정'
                )

    node.get_logger().info(
        f'[Nursery {position}] tray={counts["tray"]}, sprout3={counts["sprout3"]}, '
        f'stable={stable}, label={label}'
    )

    y = 30
    for name, count in counts.items():
        cv2.putText(
            disp,
            f'{name}: {count}',
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )
        y += 30

    cv2.putText(
        disp,
        f'nursery stable: {stable}/{NURSERY_STABLE_FRAMES}',
        (20, y + 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2
    )

    cv2.putText(
        disp,
        f'label: {label}',
        (20, y + 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2
    )

    #target_queue = nursery_left_frame_queue if position == 'left' else nursery_right_frame_queue

    #try:
        #target_queue.put_nowait(disp)
    #except queue.Full:
        #pass

# ══════════════════════════════════════════════
# 카메라 스트림 수신
# ══════════════════════════════════════════════
def video_receive_loop(node: MasterNode):
    calib      = np.load(CALIB_PATH)
    K          = calib['camera_matrix']
    dist       = calib['dist_coeffs']
    map1, map2 = None, None

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', STREAM_PORT))
    server.listen(1)
    print(f'[Stream] Pi1(cam1) 연결 대기 (port={STREAM_PORT})')

    while rclpy.ok():
        conn = None
        try:
            conn, addr = server.accept()
            print(f'[Stream] Pi1(cam1) 연결됨: {addr}')

            with node.state_lock:
                node.cam1_connected = True

            data         = b''
            payload_size = struct.calcsize('>I')

            while rclpy.ok():
                while len(data) < payload_size:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionError('stream disconnected')
                    data += packet

                msg_size = struct.unpack('>I', data[:payload_size])[0]
                data     = data[payload_size:]

                while len(data) < msg_size:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionError('stream disconnected')
                    data += packet

                frame_data = data[:msg_size]
                data       = data[msg_size:]

                frame = cv2.imdecode(
                    np.frombuffer(frame_data, dtype=np.uint8),
                    cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                if map1 is None:
                    h, w = frame.shape[:2]
                    newK, _ = cv2.getOptimalNewCameraMatrix(
                        K, dist, (w,h), 0, (w,h))
                    map1, map2 = cv2.initUndistortRectifyMap(
                        K, dist, None, newK, (w,h), cv2.CV_16SC2)
                    print('[Calib] 왜곡 보정 맵 초기화 완료')

                frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

                with node.state_lock:
                    node.cam1_last_frame_ts = time.time()

                process_frame(node, frame)

        except Exception as e:
            print(f'[Stream] 오류: {e}')
        finally:
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass
            with node.state_lock:
                node.cam1_connected = False

def nursery_video_receive_loop(node: MasterNode, stream_port: int, position: str, calib_path: str | None = None):
    K = None
    dist = None
    map1, map2 = None, None

    if calib_path is not None:
        try:
            calib = np.load(calib_path)
            K = calib['camera_matrix']
            dist = calib['dist_coeffs']
            print(f'[Nursery Calib {position}] 로드 완료: {calib_path}')
        except Exception as e:
            print(f'[Nursery Calib {position}] 로드 실패: {e}')
            K = None
            dist = None

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', stream_port))
    server.listen(1)
    print(f'[Nursery Stream {position}] 연결 대기 (port={stream_port})')
    
    while rclpy.ok():
        conn = None
        try:
            conn, addr = server.accept()
            print(f'[Nursery Stream {position}] 연결됨: {addr}')

            data = b''
            payload_size = struct.calcsize('>I')
            last_process_ts = 0.0
            PROCESS_INTERVAL = 0.5   # 약 2fps

            while rclpy.ok():
                while len(data) < payload_size:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionError('nursery stream disconnected')
                    data += packet

                msg_size = struct.unpack('>I', data[:payload_size])[0]
                data = data[payload_size:]

                while len(data) < msg_size:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionError('nursery stream disconnected')
                    data += packet

                frame_data = data[:msg_size]
                data = data[msg_size:]

                frame = cv2.imdecode(
                    np.frombuffer(frame_data, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )

                if frame is None:
                    continue

                if K is not None and dist is not None:
                    if map1 is None:
                        h, w = frame.shape[:2]
                        newK, _ = cv2.getOptimalNewCameraMatrix(
                            K, dist, (w, h), 0, (w, h)
                        )
                        map1, map2 = cv2.initUndistortRectifyMap(
                            K, dist, None, newK, (w, h), cv2.CV_16SC2
                        )
                        print(f'[Nursery Calib {position}] 왜곡 보정 맵 초기화 완료')

                    frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
                now = time.time()
                if now - last_process_ts >= PROCESS_INTERVAL:
                    last_process_ts = now
                    process_nursery_frame(node, frame, position)
                

        except Exception as e:
            print(f'[Nursery Stream {position}] 오류: {e}')
        finally:
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass

def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()

    threading.Thread(
        target=video_receive_loop,
        args=(node,), daemon=True).start()
    
    threading.Thread(
        target=nursery_video_receive_loop,
        args=(node, NURSERY_LEFT_STREAM_PORT, 'left', NURSERY_LEFT_CALIB_PATH),
        daemon=True
    ).start()

    threading.Thread(
        target=nursery_video_receive_loop,
        args=(node, NURSERY_RIGHT_STREAM_PORT, 'right', NURSERY_RIGHT_CALIB_PATH),
        daemon=True
    ).start()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            try:
                frame = frame_queue.get_nowait()
                #cv2.imshow('Pi1 Camera', frame)               
            except queue.Empty:
                pass
            try:
                frame = nursery_left_frame_queue.get_nowait()
                #cv2.imshow('Pi1 Nursery Left Camera', frame)
            except queue.Empty:
                pass
            try:
                frame = nursery_right_frame_queue.get_nowait()
                #cv2.imshow('Pi1 Nursery Right Camera', frame)
            except queue.Empty:
                pass
            cv2.waitKey(1) 
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
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