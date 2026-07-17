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
import torch
from ultralytics import YOLO
#import sys
#from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

frame_queue = queue.Queue(maxsize=2)
nursery_left_frame_queue = queue.Queue(maxsize=2)
nursery_right_frame_queue = queue.Queue(maxsize=2)
water_left_frame_queue = queue.Queue(maxsize=2)
water_right_frame_queue = queue.Queue(maxsize=2)
def put_latest(q, item):
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(item)
        except queue.Full:
            pass

# CAM_MODULE_CANDIDATES = [
#     Path('/home/thumb/aquaponic_copy/Automated-Aquaponics-System/first_convey/cam'),
#     Path('/home/thumb/aquaponics_copy/Automated-Aquaponics-System/first_convey/cam'),
# ]

# for cam_module_dir in CAM_MODULE_CANDIDATES:
#     if cam_module_dir.exists():
#         sys.path.append(str(cam_module_dir))
#         break

# try:
#     from op_sprout import detect_sprouts
# except Exception as e:
#     detect_sprouts = None
#     print(f'[Sprout] op_sprout import failed: {e}')

# ── 설정 ──────────────────────────────────────
# 컨베이어 트레이인식 카메라
MODEL_PATH    = '/home/thumb/aquaponic_copy/tray2/best.pt'
STREAM_PORT   = 5000
TRAY_CLASS_ID = 0
MIN_CONF      = 0.35
STABLE_FRAMES = 3
COOLDOWN_SEC  = 2.0

# 컨베이어 ROI (트레이 감지 유효 영역)
ROI_X_MIN = 0.25
ROI_X_MAX = 0.80
ROI_Y_MIN = 0.05
ROI_Y_MAX = 0.95
# 발아실 ROI
NURSERY_ROI_X_MIN = 0.13
NURSERY_ROI_X_MAX = 0.90
NURSERY_ROI_Y_MIN = 0.17
NURSERY_ROI_Y_MAX = 0.74
NURSERY_TRAY_MIN_BOX_RATIO = 0.5   # 트레이 박스가 화면 대비 최소 50% 이상이어야 감지

# 박스 최소 크기 (화면 대비 비율)
MIN_BOX_RATIO = 0.25

# 캘리브레이션 파일
CALIB_PATH = '/home/thumb/aquaponic_copy/smartfarm_ws/camera_calib.npz'
NURSERY_LEFT_CALIB_PATH = '/home/thumb/aquaponic_copy/smartfarm_ws/uv_left_calib.npz'
NURSERY_RIGHT_CALIB_PATH = '/home/thumb/aquaponic_copy/smartfarm_ws/uv_right_calib.npz'
WATER_LEFT_CALIB_PATH = '/home/thumb/aquaponic_copy/smartfarm_ws/camera_calib.npz'
WATER_RIGHT_CALIB_PATH = '/home/thumb/aquaponic_copy/smartfarm_ws/camera_calib.npz'

# 발아실 카메라 / 모델 (pi1)
NURSERY_TRAY_MODEL_PATH = '/home/thumb/aquaponic_copy/tray2/sprout_tray_best.pt'

NURSERY_LEFT_STREAM_PORT = 5001
NURSERY_RIGHT_STREAM_PORT = 5002

#NURSERY_MIN_CONF = 0.2     # YOLO conf 최소값, 이게 트레이일 확률이 20%이상이어야 박스를 그림
NURSERY_TRAY_CONF = 0.35

# 초록색 새싹 (hsv)
NURSERY_LOWER_GREEN = np.array([35, 60, 70], dtype=np.uint8)
NURSERY_UPPER_GREEN = np.array([90, 255, 255], dtype=np.uint8)

# 노란색·황록색 새싹
NURSERY_LOWER_YELLOW = np.array([20, 55, 110], dtype=np.uint8)
NURSERY_UPPER_YELLOW = np.array([42, 220, 255], dtype=np.uint8)

# 새싹으로 인정할 최소/최대 면적
NURSERY_MIN_SPROUT_AREA = 15
NURSERY_MAX_SPROUT_AREA = 20000

# 작은 노이즈 제거용
NURSERY_OPEN_KERNEL_SIZE = 3

# 한 새싹의 떨어진 잎을 연결하기 위한 크기
NURSERY_CLOSE_KERNEL_SIZE = 3

# 너무 작고 가느다란 영역 제거
NURSERY_MIN_SPROUT_WIDTH = 6
NURSERY_MIN_SPROUT_HEIGHT = 6

# 발아실 ROI 내부에서 가운데 흰색 영역 제외
# ROI 내부 너비를 0~1로 봤을 때의 비율
NURSERY_CENTER_EXCLUDE_X_MIN = 0.36
NURSERY_CENTER_EXCLUDE_X_MAX = 0.67

NURSERY_STABLE_FRAMES = 3
NURSERY_COOLDOWN_SEC = 3.0
NURSERY_SPROUT_DONE_COUNT = 25  # 새싹 후보가 몇 개 이상이면 발아 완료로 볼지 정하는 값. 
NURSERY_SEND_FLAGS = True  

TRAY_OCCUPY_FRAMES = 2
TRAY_LOST_FRAMES = 7

# 수경재배실 카메라(pi3)
WATER_LEFT_STREAM_PORT = 5011
WATER_RIGHT_STREAM_PORT = 5012

#수경재배실 opencv기준값
WATER_LOWER_GREEN = np.array([35, 45, 40], dtype=np.uint8)
WATER_UPPER_GREEN = np.array([90, 255, 255], dtype=np.uint8)

WATER_ROI_X_MIN = 0.10
WATER_ROI_X_MAX = 0.90
WATER_ROI_Y_MIN = 0.10
WATER_ROI_Y_MAX = 0.90

WATER_GROWTH_AREA_RATIO = 0.18
WATER_MIN_LEAF_AREA = 2500
WATER_STABLE_FRAMES = 5
WATER_LOST_FRAMES = 8
WATER_COOLDOWN_SEC = 3.0

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
PID_HMF   = 0x12

#로봇 작업 PID, 작업 종류, 슬롯 코드
PID_ROBOT_JOB = 0x13

JOB_C1_TO_NURSERY = 0x01
JOB_NURSERY_TO_WATER = 0x02
JOB_WATER_TO_VIB_AND_C2 = 0x03
JOB_HOME = 0x04
JOB_HARVEST = 0x05

SLOT_NONE = 0x00
SLOT_LEFT = 0x01
SLOT_RIGHT = 0x02

SLOT_CODE = {
    None: SLOT_NONE,
    'left': SLOT_LEFT,
    'right': SLOT_RIGHT,
}

SLOT_UNKNOWN = 'unknown'
SLOT_EMPTY = 'empty'
SLOT_RESERVED_IN = 'reserved_in'
SLOT_OCCUPIED = 'occupied'
SLOT_GROWING = 'growing'
SLOT_READY = 'ready'
SLOT_RESERVED_OUT = 'reserved_out'

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
        
        self.state_lock = threading.RLock()
        self.yolo_lock = threading.Lock()

        self.yolo_device = 0 if torch.cuda.is_available() else 'cpu'
        self.create_timer(0.1, self._schedule_scara_jobs)

        print('[Torch check]')
        print('torch cuda available:', torch.cuda.is_available())
        print('torch cuda count:', torch.cuda.device_count())
        print('torch version:', torch.__version__)
        print('YOLO device:', self.yolo_device)

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
            'hmf':  0,
        }

        self.nursery_slots = {
            'left': {
                'state': SLOT_UNKNOWN,
                'job_id': None,
            },
            'right': {
                'state': SLOT_UNKNOWN,
                'job_id': None,
            },
        }

        self.water_slots = {
            'left': {
                'state': SLOT_UNKNOWN,
                'job_id': None,
            },
            'right': {
                'state': SLOT_UNKNOWN,
                'job_id': None,
            },
        }
        # 첫 컨베이어 파종 트레이가 들어갈 예약 위치
        self.seed_target_slot = None
        # 스카라 작업 관리
        self.pending_scara_jobs = []
        self.active_scara_job = None
        self.next_scara_job_id = 1
        # 매니퓰레이터 작업 관리
        self.active_manip_job = None
        self.next_manip_job_id = 1
        # 발표 데모 진행 상태
        self.demo_mode = True
        self.demo_phase = 'WAIT_FIRST_TRAY'
        
        # ── STM1 상태 ─────────────────────────
        self.start_flag      = False
        self.stm_state       = 'idle'
        self.scara_prehome_sent = False   # 이번 사이클에서 스카라 사전 홈잉을 보냈는지
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
        #수경재배실 카메라
        self.water_state = {
            'left': {
                'stable_cnt': 0,
                'lost_cnt': 0,
                'last_tx': 0.0,
                'last_label': 'none',
                'last_score': 0.0,
            },
            'right': {
                'stable_cnt': 0,
                'lost_cnt': 0,
                'last_tx': 0.0,
                'last_label': 'none',
                'last_score': 0.0,
            },
        }

        # ── 공통 ──────────────────────────────
        self.emergency = False
        self.device_estop = {
            'stm1': False,
            'scara': False,
        }

        # ── YOLO 모델 ─────────────────────────
        print('[YOLO] 모델 로딩 중...')
        self.model = YOLO(MODEL_PATH)
        print('[YOLO] 로딩 완료')

        print('[YOLO] 발아실 트레이 모델 로딩 중...')
        self.nursery_tray_model = YOLO(NURSERY_TRAY_MODEL_PATH)
        print('[YOLO] 발아실 트레이 모델 로딩 완료')
        print('[Nursery tray model names]', self.nursery_tray_model.names)

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
    #호환 플래그를 다시 계산하는 함수
    def _sync_slot_flags_locked(self):
        occupied_states = {
            SLOT_RESERVED_IN,
            SLOT_OCCUPIED,
            SLOT_GROWING,
            SLOT_READY,
            SLOT_RESERVED_OUT,
        }

        self.flags['uv'] = sum(
            1
            for slot in self.nursery_slots.values()
            if slot['state'] in occupied_states
        )

        self.flags['wcnt'] = sum(
            1
            for slot in self.water_slots.values()
            if slot['state'] in occupied_states
        )

        self.flags['ulf'] = int(
            self.nursery_slots['left']['state'] == SLOT_READY
        )
        self.flags['urf'] = int(
            self.nursery_slots['right']['state'] == SLOT_READY
        )

        self.flags['wlf'] = int(
            self.water_slots['left']['state'] == SLOT_READY
        )
        self.flags['wrf'] = int(
            self.water_slots['right']['state'] == SLOT_READY
        )
    # 빈 슬롯 선택 함수 
    def _find_empty_slot_locked(self, room_name: str):
        slots = (
            self.nursery_slots
            if room_name == 'nursery'
            else self.water_slots
        )

        if slots['left']['state'] == SLOT_EMPTY:
            return 'left'

        if slots['right']['state'] == SLOT_EMPTY:
            return 'right'

        return None
    #작업 프레임 생성 함수
    def _make_scara_job_frame(self, job):
        data = bytes([
            job['type'],
            SLOT_CODE[job['source']],
            SLOT_CODE[job['destination']],
            job['job_id'],
        ])

        return make_frame(PID_ROBOT_JOB, data)
    # 스케줄러 함수
    def _schedule_scara_jobs(self):
        frame_to_send = None

        with self.state_lock:
            if self.emergency:
                return

            if not self.pi2_alive:
                return

            if self.active_scara_job is not None:
                return

            if not self.pending_scara_jobs:
                self._create_automatic_transfer_job_locked()

            if not self.pending_scara_jobs:
                return

            job = self.pending_scara_jobs.pop(0)

            self.active_scara_job = job
            self.flags['smf'] = 1

            if job['type'] == JOB_C1_TO_NURSERY:
                self.flags['ssf'] = 1
                self.flags['crf'] = 1

            job['sent_at'] = time.time()
            frame_to_send = self._make_scara_job_frame(job)

        if frame_to_send is not None:
            self._send_scara(frame_to_send)

            if job['type'] == JOB_C1_TO_NURSERY:
                self._send_binary(
                    make_flag_u8(PID_CRF, 1)
                )

            self.get_logger().info(
                f'SCARA 작업 전송: {job}'
            )
    # 자동 작업 선택 함수
    def _create_automatic_transfer_job_locked(self):
        # 1. 수경실 READY 트레이 수확 경로
        water_source = self._find_ready_slot_locked('water')

        if (
            water_source is not None
            and self.stm2_state == 'idle'
        ):
            self._queue_water_to_c2_locked(water_source)
            return

        # 2. 발아실 READY 트레이를 수경실로 이동
        nursery_source = self._find_ready_slot_locked('nursery')
        water_destination = self._find_empty_slot_locked('water')

        if (
            nursery_source is not None
            and water_destination is not None
        ):
            self._queue_nursery_to_water_locked(
                nursery_source,
                water_destination,
            )
    def _find_ready_slot_locked(self, room_name):
        slots = (
            self.nursery_slots
            if room_name == 'nursery'
            else self.water_slots
        )

        if slots['left']['state'] == SLOT_READY:
            return 'left'

        if slots['right']['state'] == SLOT_READY:
            return 'right'

        return None
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
        self._send_scara(make_flag_u8(PID_HMF, self.flags['hmf']))

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
    # 작업 ID 생성 함수
    def _next_scara_job_id_locked(self):
        job_id = self.next_scara_job_id

        self.next_scara_job_id += 1
        if self.next_scara_job_id > 255:
            self.next_scara_job_id = 1

        return job_id
    def _on_uart1(self, msg: String):
        line = msg.data.strip()
        self.get_logger().info(f'[STM1] {line}')

        with self.state_lock:
            if line == 'STM1:PC:STATE:HOMING':
                self.stm_state = 'homing'
                self.scara_prehome_sent = False

            elif line == 'STM1:PC:STATE:RUN_CONVEYOR1':
                self.stm_state = 'running'
            elif line == 'STM1:PC:STATE:SEEDING':
                self.stm_state = 'seeding'
                    # 직교로봇이 실제 파종 단계에 들어갔을 때 스카라 사전 홈잉 시작
                if (
                    not self.scara_prehome_sent
                    and self.flags['smf'] == 0
                    and self.flags['uv'] < 2
                    and self.flags['hmf'] == 0
                    and not self.emergency
                    and self.pi2_alive
                ):
                    self.scara_prehome_sent = True
                    self._set_flag('hmf', 1)
                    self._send_scara(make_flag_u8(PID_HMF, 1))
                    self.get_logger().info('STM1 파종 시작 → 스카라 사전 홈잉 HMF=1 전송')
                else:
                    self.get_logger().warn(
                        f'스카라 사전 홈잉 생략: '
                        f'sent={self.scara_prehome_sent}, '
                        f'smf={self.flags["smf"]}, '
                        f'uv={self.flags["uv"]}, '
                        f'hmf={self.flags["hmf"]}, '
                        f'pi2_alive={self.pi2_alive}, '
                        f'emergency={self.emergency}'
                    )
            elif line == 'STM1:PC:STATE:EJECTING':
                self.stm_state = 'ejecting'
            elif line == 'STM1:PC:STATE:WAIT_SCARA_PICK':
                self.stm_state = 'waiting_scara'
            elif line == 'STM1:PC:STATE:ESTOP':
                self.stm_state  = 'error'
                self.start_flag = False
                self.get_logger().warn('[STM1] ESTOP')
            elif line == 'STM1:PC:DONE:CYCLE1':
                target_slot = self.seed_target_slot

                if target_slot is None:
                    self.get_logger().error(
                        '파종 완료됐지만 예약된 발아실 슬롯이 없음'
                    )
                    self.stm_state = 'error'
                    return

                job_id = self._next_scara_job_id_locked()

                job = {
                    'job_id': job_id,
                    'type': JOB_C1_TO_NURSERY,
                    'source': None,
                    'destination': target_slot,
                    'sent_at': 0.0,
                    'retry_count': 0,
                }

                self.nursery_slots[target_slot]['job_id'] = job_id
                self.pending_scara_jobs.append(job)

                self.get_logger().info(
                    f'C1_TO_NURSERY 작업 대기: '
                    f'job_id={job_id}, destination={target_slot}'
                )
            elif line == 'STM1:PC:STATE:IDLE':
                self.stm_state  = 'idle'
                self.start_flag = False
                self.scara_prehome_sent = False
                self._set_flag('c1f', 0)
                self._set_flag('hmf', 0) 
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
            if line == 'SCARA:PC:EVENT:HOME_DONE':
                self._set_flag('hmf', 0)
                self.get_logger().info('스카라 사전 홈잉 완료 → HMF=0')
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

        # ─────────────────────────────
        # 전체 긴급정지: STM1 + SCARA
        # ─────────────────────────────
        if cmd in ('EMERGENCY', 'EMERGENCY_ALL'):
            with self.state_lock:
                self.emergency = True
                self.start_flag = False
                self.stm_state = 'error'
                self.scara_prehome_sent = False

                # STM1/SCARA 관련 플래그 정리
                self.flags['c1f'] = 0
                self.flags['ssf'] = 0
                self.flags['smf'] = 0
                self.flags['crf'] = 0
                self.flags['hmf'] = 0

                self.device_estop['stm1'] = True
                self.device_estop['scara'] = True

            self._send_estop_stm1_scara()
            self.get_logger().warn('EMERGENCY_ALL: STM1 + SCARA 정지')
            return

        # ─────────────────────────────
        # 전체 리셋: STM1 + SCARA
        # ─────────────────────────────
        elif cmd in ('RESET', 'RESET_ALL'):
            with self.state_lock:
                self.emergency = False
                self.start_flag = False
                self.stm_state = 'idle'
                self.scara_prehome_sent = False
                self.stable_cnt = 0
                self.no_tray_cnt = 0
                self.cam1_detect_label = 'none'
                self.cam1_last_conf = 0.0

                # 우선 STM1/SCARA 관련 플래그만 초기화
                self.flags['c1f'] = 0
                self.flags['ssf'] = 0
                self.flags['smf'] = 0
                self.flags['crf'] = 0
                self.flags['hmf'] = 0

                self.device_estop['stm1'] = False
                self.device_estop['scara'] = False


            self._send_reset_stm1_scara()
            self.get_logger().info('RESET_ALL: STM1 + SCARA 리셋')
            return

        # ─────────────────────────────
        # 개별 STM1 긴급정지 / 리셋
        # ─────────────────────────────
        elif cmd == 'EMERGENCY_STM1':
            with self.state_lock:
                self.start_flag = False
                self.stm_state = 'error'
                self.flags['c1f'] = 0
                self.flags['crf'] = 0
                self.device_estop['stm1'] = True

            self._send_estop_stm1()
            self.get_logger().warn('EMERGENCY_STM1')
            return

        elif cmd == 'RESET_STM1':
            with self.state_lock:
                self.start_flag = False
                self.stm_state = 'idle'
                self.flags['c1f'] = 0
                self.flags['crf'] = 0
                self.device_estop['stm1'] = False

            self._send_reset_stm1()
            self.get_logger().info('RESET_STM1')
            return

        # ─────────────────────────────
        # 개별 SCARA 긴급정지 / 리셋
        # ─────────────────────────────
        elif cmd == 'EMERGENCY_SCARA':
            with self.state_lock:
                self.flags['ssf'] = 0
                self.flags['smf'] = 0
                self.flags['crf'] = 0
                self.flags['hmf'] = 0
                self.device_estop['scara'] = True

            self._send_estop_scara()
            self.get_logger().warn('EMERGENCY_SCARA')
            return

        elif cmd == 'RESET_SCARA':
            with self.state_lock:
                self.flags['ssf'] = 0
                self.flags['smf'] = 0
                self.flags['crf'] = 0
                self.flags['hmf'] = 0
                self.device_estop['scara'] = False

            self._send_reset_scara()
            self.get_logger().info('RESET_SCARA')
            return
        # 데모용 초기화 명령
        elif cmd == 'DEMO_RESET':
            with self.state_lock:
                if self.active_scara_job is not None:
                    self.get_logger().warn(
                        'SCARA 작업 중에는 DEMO_RESET 불가'
                    )
                    return

                self.pending_scara_jobs.clear()
                self.active_scara_job = None
                self.active_manip_job = None
                self.seed_target_slot = None

                for slot in self.nursery_slots.values():
                    slot['state'] = SLOT_EMPTY
                    slot['job_id'] = None

                for slot in self.water_slots.values():
                    slot['state'] = SLOT_EMPTY
                    slot['job_id'] = None

                self.demo_phase = 'WAIT_FIRST_TRAY'

                self.flags['ssf'] = 0
                self.flags['smf'] = 0
                self.flags['crf'] = 0
                self.flags['ff'] = 0
                self.flags['hf'] = 0

                self._sync_slot_flags_locked()

            self.get_logger().info(
                'DEMO_RESET 완료: 실제 설비도 비어 있는지 확인 필요'
            )
            return

        else:
            self.get_logger().warn(f'알 수 없는 명령: {cmd}')
    
    # 긴급정지 리셋
    def _send_estop_stm1(self):
        self._send_binary(make_estop())

    def _send_reset_stm1(self):
        self._send_binary(make_reset())

    def _send_estop_scara(self):
        self._send_scara(make_estop())

    def _send_reset_scara(self):
        self._send_scara(make_reset())

    def _send_estop_stm1_scara(self):
        self._send_estop_stm1()
        self._send_estop_scara()

    def _send_reset_stm1_scara(self):
        self._send_reset_stm1()
        self._send_reset_scara()
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
            active_scara_job_id = (
                self.active_scara_job['job_id']
                if self.active_scara_job is not None
                else 0
            )
            active_manip_job_id = (
                self.active_manip_job['job_id']
                if self.active_manip_job is not None
                else 0
            )
            status = (
                f'start_flag:{self.start_flag},'
                f'stm_state:{self.stm_state},'
                f'stm2_state:{self.stm2_state},'
                f'pi1_alive:{self.pi1_alive},'
                f'pi2_alive:{self.pi2_alive},'
                f'pi3_alive:{self.pi3_alive},'
                f'emergency:{self.emergency},'
                f'estop_stm1:{self.device_estop["stm1"]},'
                f'estop_scara:{self.device_estop["scara"]},'
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

                f'nursery_left_slot:{self.nursery_slots["left"]["state"]},'
                f'nursery_right_slot:{self.nursery_slots["right"]["state"]},'
                f'water_left_slot:{self.water_slots["left"]["state"]},'
                f'water_right_slot:{self.water_slots["right"]["state"]},'
                f'active_scara_job:{active_scara_job_id},'
                f'active_manip_job:{active_manip_job_id},'
                f'demo_phase:{self.demo_phase}'
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
        imgsz=416, #640,
        conf=0.25,
        device=node.yolo_device,
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
    reserved_slot = None
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

                                        # 트레이가 안정적으로 감지되면
                    # 발아실 빈 슬롯을 먼저 예약한 후 C1F=1 전송
                    if (
                        node.stable_cnt >= STABLE_FRAMES
                        and not node.start_flag
                        and node.seed_target_slot is None
                        and node.stm_state == 'idle'
                        and (now - node.last_tx) > COOLDOWN_SEC
                        and not node.emergency
                        and node.pi1_alive
                    ):
                        target_slot = node._find_empty_slot_locked(
                            'nursery'
                        )

                        if target_slot is not None:
                            # C1F를 보내기 전에 슬롯부터 예약한다.
                            # 그래야 다음 카메라 프레임이 같은 슬롯을
                            # 중복으로 선택하지 않는다.
                            node.nursery_slots[target_slot]['state'] = (
                                SLOT_RESERVED_IN
                            )
                            node.nursery_slots[target_slot]['job_id'] = None

                            node.seed_target_slot = target_slot

                            # RESERVED_IN도 UV 점유 개수에 포함
                            node._sync_slot_flags_locked()

                            node.start_flag = True
                            node.last_tx = now
                            node.stable_cnt = 0
                            node._set_flag('c1f', 1)

                            reserved_slot = target_slot
                            send_now = True

                        else:
                            # 빈 슬롯이 없을 때 매 프레임마다 로그가
                            # 출력되지 않도록 최초 임계점에서만 경고
                            if node.stable_cnt == STABLE_FRAMES:
                                node.get_logger().warn(
                                    '트레이 감지됐지만 '
                                    '발아실 예약 가능한 슬롯 없음'
                                )
                else:
                    node.stable_cnt   = 0
                    node.no_tray_cnt += 1

        if send_now:
            msg = UInt8MultiArray()
            msg.data = make_flag_u8(PID_C1F, 1)
            node.pub_pi1.publish(msg)

            node.get_logger().info(
                f'트레이 감지 → '
                f'발아실 {reserved_slot} 예약 → '
                f'C1F=1 전송'
            )

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

    
    put_latest(frame_queue, disp)
    #try:
    #    put_latest(frame_queue, disp)
    #except queue.Full:
    #    pass

def merge_nearby_sprout_boxes(boxes, merge_distance=22):
    """
    서로 가까운 새싹 contour 박스를 하나의 새싹 후보로 합친다.
    """
    if not boxes:
        return []

    groups = []

    for box in boxes:
        x1, y1, x2, y2, area = box
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        merged = False

        for group in groups:
            gx1, gy1, gx2, gy2, garea = group

            gcx = (gx1 + gx2) / 2
            gcy = (gy1 + gy2) / 2

            distance = np.hypot(cx - gcx, cy - gcy)

            if distance < merge_distance:
                group[0] = min(gx1, x1)
                group[1] = min(gy1, y1)
                group[2] = max(gx2, x2)
                group[3] = max(gy2, y2)
                group[4] = garea + area
                merged = True
                break

        if not merged:
            groups.append([x1, y1, x2, y2, area])

    return [tuple(group) for group in groups]
def detect_sprouts_by_color(
    frame: np.ndarray,
    roi_x_min: float,
    roi_x_max: float,
    roi_y_min: float,
    roi_y_max: float
):
    frame_h, frame_w = frame.shape[:2]

    # 비율 ROI → 픽셀 좌표
    rx1 = int(frame_w * roi_x_min)
    rx2 = int(frame_w * roi_x_max)
    ry1 = int(frame_h * roi_y_min)
    ry2 = int(frame_h * roi_y_max)

    # 이미지 범위 안전 보정
    rx1 = max(0, rx1)
    ry1 = max(0, ry1)
    rx2 = min(frame_w, rx2)
    ry2 = min(frame_h, ry2)

    if rx2 <= rx1 or ry2 <= ry1:
        empty_mask = np.zeros((1, 1), dtype=np.uint8)
        return 0, [], empty_mask

    # 발아실 ROI 추출
    roi = frame[ry1:ry2, rx1:rx2]

    roi_h, roi_w = roi.shape[:2]

    # BGR → HSV
    hsv = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2HSV
    )
    # 초록
    green_mask = cv2.inRange(
        hsv,
        NURSERY_LOWER_GREEN,
        NURSERY_UPPER_GREEN
    )
    # 노랑
    yellow_mask = cv2.inRange(
        hsv,
        NURSERY_LOWER_YELLOW,
        NURSERY_UPPER_YELLOW
    )

    mask = cv2.bitwise_or(
        green_mask,
        yellow_mask
    )

    # ROI 오른쪽 새싹 영역
    right_start = int(roi_w * 0.67)

    right_roi = roi[:, right_start:]
    right_hsv = hsv[:, right_start:]

    # 오른쪽 새싹은 밝은 노랑·황록색
    right_hsv_mask = cv2.inRange(
        right_hsv,
        np.array([24, 45, 115], dtype=np.uint8),
        np.array([42, 220, 255], dtype=np.uint8)
    )

    # BGR 채널 조건
    b, g, r = cv2.split(right_roi)

    right_bgr_mask = (
        (g >= 120)
        & (r >= 110)
        & ((g.astype(np.int16) - b.astype(np.int16)) >= 25)
        & ((r.astype(np.int16) - b.astype(np.int16)) >= 20)
    ).astype(np.uint8) * 255

    # HSV 또는 BGR 조건을 만족하는 영역
    right_light_mask = cv2.bitwise_or(
        right_hsv_mask,
        right_bgr_mask
    )

    # 작은 점만 제거
    right_kernel = np.ones((3, 3), dtype=np.uint8)

    right_light_mask = cv2.morphologyEx(
        right_light_mask,
        cv2.MORPH_OPEN,
        right_kernel,
        iterations=1
    )

    right_light_mask = cv2.morphologyEx(
        right_light_mask,
        cv2.MORPH_CLOSE,
        right_kernel,
        iterations=1
    )

    mask[:, right_start:] = right_light_mask

    # ------------------------------------------
    # 가운데 흰색 판 영역 강제 제외
    # ------------------------------------------
    exclude_x1 = int(
        roi_w * NURSERY_CENTER_EXCLUDE_X_MIN
    )

    exclude_x2 = int(
        roi_w * NURSERY_CENTER_EXCLUDE_X_MAX
    )

    exclude_x1 = max(0, exclude_x1)
    exclude_x2 = min(roi_w, exclude_x2)

    if exclude_x2 > exclude_x1:
        mask[:, exclude_x1:exclude_x2] = 0

    # ------------------------------------------
    # 작은 점 제거
    # ------------------------------------------
    open_kernel = np.ones(
        (
            NURSERY_OPEN_KERNEL_SIZE,
            NURSERY_OPEN_KERNEL_SIZE
        ),
        dtype=np.uint8
    )

    mask_clean = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        open_kernel,
        iterations=1
    )

    # ------------------------------------------
    # 한 새싹의 떨어진 잎 영역 연결
    # ------------------------------------------
    close_kernel = np.ones(
        (
            NURSERY_CLOSE_KERNEL_SIZE,
            NURSERY_CLOSE_KERNEL_SIZE
        ),
        dtype=np.uint8
    )

    mask_clean = cv2.morphologyEx(
        mask_clean,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=1
    )

    # 윤곽선 검출
    contours, _ = cv2.findContours(
        mask_clean,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    sprout_boxes = []

    for contour in contours:
        area = cv2.contourArea(contour)

        # 너무 작은 노이즈 제거
        if area < NURSERY_MIN_SPROUT_AREA:
            continue

        # 지나치게 큰 배경 영역 제거
        if area > NURSERY_MAX_SPROUT_AREA:
            continue

        x, y, bw, bh = cv2.boundingRect(contour)

        # 너무 작거나 가느다란 형태 제거
        if bw < NURSERY_MIN_SPROUT_WIDTH:
            continue

        if bh < NURSERY_MIN_SPROUT_HEIGHT:
            continue
        
        # 지나치게 가늘고 긴 물체 제거
        aspect_ratio = bw / float(bh)

        if aspect_ratio < 0.20 or aspect_ratio > 5.0:
            continue

        # 박스 면적 대비 실제 초록 영역 비율
        box_area = bw * bh
        fill_ratio = area / max(box_area, 1)

        if fill_ratio < 0.08:
            continue

        # ROI 가장자리에 붙은 구조물 제거
        # border_margin = 5

        # touches_border = (
        #     x <= border_margin
        #     or y <= border_margin
        #     or x + bw >= roi_w - border_margin
        #     or y + bh >= roi_h - border_margin
        # )

        # if touches_border:
        #     continue

        # ROI 좌표 → 전체 프레임 좌표
        x1 = rx1 + x
        y1 = ry1 + y
        x2 = x1 + bw
        y2 = y1 + bh

        sprout_boxes.append(
            (x1, y1, x2, y2, area)
        )

    sprout_boxes = merge_nearby_sprout_boxes(
        sprout_boxes,
        merge_distance=35
    )

    sprout_boxes.sort(
        key=lambda box: (box[1], box[0])
    )

    sprout_count = len(sprout_boxes)

    return sprout_count, sprout_boxes, mask_clean

def process_nursery_frame(
    node: MasterNode,
    frame: np.ndarray,
    position: str
):  
    # ─────────────────────────────────────────
    # 1. YOLO 추론
    # YOLO로 발아실 트레이 검출
    # OpenCV HSV 색상 분할로 새싹 검출
    # ─────────────────────────────────────────
    with node.yolo_lock:
        tray_results = node.nursery_tray_model.predict(
            source=frame,
            imgsz=416,
            conf=NURSERY_TRAY_CONF,
            device=node.yolo_device,
            verbose=False
        )[0]

    disp = frame.copy()
    h, w = frame.shape[:2]

    roi_x_min = NURSERY_ROI_X_MIN
    roi_x_max = NURSERY_ROI_X_MAX
    roi_y_min = NURSERY_ROI_Y_MIN
    roi_y_max = NURSERY_ROI_Y_MAX
    
    # 가운데 흰색 영역 표시
    roi_px1 = int(w * roi_x_min)
    roi_px2 = int(w * roi_x_max)
    roi_py1 = int(h * roi_y_min)
    roi_py2 = int(h * roi_y_max)

    roi_width = roi_px2 - roi_px1

    exclude_px1 = roi_px1 + int(
        roi_width * NURSERY_CENTER_EXCLUDE_X_MIN
    )

    exclude_px2 = roi_px1 + int(
        roi_width * NURSERY_CENTER_EXCLUDE_X_MAX
    )

    cv2.rectangle(
        disp,
        (exclude_px1, roi_py1),
        (exclude_px2, roi_py2),
        (0, 0, 255),
        2
    )

    cv2.putText(
        disp,
        'IGNORE',
        (exclude_px1 + 5, roi_py1 + 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2
    )

    # ─────────────────────────────────────────
    # 2. op_sprout 기반 새싹 개수 판단
    # ─────────────────────────────────────────
    # sprout_count = 0
    # sprout_done = False

    # if detect_sprouts is not None:
    #     try:
    #         rx1 = int(w * roi_x_min)
    #         rx2 = int(w * roi_x_max)
    #         ry1 = int(h * roi_y_min)
    #         ry2 = int(h * roi_y_max)

    #         roi_frame = frame[ry1:ry2, rx1:rx2]

    #         _, sprout_count, _ = detect_sprouts(roi_frame)
    #         sprout_done = sprout_count >= NURSERY_SPROUT_DONE_COUNT

    #     except Exception as e:
    #         node.get_logger().warn(
    #             f'[Sprout {position}] detect_sprouts failed: {e}'
    #         )

    # ─────────────────────────────────────────
    # 3. ROI 표시
    # ─────────────────────────────────────────
    cv2.rectangle(
        disp,
        (int(w * roi_x_min), int(h * roi_y_min)),
        (int(w * roi_x_max), int(h * roi_y_max)),
        (255, 255, 0),
        2
    )

    tray_count = 0
    sprout_count = 0

    #best_sprout3_conf = 0.0

    # ─────────────────────────────────────────
    # 4. 트레이 모델 처리
    # tray_results만 사용
    # ─────────────────────────────────────────
    for box in tray_results.boxes:
        x1, y1, x2, y2 = map(
            int,
            box.xyxy[0].tolist()
        )

        cls_id = int(box.cls[0].item())
        conf = float(box.conf[0].item())

        class_name = node.nursery_tray_model.names[cls_id]

        cx = (x1 + x2) / 2 / w
        cy = (y1 + y2) / 2 / h
        box_w_ratio = (x2 - x1) / w
        box_h_ratio = (y2 - y1) / h

        # 왼쪽 카메라는 기존 조건 유지
        min_box_ratio = (
            0.08
            if position == 'left'
            else NURSERY_TRAY_MIN_BOX_RATIO
        )

        in_roi = (
            roi_x_min < cx < roi_x_max
            and roi_y_min < cy < roi_y_max
            and box_w_ratio > min_box_ratio
            and box_h_ratio > min_box_ratio
        )

        color = (0, 255, 0) if in_roi else (0, 0, 255)

        cv2.rectangle(
            disp,
            (x1, y1),
            (x2, y2),
            color,
            2
        )

        cv2.putText(
            disp,
            f'tray {conf:.2f}',
            (x1, max(20, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2
        )

        # 새 트레이 모델이 클래스 이름을 tray로 사용한다는 전제
        if class_name == 'tray' and in_roi:
            tray_count = 1

    tray_detected = tray_count > 0

    # 트레이가 있을 때만 OpenCV 새싹 검출
    if tray_detected:
        sprout_count, sprout_boxes, sprout_mask = detect_sprouts_by_color(
            frame,
            roi_x_min,
            roi_x_max,
            roi_y_min,
            roi_y_max
        )
    else:
        sprout_count = 0
        sprout_boxes = []

        roi_h = int(h * (roi_y_max - roi_y_min))
        roi_w = int(w * (roi_x_max - roi_x_min))

        sprout_mask = np.zeros(
            (max(1, roi_h), max(1, roi_w)),
            dtype=np.uint8
        )   

    # 검출된 새싹 박스 표시
    for index, (x1, y1, x2, y2, area) in enumerate(
        sprout_boxes,
        start=1
    ):
        cv2.rectangle(
            disp,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )

        cv2.putText(
            disp,
            f'{index}: {int(area)}',
            (x1, max(20, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1
        )
    # mask_bgr = cv2.cvtColor(
    #     sprout_mask,
    #     cv2.COLOR_GRAY2BGR
    # )

    # preview_w = 240
    # preview_h = 140

    # mask_preview = cv2.resize(
    #     mask_bgr,
    #     (preview_w, preview_h)
    # )

    # ph, pw = disp.shape[:2]

    # if ph >= preview_h and pw >= preview_w:
    #     disp[
    #         ph - preview_h:ph,
    #         pw - preview_w:pw
    #     ] = mask_preview

    # ─────────────────────────────────────────
    # 6. OpenCV 새싹 개수로 발아 완료 판단
    # ─────────────────────────────────────────
    # YOLO에서 검출한 전체 새싹 개수로 발아 완료 판단
    sprout_done_detected = (
        sprout_count >= NURSERY_SPROUT_DONE_COUNT
    )

    sprout_progress = min(
        1.0,
        sprout_count / max(NURSERY_SPROUT_DONE_COUNT, 1)
    )

    now = time.time()
    send_event = False
    flag_name = None

    # ─────────────────────────────────────────
# 7. 카메라 관측값과 논리 슬롯 상태 처리
# ─────────────────────────────────────────
    with node.state_lock:
        # 카메라 관측 상태
        st = node.nursery_state[position]

        # 마스터가 관리하는 논리 슬롯 상태
        logical_slot = node.nursery_slots[position]

        previous_slot_state = logical_slot['state']
        previous_uv = node.flags['uv']
        previous_ulf = node.flags['ulf']
        previous_urf = node.flags['urf']

        # ─────────────────────────────────────
        # 7-1. 트레이 카메라 관측 카운터
        # ─────────────────────────────────────
        if tray_detected:
            st['tray_seen_cnt'] += 1
            st['tray_lost_cnt'] = 0
        else:
            st['tray_lost_cnt'] += 1
            st['tray_seen_cnt'] = 0

        # ─────────────────────────────────────
        # 7-2. 트레이 점유 확정
        # ─────────────────────────────────────
        if st['tray_seen_cnt'] >= TRAY_OCCUPY_FRAMES:
            st['occupied'] = 1

            # 사람이 직접 트레이를 놓은 경우
            # EMPTY/UNKNOWN에서 OCCUPIED로 등록
            if logical_slot['state'] in (
                SLOT_EMPTY,
                SLOT_UNKNOWN,
            ):
                logical_slot['state'] = SLOT_OCCUPIED
                logical_slot['job_id'] = None

            # RESERVED_IN은 그대로 둔다.
            # SCARA 완료 이벤트가 와야 OCCUPIED로 확정한다.

        # ─────────────────────────────────────
        # 7-3. 트레이 소실 확정
        # ─────────────────────────────────────
        if st['tray_lost_cnt'] >= TRAY_LOST_FRAMES:
            st['occupied'] = 0

            # 일반 점유 트레이가 사라진 경우에만 EMPTY
            if logical_slot['state'] in (
                SLOT_OCCUPIED,
                SLOT_READY,
            ):
                logical_slot['state'] = SLOT_EMPTY
                logical_slot['job_id'] = None

            # RESERVED_IN:
            # 아직 SCARA가 트레이를 가져오는 중일 수 있으므로 유지
            #
            # RESERVED_OUT:
            # SCARA가 트레이를 들고 이동 중일 수 있으므로 유지
            #
            # 두 상태 모두 카메라만 보고 EMPTY로 만들지 않는다.

        # ─────────────────────────────────────
        # 7-4. 발아 진행 상태 표시
        # ─────────────────────────────────────
        if tray_detected and sprout_done_detected:
            st['stable_cnt'] += 1
            st['last_label'] = 'sprout_done'
            st['last_conf'] = sprout_progress

        else:
            st['stable_cnt'] = 0

            if not tray_detected:
                st['last_label'] = 'none'
                st['last_conf'] = 0.0

            elif sprout_count == 0:
                st['last_label'] = 'tray_only'
                st['last_conf'] = 0.0

            else:
                st['last_label'] = 'sprout_growing'
                st['last_conf'] = sprout_progress

        # ─────────────────────────────────────
        # 7-5. 발아 완료 확정
        # ─────────────────────────────────────
        if (
            st['stable_cnt'] >= NURSERY_STABLE_FRAMES
            and now - st['last_tx'] > NURSERY_COOLDOWN_SEC
        ):
            st['last_tx'] = now
            st['stable_cnt'] = 0

            # 트레이 감지 + 발아 완료를 모두 확인했으므로
            # READY 상태로 변경
            if logical_slot['state'] in (
                SLOT_EMPTY,
                SLOT_UNKNOWN,
                SLOT_OCCUPIED,
            ):
                logical_slot['state'] = SLOT_READY
                logical_slot['job_id'] = None

            # RESERVED_IN은 SCARA 완료 전이므로 변경하지 않는다.
            # RESERVED_OUT도 이동 중이므로 변경하지 않는다.

        # ─────────────────────────────────────
        # 7-6. 논리 슬롯을 기존 플래그로 변환
        # ─────────────────────────────────────
        node._sync_slot_flags_locked()

        new_uv = node.flags['uv']
        ulf_value = node.flags['ulf']
        urf_value = node.flags['urf']

        # ─────────────────────────────────────
        # 7-7. 외부 전송 필요 여부 판단
        # ─────────────────────────────────────
        uv_changed = previous_uv != new_uv
        ulf_changed = previous_ulf != ulf_value
        urf_changed = previous_urf != urf_value

        if uv_changed or ulf_changed or urf_changed:
            send_event = True

        if ulf_changed and ulf_value == 1:
            flag_name = 'ULF'

        elif urf_changed and urf_value == 1:
            flag_name = 'URF'

        # ─────────────────────────────────────
        # 7-8. 상태 변화 로그
        # ─────────────────────────────────────
        if previous_slot_state != logical_slot['state']:
            node.get_logger().info(
                f'[Nursery {position}] '
                f'slot state: '
                f'{previous_slot_state} '
                f'-> {logical_slot["state"]}'
            )

        if send_event:
            if flag_name is not None:
                node.get_logger().info(
                    f'[Nursery {position}] '
                    f'sprout done -> {flag_name}=1, '
                    f'sprout_count={sprout_count}, '
                    f'uv={new_uv}'
                )
            else:
                node.get_logger().info(
                    f'[Nursery {position}] '
                    f'flag changed: '
                    f'UV={new_uv}, '
                    f'ULF={ulf_value}, '
                    f'URF={urf_value}'
                )

        stable = st['stable_cnt']
        label = st['last_label']
        slot_state = logical_slot['state']

    # ─────────────────────────────────────────
    # 8. 로그 출력
    # ─────────────────────────────────────────
    node.get_logger().info(
        f'[Nursery {position}] '
        f'tray={tray_count}, '
        f'sprouts={sprout_count}/{NURSERY_SPROUT_DONE_COUNT}, '
        f'progress={sprout_progress:.2f}, '
        f'done={sprout_done_detected}, '
        f'stable={stable}, '
        f'label={label}, '
        f'slot={slot_state}'
    )

    # ─────────────────────────────────────────
    # 9. 화면 텍스트
    # ─────────────────────────────────────────
    y = 30

    cv2.putText(
        disp,
        f'tray: {tray_count}',
        (20, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2
    )

    cv2.putText(
        disp,
        f'sprouts: {sprout_count}/{NURSERY_SPROUT_DONE_COUNT}',
        (20, y + 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    cv2.putText(
        disp,
        f'progress: {sprout_progress:.2f}',
        (20, y + 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    cv2.putText(
        disp,
        f'stable: {stable}/{NURSERY_STABLE_FRAMES}',
        (20, y + 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2
    )

    cv2.putText(
        disp,
        f'label: {label}',
        (20, y + 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2
    )

    target_queue = (
        nursery_left_frame_queue
        if position == 'left'
        else nursery_right_frame_queue
    )

    put_latest(target_queue, disp)

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

#수경재배실 프레임 처리 함수
def process_water_frame(node: MasterNode, frame: np.ndarray, position: str):
    disp = frame.copy()
    h, w = frame.shape[:2]
    rx1 = int(w * WATER_ROI_X_MIN)
    rx2 = int(w * WATER_ROI_X_MAX)
    ry1 = int(h * WATER_ROI_Y_MIN)
    ry2 = int(h * WATER_ROI_Y_MAX)

    roi = frame[ry1:ry2, rx1:rx2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, WATER_LOWER_GREEN, WATER_UPPER_GREEN)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    green_pixels = cv2.countNonZero(mask)
    roi_pixels = max(1, mask.shape[0] * mask.shape[1])
    green_ratio = green_pixels / roi_pixels

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    largest_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > largest_area:
            largest_area = area

    growth_done = (
        green_ratio >= WATER_GROWTH_AREA_RATIO
        and largest_area >= WATER_MIN_LEAF_AREA
    )

    now = time.time()
    send_event = False
    flag_name = 'wlf' if position == 'left' else 'wrf'
    pid = PID_WLF if position == 'left' else PID_WRF

    with node.state_lock:
        st = node.water_state[position]

        if growth_done:
            st['stable_cnt'] += 1
            st['lost_cnt'] = 0
            st['last_label'] = 'growth_done'
            st['last_score'] = green_ratio
        else:
            st['stable_cnt'] = 0
            st['lost_cnt'] += 1
            st['last_label'] = 'growing' if green_ratio > 0.03 else 'none'
            st['last_score'] = green_ratio

        if (
            st['stable_cnt'] >= WATER_STABLE_FRAMES
            and now - st['last_tx'] > WATER_COOLDOWN_SEC
        ):
            st['last_tx'] = now
            st['stable_cnt'] = 0

            if node.flags[flag_name] == 0:
                node._set_flag(flag_name, 1)
                send_event = True
                node.get_logger().info(
                    f'[Water {position}] growth done -> '
                    f'{flag_name.upper()}=1, '
                    f'green_ratio={green_ratio:.3f}, '
                    f'largest_area={largest_area:.1f}'
                )

        if st['lost_cnt'] >= WATER_LOST_FRAMES:
            if node.flags[flag_name] != 0:
                node._set_flag(flag_name, 0)
                send_event = True
                node.get_logger().info(
                    f'[Water {position}] plant lost -> {flag_name.upper()}=0'
                )

    if send_event:
        node._broadcast_all_flags()
#수경재배실 카메라
def water_video_receive_loop(node: MasterNode, stream_port: int, position: str, calib_path: str | None = None):
    K = None
    dist = None
    map1, map2 = None, None

    if calib_path is not None:
        try:
            calib = np.load(calib_path)
            K = calib['camera_matrix']
            dist = calib['dist_coeffs']
            print(f'[Water Calib {position}] 로드 완료: {calib_path}')
        except Exception as e:
            print(f'[Water Calib {position}] 로드 실패: {e}')
            K = None
            dist = None
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', stream_port))
    server.listen(1)
    print(f'[Water Stream {position}] 연결 대기 (port={stream_port})')
    target_queue = (
        water_left_frame_queue
        if position == 'left'
        else water_right_frame_queue
    )

    while rclpy.ok():
        conn = None
        try:
            conn, addr = server.accept()
            print(f'[Water Stream {position}] 연결됨: {addr}')

            data = b''
            payload_size = struct.calcsize('>I')
            last_process_ts = 0.0
            PROCESS_INTERVAL = 0.3

            while rclpy.ok():
                while len(data) < payload_size:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionError('water stream disconnected')
                    data += packet

                msg_size = struct.unpack('>I', data[:payload_size])[0]
                data = data[payload_size:]

                while len(data) < msg_size:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionError('water stream disconnected')
                    data += packet

                frame_data = data[:msg_size]
                data = data[msg_size:]

                now = time.time()
                if now - last_process_ts < PROCESS_INTERVAL:
                    continue

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
                        print(f'[Water Calib {position}] 왜곡 보정 맵 초기화 완료')

                    frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

                cv2.putText(
                    frame,
                    f'Pi3 Water {position}',
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 255),
                    2
                )

                put_latest(target_queue, frame)

                last_process_ts = now
                process_water_frame(node, frame, position)

        except Exception as e:
            print(f'[Water Stream {position}] 오류: {e}')
        finally:
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass

# 발아실 카메라
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
            PROCESS_INTERVAL = 0.3

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

                # 처리 주기가 아니면 JPEG 디코딩도 하지 않고 바로 버림
                now = time.time()
                if now - last_process_ts < PROCESS_INTERVAL:
                    continue

                frame = cv2.imdecode(
                    np.frombuffer(frame_data, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )

                if frame is None:
                    continue
                
                last_process_ts = now

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
    threading.Thread(
        target=water_video_receive_loop,
        args=(node, WATER_LEFT_STREAM_PORT, 'left', WATER_LEFT_CALIB_PATH),
        daemon=True
    ).start()
    threading.Thread(
        target=water_video_receive_loop,
        args=(node, WATER_RIGHT_STREAM_PORT, 'right', WATER_RIGHT_CALIB_PATH),
        daemon=True
    ).start()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            try:
                frame = frame_queue.get_nowait()
                cv2.imshow('Pi1 Camera', frame)               
            except queue.Empty:
                pass
            try:
                frame = nursery_left_frame_queue.get_nowait()
                cv2.imshow('Pi1 Nursery Left Camera', frame)
            except queue.Empty:
                pass
            try:
                frame = nursery_right_frame_queue.get_nowait()
                cv2.imshow('Pi1 Nursery Right Camera', frame)
            except queue.Empty:
                pass
            try:
                frame = water_left_frame_queue.get_nowait()
                cv2.imshow('Pi3 Water Left Camera', frame)
            except queue.Empty:
                pass

            try:
                frame = water_right_frame_queue.get_nowait()
                cv2.imshow('Pi3 Water Right Camera', frame)
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