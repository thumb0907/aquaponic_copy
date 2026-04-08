#!/usr/bin/env python3
"""
master_node.py — PC 마스터 노드

역할:
  1. Pi1 카메라 영상 수신 → YOLO 추론 → 트레이 감지
  2. 트레이 감지 시 SSF=1 바이너리 프레임 → Pi1 → STM1
  3. STM1/STM2 상태 수신 → 플래그 관리
  4. 전체 플래그를 모니터 노드에 publish
  5. 긴급정지/리셋 명령 처리
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

# ── 설정 ──────────────────────────────────────
MODEL_PATH    = '/home/thumb/aquaponic_copy/tray2/best.pt'
STREAM_PORT   = 5000
TRAY_CLASS_ID = 0
MIN_CONF      = 0.90
STABLE_FRAMES = 5
COOLDOWN_SEC  = 2.0

# ROI (트레이 감지 유효 영역)
ROI_X_MIN = 0.37
ROI_X_MAX = 0.73
ROI_Y_MIN = 0.10
ROI_Y_MAX = 0.90

# 박스 최소 크기 (화면 대비 비율)
MIN_BOX_RATIO = 0.25

# 캘리브레이션 파일
CALIB_PATH = '/home/thumb/aquaponic_copy/smartfarm_ws/camera_calib.npz'

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
    """2바이트 플래그 프레임 (UV용)"""
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
        self.start_flag  = False
        self.stm_state   = 'idle'
        self.stable_cnt  = 0
        self.no_tray_cnt = 0
        self.last_tx     = 0.0
        self.pi1_alive   = False
        self.pi1_last_hb = 0.0

        # ── STM2 상태 ─────────────────────────
        self.stm2_state  = 'idle'
        self.pi2_alive   = False
        self.pi2_last_hb = 0.0

        # ── 카메라 상태 ───────────────────────
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

        # ── 공통 ──────────────────────────────
        self.emergency = False

        # ── YOLO 모델 ─────────────────────────
        print('[YOLO] 모델 로딩 중...')
        self.model = YOLO(MODEL_PATH)
        print('[YOLO] 로딩 완료')

        # ── Publisher ─────────────────────────
        # Pi1 → STM1 명령 (바이너리)
        self.pub_pi1     = self.create_publisher(
            UInt8MultiArray, '/pi1/uart_cmd', 10)
        # Pi2 → STM2 명령 (바이너리) ← String에서 변경
        self.pub_pi2     = self.create_publisher(
            UInt8MultiArray, '/pi2/uart_cmd', 10)
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
            String, '/pi2/flag_update',      # Pi2 플래그 업데이트
            self._on_pi2_flag, 10)
        self.create_subscription(
            String, '/system/heartbeat',
            self._on_heartbeat, 10)
        self.create_subscription(
            String, '/system/command',
            self._on_command, 10)
        
        # Pi2 플래그 업데이트
        self.create_subscription(
            String, '/pi2/flag_update',
            self._on_pi2_flag, 10)

        self.create_timer(1.0, self._check_heartbeat)
        self.create_timer(0.5, self._publish_monitor)

        self.get_logger().info('Master 노드 시작')

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
            # 파종 완료 → PC가 직접 Pi2에 SSF=1, CRF=1 전송
                self.stm_state = 'waiting_scara'
                self._send_binary_pi2(make_flag_u8(PID_SSF, 1))
                self._send_binary_pi2(make_flag_u8(PID_CRF, 1))
                self.get_logger().info('파종 완료 → Pi2에 SSF=1, CRF=1 전송')
            elif line == 'STM1:PC:STATE:IDLE':
                self.stm_state  = 'idle'
                self.start_flag = False
            elif line.startswith('STM1:PC:ERR:'):
                self.stm_state  = 'error'
                self.start_flag = False
                self.get_logger().error(line)
            elif line.startswith('STM1:PC:FLAG:'):
                # STM1:PC:FLAG:SSF:1
                parts = line.split(':')
                if len(parts) == 5:
                    k = parts[3].lower()
                    v = int(parts[4])
                    if k in self.flags:
                        self.flags[k] = v
            elif line.startswith('STM1:PI1:ACK:'):
                self.get_logger().info(f'ACK: {line}')

    # ══════════════════════════════════════════
    # STM2 상태 수신
    # ══════════════════════════════════════════
    def _on_uart2(self, msg: String):
        line = msg.data.strip()
        self.get_logger().info(f'[STM2] {line}')

        with self.state_lock:
            if line == 'STM2:PC:STATE:CONVEY_RUN':
                self.stm2_state = 'convey_run'
            elif line == 'STM2:PC:STATE:IR_DETECTED':
                self.stm2_state = 'ir_detected'
            elif line == 'STM2:PC:STATE:Z_FIX':
                self.stm2_state = 'z_fix'
            elif line == 'STM2:PC:STATE:HARVESTING':
                self.stm2_state = 'harvesting'
            elif line == 'STM2:PC:STATE:EJECTING':
                self.stm2_state = 'ejecting'
            elif line == 'STM2:PC:DONE:CYCLE':
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
    # ══════════════════════════════════════════
    def _on_pi2_flag(self, msg: String):
        """
        Pi2에서 오는 플래그 업데이트
        형식: FLAG:SMF:1
        """
        line  = msg.data.strip()
        parts = line.split(':')

        with self.state_lock:
            if len(parts) == 3 and parts[0] == 'FLAG':
                k = parts[1].lower()
                v = int(parts[2])
                if k in self.flags:
                    self.flags[k] = v
                    self.get_logger().info(
                        f'Pi2 플래그: {k}={v}')

                # 발아 완료 → Pi2에 이동 명령
                if k in ('ulf', 'urf') and v == 1:
                    self._relay_to_pi2(f'UV_DONE:{k.upper()}')

                # 성장 완료 → Pi2에 이동 명령
                if k in ('wlf', 'wrf') and v == 1:
                    self._relay_to_pi2(f'WATER_DONE:{k.upper()}')

    # ══════════════════════════════════════════
    # Pi1 ↔ Pi2 중계 수신 확인
    # ══════════════════════════════════════════
    def _on_link_to_pi1(self, msg: String):
        payload = msg.data.strip()
        if payload:
            self.get_logger().info(
                f'[InterPi] Pi2→Pi1 확인: {payload}')

    def _on_link_to_pi2(self, msg: String):
        payload = msg.data.strip()
        if payload:
            self.get_logger().info(
                f'[InterPi] Pi1→Pi2 확인: {payload}')

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

    def _check_heartbeat(self):
        now = time.time()
        with self.state_lock:
            self.pi1_alive = (now - self.pi1_last_hb) < 3.0
            self.pi2_alive = (now - self.pi2_last_hb) < 3.0

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
            self._send_binary(make_estop())      # STM1
            self._send_binary_pi2(make_estop())  # STM2
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
            self._send_binary(make_reset())      # STM1
            self._send_binary_pi2(make_reset())  # STM2
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
            # 트레이 올려놨다는 신호 → FF=1 바이너리 전송
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
        """Pi2 → STM2 바이너리 전송"""
        msg      = UInt8MultiArray()
        msg.data = list(frame)  # bytes도 list로 변환
        self.pub_pi2.publish(msg)

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
                f'emergency:{self.emergency},'
                f'cam1_connected:{self.cam1_connected},'
                f'cam1_status:{self.cam1_status},'
                f'cam1_detect_label:{self.cam1_detect_label},'
                f'cam1_last_conf:{self.cam1_last_conf:.2f}'
            )
            flags_str = ','.join(
                f'{k}:{v}' for k, v in self.flags.items()
            )
        msg = String()
        msg.data = status + ',' + flags_str
        self.pub_monitor.publish(msg)


# ══════════════════════════════════════════════
# YOLO 프레임 처리
# ══════════════════════════════════════════════
def process_frame(node: MasterNode, frame: np.ndarray):
    results = node.model(frame, verbose=False)[0]

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
                        node.start_flag  = True
                        node.last_tx     = now
                        node.stable_cnt  = 0
                        node.flags['ssf'] = 1
                        send_now = True
                else:
                    node.stable_cnt   = 0
                    node.no_tray_cnt += 1

    if send_now:
        # 바이너리 SSF=1 프레임 전송
        msg = UInt8MultiArray()
        msg.data = make_flag_u8(PID_SSF, 1)
        node.pub_pi1.publish(msg)
        node.get_logger().info('트레이 감지 → SSF=1 전송')

    # ── 화면 오버레이 ─────────────────────────
    with node.state_lock:
        start_flag       = node.start_flag
        stm_state        = node.stm_state
        stable_cnt       = node.stable_cnt
        last_bbox        = node.cam1_last_bbox
        last_bbox_ts     = node.cam1_last_bbox_ts
        hold_sec         = node.cam1_overlay_hold_sec
        last_conf_hold   = node.cam1_last_conf_hold
        emergency        = node.emergency
        pi1_ok           = node.pi1_alive

    h, w = disp.shape[:2]

    # ROI 박스
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


# ══════════════════════════════════════════════
# 카메라 스트림 수신
# ══════════════════════════════════════════════
def video_receive_loop(node: MasterNode):
    # 캘리브레이션 로드
    calib = np.load(CALIB_PATH)
    K     = calib['camera_matrix']
    dist  = calib['dist_coeffs']
    map1, map2 = None, None

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', STREAM_PORT))
    server.listen(1)
    print(f'[Stream] Pi1 연결 대기 (port={STREAM_PORT})')

    while rclpy.ok():
        conn = None
        try:
            conn, addr = server.accept()
            print(f'[Stream] Pi1 연결됨: {addr}')

            with node.state_lock:
                node.cam1_connected = True

            data         = b''
            payload_size = struct.calcsize('>I')

            while rclpy.ok():
                # 프레임 크기 수신
                while len(data) < payload_size:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionError('stream disconnected')
                    data += packet

                msg_size = struct.unpack('>I', data[:payload_size])[0]
                data     = data[payload_size:]

                # 프레임 데이터 수신
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

                # 캘리브레이션 맵 초기화 (최초 1회)
                if map1 is None:
                    h, w = frame.shape[:2]
                    newK, _ = cv2.getOptimalNewCameraMatrix(
                        K, dist, (w,h), 0, (w,h))
                    map1, map2 = cv2.initUndistortRectifyMap(
                        K, dist, None, newK, (w,h), cv2.CV_16SC2)
                    print('[Calib] 왜곡 보정 맵 초기화 완료')

                # 왜곡 보정
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


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()

    threading.Thread(
        target=video_receive_loop,
        args=(node,), daemon=True).start()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            try:
                frame = frame_queue.get_nowait()
                cv2.imshow('Pi1 Camera', frame)
                cv2.waitKey(1)
            except queue.Empty:
                pass
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