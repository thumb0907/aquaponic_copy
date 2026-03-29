#!/usr/bin/env python3
"""
master_node.py  ─  PC 마스터 노드
=========================================================
라파1(첫번째 컨베이어) + 라파2(두번째 컨베이어) 통합 관리.
라파3(수경재배실)은 추후 추가 예정.

전체 흐름:
  라파1(pi1_node) ──UART──  STM32 #1 (첫번째 컨베이어, 파종)
  라파2(pi2_node) ──UART──  STM32 #2 (두번째 컨베이어, 수확)
  마스터노드는 라파들과 ROS2 토픽으로 통신하며 전체 상태를 관리

구독 토픽 (받는 것)
  /pi1/uart_response   STM1이 라파1을 통해 보내는 상태 문자열
  /pi2/uart_response   STM2가 라파2를 통해 보내는 상태 문자열
  /system/heartbeat    각 라파가 1초마다 보내는 생존 신호 ('pi1' or 'pi2')
  /system/command      외부에서 보내는 제어 명령 (EMERGENCY / RESET / START_2)

발행 토픽 (보내는 것)
  /pi1/uart_cmd        라파1 → STM1으로 전달할 명령
  /pi2/uart_cmd        라파2 → STM2으로 전달할 명령
  /monitor/state       모니터 노드에 전체 상태 요약 전송
"""

from __future__ import annotations

import time
import threading
import socket
import struct

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import queue
frame_queue = queue.Queue(maxsize=2)

# ── 설정값 ────────────────────────────────────────────────────
#MODEL_PATH    = '/home/thumb/models/best.pt'  # YOLO 모델 경로
#MODEL_PATH    = '/home/thumb/aquaponic_copy/tray2/best.pt'
MODEL_PATH    = '/home/thumb/aquaponic_copy/tray2/test1/best.pt' 
STREAM_PORT   = 5000                           # 라파1 카메라 스트림 수신 포트
TRAY_CLASS_ID = 0                              # YOLO에서 트레이로 인식하는 클래스 번호
MIN_CONF      = 0.90                           # 트레이로 판정하는 최소 신뢰도
STABLE_FRAMES = 5                              # 연속 몇 프레임 감지돼야 진짜로 인정
COOLDOWN_SEC  = 2.0                            # 한 번 전송 후 재전송 대기 시간(초)
# 카메라 감지 영역
ROI_X_MIN = 0.37
ROI_X_MAX = 0.73
ROI_Y_MIN = 0.10
ROI_Y_MAX = 0.90
#캘리브레이션파일위치
CALIB_PATH = '/home/thumb/aquaponic_copy/smartfarm_ws/camera_calib.npz'

class MasterNode(Node):

    def __init__(self):
        super().__init__('master_node')

        # 여러 스레드(ROS 콜백, 카메라 루프)가 동시에 변수를 건드리므로 락 사용
        self.state_lock = threading.Lock()

        # ── 라파1 / STM32 #1 상태 ─────────────────────
        self.start_flag  = False   # CAM1이 트레이를 감지해서 STM1에 명령 보낸 상태
        self.stm_state   = 'idle'  # STM1 현재 동작 상태
        self.stable_cnt  = 0       # 트레이가 연속으로 감지된 프레임 수
        self.no_tray_cnt = 0       # 트레이가 연속으로 없었던 프레임 수
        self.last_tx     = 0.0     # 마지막으로 STM1에 명령 보낸 시각
        self.pi1_alive   = False   # 라파1 heartbeat 기반 연결 상태
        self.pi1_last_hb = 0.0     # 라파1 마지막 heartbeat 수신 시각

        # ── 라파2 / STM32 #2 상태 ─────────────────────
        self.stm2_state  = 'idle'  # STM2 현재 동작 상태
        self.pi2_alive   = False   # 라파2 연결 상태
        self.pi2_last_hb = 0.0     # 라파2 마지막 heartbeat 수신 시각

        # ── 카메라(CAM1) 상태 ─────────────────────────
        self.cam1_connected      = False   # 스트림 소켓 연결 여부
        self.cam1_last_frame_ts  = 0.0     # 마지막 프레임 수신 시각
        self.cam1_last_detect_ts = 0.0     # 마지막 검출 시각
        self.cam1_last_conf      = 0.0     # 마지막 검출 confidence
        self.cam1_status         = 'disconnected'  # normal / delay / disconnected
        self.cam1_detect_label   = 'none'          # tray_detected / none

        # ── 카메라 영상 ──────
        self.cam1_last_bbox = None
        self.cam1_last_bbox_ts = 0.0
        self.cam1_overlay_hold_sec = 0.7
        self.cam1_last_conf_hold = 0.0

        # ── 공통 ───────────────────────────────────────
        self.emergency   = False   # 긴급정지 상태 (True면 모든 동작 차단)

        # ── YOLO 모델 ─────────────────────────────────
        self.model = YOLO(MODEL_PATH)

        # ── 발행 토픽 ─────────────────────────────────
        self.pub_pi1     = self.create_publisher(String, '/pi1/uart_cmd',  10)
        self.pub_pi2     = self.create_publisher(String, '/pi2/uart_cmd',  10)
        self.pub_pi1_link= self.create_publisher(String, '/pi1/interpi_send', 10)
        self.pub_pi2_link= self.create_publisher(String, '/pi2/interpi_send', 10)
        self.pub_monitor = self.create_publisher(String, '/monitor/state', 10)

        # ── 구독 토픽 ─────────────────────────────────
        self.create_subscription(String, '/pi1/uart_response', self._on_uart1,     10)
        self.create_subscription(String, '/pi2/uart_response', self._on_uart2,     10)
        self.create_subscription(String, '/pi1/interpi_rx', self._on_link_to_pi1,     10)
        self.create_subscription(String, '/pi2/interpi_rx', self._on_link_to_pi2,     10)
        self.create_subscription(String, '/system/heartbeat',  self._on_heartbeat, 10)
        self.create_subscription(String, '/system/command',    self._on_command,   10)

        # ── 타이머 ────────────────────────────────────
        self.create_timer(1.0, self._check_heartbeat)
        self.create_timer(0.5, self._publish_monitor)

        self.get_logger().info('Master 노드 시작')

    # ══════════════════════════════════════════════════════
    # 라파1 / STM32 #1 상태 수신
    # ══════════════════════════════════════════════════════
    def _on_uart1(self, msg: String):
        line = msg.data.strip()
        self.get_logger().info(f'[STM1] {line}')

        with self.state_lock:
            if line == 'STM1:PC:STATE:HOMING':
                self.stm_state = 'homing'
            elif line == 'STM1:PC:DONE:HOMING':
                pass
            elif line == 'STM1:PC:STATE:RUN_CONVEYOR1':
                self.stm_state = 'running'
            elif line == 'STM1:PC:STATE:SEEDING':
                self.stm_state = 'seeding'
            elif line == 'STM1:PC:STATE:EJECTING':
                self.stm_state = 'ejecting'
            elif line == 'STM1:PC:STATE:WAIT_SCARA_PICK':
                self.stm_state = 'waiting_scara'
            elif line == 'STM1:PC:STATE:SCARA_PICK_STARTED':
                self.stm_state = 'waiting_scara'
            elif line == 'STM1:PC:STATE:ESTOP':
                self.stm_state  = 'error'
                self.start_flag = False
                self.get_logger().warn('[STM1] ESTOP')
            elif line == 'STM1:PC:DONE:CYCLE1':
                self.stm_state  = 'idle'
                self.start_flag = False
                self.get_logger().info('[STM1] 사이클 완료 → idle')
            elif line.startswith('STM1:PC:ERR:'):
                self.stm_state  = 'error'
                self.start_flag = False
                self.get_logger().error(line)
            elif line.startswith('STM1:PI1:ACK:'):
                self.get_logger().info(f'ACK: {line}')

    # ══════════════════════════════════════════════════════
    # 라파2 / STM32 #2 상태 수신
    # ══════════════════════════════════════════════════════
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
            elif line == 'STM2:PC:STATE:HARVEST_DONE':
                self.stm2_state = 'harvesting'
            elif line == 'STM2:PC:STATE:EJECTING':
                self.stm2_state = 'ejecting'
            elif line == 'STM2:PC:STATE:EJECT_DONE':
                self.stm2_state = 'ejecting'
            elif line == 'STM2:PC:DONE:CYCLE':
                self.stm2_state = 'idle'
                self.get_logger().info('[STM2] 사이클 완료 → idle')
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

    # ══════════════════════════════════════════════════════
    # heartbeat 수신
    # ══════════════════════════════════════════════════════
    def _on_heartbeat(self, msg: String):
        now = time.time()
        with self.state_lock:
            if msg.data == 'pi1':
                self.pi1_last_hb = now
            elif msg.data == 'pi2':
                self.pi2_last_hb = now

    # ══════════════════════════════════════════════════════
    # 
    # ══════════════════════════════════════════════════════
    def _on_link_to_pi1(self, msg: String):
        payload = msg.data.strip()
        if payload:
            self.get_logger().info(f'[InterPi] Pi2->Pi1 수신 확인: {payload}')

    def _on_link_to_pi2(self, msg: String):
        payload = msg.data.strip()
        if payload:
            self.get_logger().info(f'[InterPi] Pi1->Pi2 수신 확인: {payload}')

    # ══════════════════════════════════════════════════════
    # heartbeat + 카메라 상태 판정
    # ══════════════════════════════════════════════════════
    def _check_heartbeat(self):
        now = time.time()
        with self.state_lock:
            self.pi1_alive = (now - self.pi1_last_hb) < 3.0
            self.pi2_alive = (now - self.pi2_last_hb) < 3.0

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

            if (now - self.cam1_last_detect_ts) > 3.0:
                self.cam1_detect_label = 'none'

    # ══════════════════════════════════════════════════════
    # 외부 명령 처리
    # ══════════════════════════════════════════════════════
    def _on_command(self, msg: String):
        cmd = msg.data.strip()

        if cmd == 'EMERGENCY':
            with self.state_lock:
                self.emergency  = True
                self.start_flag = False
            self._send_pi1('PC:STM1:CMD:ESTOP')
            self._send_pi2('PC:STM2:CMD:ESTOP')
            self.get_logger().warn('EMERGENCY — Pi1·Pi2 전송')

        elif cmd == 'RESET':
            with self.state_lock:
                self.emergency   = False
                self.start_flag  = False
                self.stm_state   = 'idle'
                self.stm2_state  = 'idle'
                self.stable_cnt  = 0
                self.no_tray_cnt = 0
                self.cam1_detect_label = 'none'
                self.cam1_last_conf = 0.0
            self._send_pi1('PC:STM1:CMD:RESET')
            self._send_pi2('PC:STM2:CMD:RESET')
            self.get_logger().info('RESET — Pi1·Pi2 전송')

        elif cmd == 'START_2':
            with self.state_lock:
                if self.emergency:
                    self.get_logger().warn('긴급정지 중 — START_2 무시')
                    return
                if self.stm2_state != 'idle':
                    self.get_logger().warn(f'STM2 동작 중({self.stm2_state}) — START_2 무시')
                    return
            self._send_pi2('PI2:STM2:EVT:TRAY_PLACED')
            self.get_logger().info('STM2 수동 시작 — TRAY_PLACED 전송')

        #라파1 -> 라파2 직접 통신 트리거 (PC에서 테스트/운영 가능)
        elif cmd.startswith('PI1_TO_PI2:'):
            payload = cmd.split(':', 1)[1].strip()
            if not payload:
                self.getlogger().warn('PI1_TO_PI2 전송 실패: payload 없음')
                return
            self._send_pi1_link(payload)
            self.get_logger().info(f'InterPi 송신 요청: Pi1->Pi2 "{payload}"')

        #라파1 -> 라파2 직접 통신 트리거 (PC에서 테스트/운영 가능)
        elif cmd.startswith('PI2_TO_PI1:'):
            payload = cmd.split(':', 1)[1].strip()
            if not payload:
                self.getlogger().warn('PI2_TO_PI1 전송 실패: payload 없음')
                return
            self._send_pi2_link(payload)
            self.get_logger().info(f'InterPi 송신 요청: Pi2->Pi1 "{payload}"') 

        else:
            self.get_logger().warn(f'알 수 없는 커맨드: {cmd}')

    # ══════════════════════════════════════════════════════
    # UART 명령 송신
    # ══════════════════════════════════════════════════════
    def _send_pi1(self, text: str):
        msg = String()
        msg.data = text
        self.pub_pi1.publish(msg)

    def _send_pi2(self, text: str):
        msg = String()
        msg.data = text
        self.pub_pi2.publish(msg)

    def _send_pi1_link(self, payload: str):
        msg = String()
        msg.data = payload
        self.pub_pi1_link.publish(msg)

    def _send_pi2_link(self, payload: str):
        msg = String()
        msg.data = payload
        self.pub_pi2_link.publish(msg)
    

    # ══════════════════════════════════════════════════════
    # 모니터 상태 발행
    # ══════════════════════════════════════════════════════
    def _publish_monitor(self):
        with self.state_lock:
            msg = String()
            msg.data = (
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
        self.pub_monitor.publish(msg)


# ══════════════════════════════════════════════════════════
# YOLO 프레임 처리
# ══════════════════════════════════════════════════════════
def process_frame(node: MasterNode, frame: np.ndarray):
    #cv2.imwrite('/home/thumb/debug_frame.jpg', frame)  # 임시 저장
    #print(f'[DEBUG] 프레임 수신: {frame.shape}')  # 이 줄 추가
    results = node.model(frame, verbose=False)[0]

    best = None
    for box in results.boxes:
        cls_id = int(box.cls[0].item())
        conf = float(box.conf[0].item())

        if cls_id == TRAY_CLASS_ID:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            if best is None or conf > best[4]:
                best = (x1, y1, x2, y2, conf)

    disp = frame.copy()
    send_now = False
    now = time.time()

    with node.state_lock:
        emergency = node.emergency

    if best is None:
        with node.state_lock:
            node.stable_cnt = 0
            node.no_tray_cnt += 1
            if node.no_tray_cnt >= 3:
                node.cam1_detect_label = 'none'
                node.cam1_last_conf = 0.0

    else:
        x1, y1, x2, y2, conf = best

        if conf < MIN_CONF:
            with node.state_lock:
                node.stable_cnt = 0
                node.no_tray_cnt = 0
                node.cam1_detect_label = 'none'
                node.cam1_last_conf = conf
        
        else:
            h, w = frame.shape[:2]
            cx = (x1 + x2) / 2 / w
            cy = (y1 + y2) / 2 / h
            in_roi = (ROI_X_MIN < cx < ROI_X_MAX) and (ROI_Y_MIN < cy < ROI_Y_MAX)
            # in_roi 계산 아래에 추가
            box_w = x2 - x1
            box_h = y2 - y1
            # 트레이가 화면의 최소 25% 이상 차지할 때만 감지
            min_box_ratio = 0.25
            in_roi = in_roi and (box_w / w > min_box_ratio) and (box_h / h > min_box_ratio)


            with node.state_lock:
                if in_roi:
                    node.cam1_last_detect_ts = now
                    node.cam1_last_conf = conf
                    node.cam1_detect_label = 'tray_detected'
                    node.stable_cnt += 1
                    node.no_tray_cnt = 0

                    node.cam1_last_bbox = (x1, y1, x2, y2)
                    node.cam1_last_bbox_ts = now
                    node.cam1_last_conf_hold = conf

                    if (
                        node.stable_cnt >= STABLE_FRAMES
                        and not node.start_flag
                        and node.stm_state == 'idle'
                        and (now - node.last_tx) > COOLDOWN_SEC
                        and not node.emergency
                    ):
                        node.start_flag = True
                        node.last_tx = now
                        node.stable_cnt = 0
                        send_now = True
                else:
                    node.stable_cnt = 0
                    node.no_tray_cnt += 1

    if send_now:
        node._send_pi1('PI1:STM1:EVT:CAM1_TRAY_DETECTED')
        node.get_logger().info('트레이 감지 → CAM1_TRAY_DETECTED')

    with node.state_lock:
        start_flag = node.start_flag
        stm_state = node.stm_state
        stable_cnt = node.stable_cnt
        last_bbox = node.cam1_last_bbox
        last_bbox_ts = node.cam1_last_bbox_ts
        hold_sec = node.cam1_overlay_hold_sec
        last_conf_hold = node.cam1_last_conf_hold
        emergency = node.emergency

    # ROI 박스 표시 
    h, w = disp.shape[:2]
    cv2.rectangle(disp,
                  (int(w * ROI_X_MIN), int(h * ROI_Y_MIN)),
                  (int(w * ROI_X_MAX), int(h * ROI_Y_MAX)),
                  (255, 255, 0), 2)

    cv2.putText(disp, f"start_flag={int(start_flag)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv2.putText(disp, f"stm_state={stm_state}", (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(disp, f"stable={stable_cnt}", (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

    if last_bbox is not None and (now - last_bbox_ts) < hold_sec:
        bx1, by1, bx2, by2 = last_bbox
        cv2.rectangle(disp, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
        cv2.putText(disp, f"tray {last_conf_hold:.2f}", (bx1, by1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(disp, "tray: none", (10, 135),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    if emergency:
        cv2.putText(disp, "EMERGENCY", (10, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    try:
        frame_queue.put_nowait(disp)
    except queue.Full:
        pass


# ══════════════════════════════════════════════════════════
# 라파1 카메라 스트림 수신 루프
# ════════════════════════════════════════  ══════════════════
def video_receive_loop(node: MasterNode):
    # 캘리브레이션 로드
    calib = np.load(CALIB_PATH)
    K = calib['camera_matrix']
    dist = calib['dist_coeffs']   
    # 리맵 준비 (첫 프레임 받은 뒤 초기화)
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

            data = b''
            payload_size = struct.calcsize('>I')

            while rclpy.ok():
                while len(data) < payload_size:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionError('Pi1 stream disconnected')
                    data += packet

                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack('>I', packed_msg_size)[0]

                while len(data) < msg_size:
                    packet = conn.recv(4096)
                    if not packet:
                        raise ConnectionError('Pi1 stream disconnected')
                    data += packet

                frame_data = data[:msg_size]
                data = data[msg_size:]

                arr = np.frombuffer(frame_data, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue          

                # 캘리브레이션 맵 초기화 (최초 1회)
                if map1 is None:
                    h, w = frame.shape[:2]
                    newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0, (w, h))
                    map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (w, h), cv2.CV_16SC2)
                    print('[Calib] 왜곡 보정 맵 초기화 완료')
                # undistort 적용
                frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
                with node.state_lock:
                    node.cam1_last_frame_ts = time.time()

                #print(f'[DEBUG] 프레임 디코딩 성공: {frame.shape}')  # 이 줄 추가
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

    t = threading.Thread(target=video_receive_loop, args=(node,), daemon=True)
    t.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            try:
                frame = frame_queue.get_nowait()
                cv2.imshow("Pi1 Camera", frame)
                cv2.waitKey(1)
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()