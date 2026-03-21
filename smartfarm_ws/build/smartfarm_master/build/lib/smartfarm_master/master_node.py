#!/usr/bin/env python3
"""
master_node.py  ─  PC 마스터 노드
=========================================================
라파1(첫번째 컨베이어) + 라파2(두번째 컨베이어) 통합 관리.
라파3(수경재배실)은 추후 추가 예정.

토픽:
  구독
    /pi1/uart_response   STM1 상태 (기존 코드 그대로)
    /pi2/uart_response   STM2 상태 (신규)
    /system/heartbeat    'pi1' / 'pi2' 생존 신호
    /system/command      EMERGENCY / RESET / START_2

  발행
    /pi1/uart_cmd        STM1 명령 (기존)
    /pi2/uart_cmd        STM2 명령 (신규)
    /monitor/state       모니터 상태 문자열
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


MODEL_PATH    = '/home/thumb/models/best.pt'
STREAM_PORT   = 5000
TRAY_CLASS_ID = 0
MIN_CONF      = 0.60
STABLE_FRAMES = 3
COOLDOWN_SEC  = 2.0


class MasterNode(Node):

    def __init__(self):
        super().__init__('master_node')

        self.state_lock = threading.Lock()

        # ── 라파1 / STM32 #1 상태 (기존 그대로) ──────────
        self.start_flag  = False
        self.stm_state   = 'idle'
        self.stable_cnt  = 0
        self.no_tray_cnt = 0
        self.last_tx     = 0.0
        self.pi1_alive   = False
        self.pi1_last_hb = 0.0

        # ── 라파2 / STM32 #2 상태 (신규) ─────────────────
        self.stm2_state  = 'idle'
        self.pi2_alive   = False
        self.pi2_last_hb = 0.0

        # ── 공통 ─────────────────────────────────────────
        self.emergency   = False

        # ── 발행 ─────────────────────────────────────────
        self.pub_pi1     = self.create_publisher(String, '/pi1/uart_cmd',  10)
        self.pub_pi2     = self.create_publisher(String, '/pi2/uart_cmd',  10)
        self.pub_monitor = self.create_publisher(String, '/monitor/state', 10)

        # ── 구독 ─────────────────────────────────────────
        self.create_subscription(String, '/pi1/uart_response', self._on_uart1,     10)
        self.create_subscription(String, '/pi2/uart_response', self._on_uart2,     10)
        self.create_subscription(String, '/system/heartbeat',  self._on_heartbeat, 10)
        self.create_subscription(String, '/system/command',    self._on_command,   10)

        # ── 타이머 ───────────────────────────────────────
        self.create_timer(1.0, self._check_heartbeat)
        self.create_timer(0.5, self._publish_monitor)

        self.get_logger().info('Master 노드 시작')

    # ══════════════════════════════════════════════════════
    # 라파1 / STM32 #1  ─  기존 코드 그대로
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
    # 라파2 / STM32 #2  ─  신규
    # ══════════════════════════════════════════════════════
    def _on_uart2(self, msg: String):
        """pi2_node가 STM2:PC:STATE:… 형식으로 변환해서 전달."""
        line = msg.data.strip()
        self.get_logger().info(f'[STM2] {line}')

        with self.state_lock:
            if   line == 'STM2:PC:STATE:CONVEY_RUN':   self.stm2_state = 'convey_run'
            elif line == 'STM2:PC:STATE:IR_DETECTED':  self.stm2_state = 'ir_detected'
            elif line == 'STM2:PC:STATE:Z_FIX':        self.stm2_state = 'z_fix'
            elif line == 'STM2:PC:STATE:HARVESTING':   self.stm2_state = 'harvesting'
            elif line == 'STM2:PC:STATE:HARVEST_DONE': self.stm2_state = 'harvest_done'
            elif line == 'STM2:PC:STATE:EJECTING':     self.stm2_state = 'ejecting'
            elif line == 'STM2:PC:STATE:EJECT_DONE':   self.stm2_state = 'eject_done'
            elif line == 'STM2:PC:DONE:CYCLE':
                self.stm2_state = 'idle'
                self.get_logger().info('[STM2] 사이클 완료 → idle')
            elif line == 'STM2:PC:STATE:ESTOP':
                self.stm2_state = 'error'
                self.emergency  = True
                self.get_logger().error('[STM2] ESTOP')
            elif line == 'STM2:PC:ERR:TIMEOUT':
                self.stm2_state = 'error'
                self.emergency  = True
                self.get_logger().error('[STM2] 타임아웃 → ESTOP')
            elif line == 'STM2:PC:STATE:RESET_DONE':
                self.stm2_state = 'idle'
                self.get_logger().info('[STM2] 리셋 완료 → idle')

    # ══════════════════════════════════════════════════════
    # YOLO 카메라 처리  ─  기존 코드 그대로
    # ══════════════════════════════════════════════════════
    def process_frame(self, frame, model):
        with self.state_lock:
            if self.emergency:
                return None

        res = model.predict(
            source=frame,
            imgsz=416,
            conf=0.25,
            iou=0.7,
            device='cpu',
            classes=[TRAY_CLASS_ID],
            verbose=False
        )[0]

        best     = self._pick_best(res)
        detected = best is not None and best[4] >= MIN_CONF
        conf     = best[4] if best else 0.0
        now      = time.time()
        send_cmd = None

        with self.state_lock:
            if detected:
                self.stable_cnt  += 1
                self.no_tray_cnt  = 0
            else:
                self.stable_cnt   = 0
                self.no_tray_cnt += 1

            if self.no_tray_cnt >= 5 and self.stm_state == 'idle':
                self.start_flag = False

            if (
                self.stable_cnt >= STABLE_FRAMES
                and not self.start_flag
                and self.stm_state == 'idle'
                and (now - self.last_tx) >= COOLDOWN_SEC
                and self.pi1_alive
            ):
                send_cmd        = 'PI1:STM1:EVT:CAM1_TRAY_DETECTED'
                self.start_flag = True
                self.stm_state  = 'homing'
                self.last_tx    = now
                self.stable_cnt = 0

        if send_cmd is not None:
            self.get_logger().info(f'트레이 감지 (conf={conf:.2f}) → {send_cmd}')
            self._send_pi1(send_cmd)

        return best

    def _pick_best(self, result):
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return None
        best, best_conf = None, -1.0
        for i in range(len(boxes)):
            cls  = int(boxes.cls[i].item())
            conf = float(boxes.conf[i].item())
            if cls != TRAY_CLASS_ID:
                continue
            if conf > best_conf:
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                best = (int(x1), int(y1), int(x2), int(y2), conf)
                best_conf = conf
        return best

    # ══════════════════════════════════════════════════════
    # 하트비트
    # ══════════════════════════════════════════════════════
    def _on_heartbeat(self, msg: String):
        now = time.time()
        with self.state_lock:
            if msg.data == 'pi1':
                self.pi1_last_hb = now
                self.pi1_alive   = True
            elif msg.data == 'pi2':
                self.pi2_last_hb = now
                self.pi2_alive   = True

    def _check_heartbeat(self):
        now = time.time()
        with self.state_lock:
            p1 = (now - self.pi1_last_hb) < 3.0
            p2 = (now - self.pi2_last_hb) < 3.0
            if self.pi1_alive != p1:
                self.pi1_alive = p1
                self.get_logger().info(f'Pi1 {"연결" if p1 else "끊김"}')
            if self.pi2_alive != p2:
                self.pi2_alive = p2
                self.get_logger().info(f'Pi2 {"연결" if p2 else "끊김"}')

    # ══════════════════════════════════════════════════════
    # /system/command 처리
    # ══════════════════════════════════════════════════════
    def _on_command(self, msg: String):
        cmd = msg.data.strip()

        # 전체 긴급정지
        if cmd == 'EMERGENCY':
            with self.state_lock:
                self.emergency  = True
                self.start_flag = False
                self.stm_state  = 'error'
                self.stm2_state = 'error'
            self._send_pi1('PC:STM1:CMD:ESTOP')
            self._send_pi2('E1')
            self.get_logger().error('긴급정지 — Pi1·Pi2 E1 전송')

        # 전체 리셋
        elif cmd == 'RESET':
            with self.state_lock:
                self.emergency   = False
                self.start_flag  = False
                self.stm_state   = 'idle'
                self.stm2_state  = 'idle'
                self.stable_cnt  = 0
                self.no_tray_cnt = 0
            self._send_pi1('PC:STM1:CMD:RESET')
            self._send_pi2('R1')
            self.get_logger().info('RESET — Pi1·Pi2 전송')

        # STM2 수동 시작 (테스트용)
        elif cmd == 'START_2':
            with self.state_lock:
                if self.emergency:
                    self.get_logger().warn('긴급정지 중 — START_2 무시')
                    return
                if self.stm2_state != 'idle':
                    self.get_logger().warn(f'STM2 동작 중({self.stm2_state}) — START_2 무시')
                    return
            self._send_pi2('S1')
            self.get_logger().info('STM2 수동 시작 — S1 전송')

        else:
            self.get_logger().warn(f'알 수 없는 커맨드: {cmd}')

    # ══════════════════════════════════════════════════════
    # 모니터 상태 발행
    # ══════════════════════════════════════════════════════
    def _publish_monitor(self):
        with self.state_lock:
            msg      = String()
            msg.data = (
                f'start_flag:{self.start_flag},'
                f'stm_state:{self.stm_state},'
                f'stm2_state:{self.stm2_state},'
                f'pi1_alive:{self.pi1_alive},'
                f'pi2_alive:{self.pi2_alive},'
                f'emergency:{self.emergency}'
            )
        self.pub_monitor.publish(msg)

    # ══════════════════════════════════════════════════════
    # 송신 헬퍼
    # ══════════════════════════════════════════════════════
    def _send_pi1(self, cmd: str):
        msg = String(); msg.data = cmd
        self.pub_pi1.publish(msg)

    def _send_pi2(self, cmd: str):
        msg = String(); msg.data = cmd
        self.pub_pi2.publish(msg)


# ── 비디오 수신 루프  ─  기존 코드 그대로 ──────────────────
def video_receive_loop(node: MasterNode, model):
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
            buf = b''

            while rclpy.ok():
                while len(buf) < 4:
                    chunk = conn.recv(4096)
                    if not chunk:
                        raise ConnectionResetError
                    buf += chunk

                size = struct.unpack('>I', buf[:4])[0]
                buf  = buf[4:]

                while len(buf) < size:
                    chunk = conn.recv(4096)
                    if not chunk:
                        raise ConnectionResetError
                    buf += chunk

                jpeg  = buf[:size]
                buf   = buf[size:]
                frame = cv2.imdecode(
                    np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if frame is None:
                    continue

                best = node.process_frame(frame, model)

                with node.state_lock:
                    s1   = node.stm_state
                    s2   = node.stm2_state
                    p1ok = node.pi1_alive
                    p2ok = node.pi2_alive

                if best is not None:
                    x1, y1, x2, y2, conf = best
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f'conf={conf:.2f}',
                                (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.putText(
                    frame,
                    (f'STM1={s1}  STM2={s2}'
                     f'  Pi1={"OK" if p1ok else "NG"}'
                     f'  Pi2={"OK" if p2ok else "NG"}'),
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2
                )

                cv2.imshow('PC - Pi1 cam', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return

        except Exception as e:
            print(f'[Stream] 연결 끊김: {e}')
        finally:
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass

    cv2.destroyAllWindows()
    server.close()


def spin_thread(node: MasterNode):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    rclpy.init()
    node  = MasterNode()
    model = YOLO(MODEL_PATH)

    threading.Thread(target=spin_thread, args=(node,), daemon=True).start()

    try:
        video_receive_loop(node, model)
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
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