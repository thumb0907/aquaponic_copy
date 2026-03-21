#!/usr/bin/env python3
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

        self.start_flag  = False      # CAM1 이벤트를 이미 보냈는지
        self.stm_state   = 'idle'
        self.emergency   = False
        self.stable_cnt  = 0
        self.no_tray_cnt = 0
        self.last_tx     = 0.0
        self.pi1_alive   = False
        self.pi1_last_hb = 0.0

        self.pub_pi1     = self.create_publisher(String, '/pi1/uart_cmd', 10)
        self.pub_monitor = self.create_publisher(String, '/monitor/state', 10)

        self.create_subscription(String, '/pi1/uart_response', self._on_uart1, 10)
        self.create_subscription(String, '/system/heartbeat', self._on_heartbeat, 10)
        self.create_subscription(String, '/system/command', self._on_command, 10)

        self.create_timer(1.0, self._check_heartbeat)
        self.create_timer(0.5, self._publish_monitor)

        self.get_logger().info('Master 노드 시작')

    def _on_uart1(self, msg: String):
        line = msg.data.strip()
        self.get_logger().info(f'STM32: "{line}"')

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
            self.stm_state = 'error'
            self.start_flag = False
            self.get_logger().warn('STM ESTOP 상태 진입')

        elif line == 'STM1:PC:DONE:CYCLE1':
            self.stm_state  = 'idle'
            self.start_flag = False
            self.get_logger().info('사이클 완료 → idle')

        elif line.startswith('STM1:PC:ERR:'):
            self.stm_state = 'error'
            self.get_logger().error(line)

        elif line.startswith('STM1:PI1:ACK:'):
            self.get_logger().info(f'ACK 수신: {line}')

    def process_frame(self, frame, model):
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

        best = self._pick_best(res)
        detected = best is not None and best[4] >= MIN_CONF
        conf = best[4] if best else 0.0

        if detected:
            self.stable_cnt += 1
            self.no_tray_cnt = 0
        else:
            self.stable_cnt = 0
            self.no_tray_cnt += 1

        if self.no_tray_cnt >= 5 and self.stm_state == 'idle':
            self.start_flag = False

        now = time.time()
        if (
            self.stable_cnt >= STABLE_FRAMES
            and not self.start_flag
            and self.stm_state == 'idle'
            and (now - self.last_tx) >= COOLDOWN_SEC
            and self.pi1_alive
        ):
            cmd = 'PI1:STM1:EVT:CAM1_TRAY_DETECTED'
            self.get_logger().info(
                f'트레이 감지 (conf={conf:.2f}) → {cmd} 전송'
            )
            self._send(cmd)

            self.start_flag = True
            self.stm_state  = 'homing'
            self.last_tx    = now
            self.stable_cnt = 0

        return best

    def _pick_best(self, result):
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return None

        best = None
        best_conf = -1.0

        for i in range(len(boxes)):
            cls = int(boxes.cls[i].item())
            conf = float(boxes.conf[i].item())
            if cls != TRAY_CLASS_ID:
                continue

            if conf > best_conf:
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                best = (int(x1), int(y1), int(x2), int(y2), conf)
                best_conf = conf

        return best

    def _on_heartbeat(self, msg: String):
        if msg.data == 'pi1':
            self.pi1_last_hb = time.time()
            self.pi1_alive = True

    def _check_heartbeat(self):
        alive = (time.time() - self.pi1_last_hb) < 3.0
        if self.pi1_alive != alive:
            self.pi1_alive = alive
            self.get_logger().info(f'Pi1 {"연결" if alive else "끊김"}')

    def _on_command(self, msg: String):
        data = msg.data.strip()

        if 'EMERGENCY' in data:
            self.emergency  = True
            self.start_flag = False
            self.stm_state  = 'error'   # idle 말고 error
            self._send('PC:STM1:CMD:ESTOP')

        elif 'RESET' in data:
            self.emergency   = False
            self.start_flag  = False
            self.stm_state   = 'idle'
            self.stable_cnt  = 0
            self.no_tray_cnt = 0
            self._send('PC:STM1:CMD:RESET')
            self.get_logger().info('RESET 전송')

    def _send(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.pub_pi1.publish(msg)

    def _publish_monitor(self):
        msg = String()
        msg.data = (
            f'start_flag:{self.start_flag},'
            f'stm_state:{self.stm_state},'
            f'pi1_alive:{self.pi1_alive},'
            f'emergency:{self.emergency}'
        )
        self.pub_monitor.publish(msg)


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
                buf = buf[4:]

                while len(buf) < size:
                    chunk = conn.recv(4096)
                    if not chunk:
                        raise ConnectionResetError
                    buf += chunk

                jpeg = buf[:size]
                buf = buf[size:]

                frame = cv2.imdecode(
                    np.frombuffer(jpeg, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )
                if frame is None:
                    continue

                best = node.process_frame(frame, model)

                if best is not None:
                    x1, y1, x2, y2, conf = best
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        frame, f'conf={conf:.2f}',
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                    )

                cv2.putText(
                    frame,
                    f'state={node.stm_state} pi1={"OK" if node.pi1_alive else "NG"}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
                )

                cv2.imshow('PC - Pi1 cam', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

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
    node = MasterNode()

    print('[YOLO] 모델 로딩 중...')
    model = YOLO(MODEL_PATH)
    print('[YOLO] 로딩 완료')

    threading.Thread(target=spin_thread, args=(node,), daemon=True).start()
    threading.Thread(target=video_receive_loop, args=(node, model), daemon=True).start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
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