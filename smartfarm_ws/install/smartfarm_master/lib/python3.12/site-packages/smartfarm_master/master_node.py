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
  /pi2/uart_cmd        라파2 → STM2로 전달할 명령
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


# ── 설정값 ────────────────────────────────────────────────────
MODEL_PATH    = '/home/thumb/models/best.pt'  # YOLO 모델 경로
STREAM_PORT   = 5000                           # 라파1 카메라 스트림 수신 포트
TRAY_CLASS_ID = 0                              # YOLO에서 트레이로 인식하는 클래스 번호
MIN_CONF      = 0.60                           # 트레이로 판정하는 최소 신뢰도
STABLE_FRAMES = 3                              # 연속 몇 프레임 감지돼야 진짜로 인정
COOLDOWN_SEC  = 2.0                            # 한 번 전송 후 재전송 대기 시간(초)


class MasterNode(Node):

    def __init__(self):
        super().__init__('master_node')

        # 여러 스레드(ROS 콜백, 카메라 루프)가 동시에 변수를 건드리므로 락 사용
        self.state_lock = threading.Lock()

        # ── 라파1 / STM32 #1 상태 (기존 그대로) ──────────
        self.start_flag  = False   # CAM1이 트레이를 감지해서 STM1에 명령 보낸 상태
        self.stm_state   = 'idle'  # STM1 현재 동작 상태
        self.stable_cnt  = 0       # 트레이가 연속으로 감지된 프레임 수
        self.no_tray_cnt = 0       # 트레이가 연속으로 없었던 프레임 수
        self.last_tx     = 0.0     # 마지막으로 STM1에 명령 보낸 시각
        self.pi1_alive   = False   # 라파1 연결 상태
        self.pi1_last_hb = 0.0     # 라파1 마지막 하트비트 수신 시각

        # ── 라파2 / STM32 #2 상태 (신규) ─────────────────
        self.stm2_state  = 'idle'  # STM2 현재 동작 상태
        self.pi2_alive   = False   # 라파2 연결 상태
        self.pi2_last_hb = 0.0     # 라파2 마지막 하트비트 수신 시각

        # ── 공통 ─────────────────────────────────────────
        self.emergency   = False   # 긴급정지 상태 (True면 모든 동작 차단)

        # ── 발행 토픽 ─────────────────────────────────────
        self.pub_pi1     = self.create_publisher(String, '/pi1/uart_cmd',  10)
        self.pub_pi2     = self.create_publisher(String, '/pi2/uart_cmd',  10)
        self.pub_monitor = self.create_publisher(String, '/monitor/state', 10)

        # ── 구독 토픽 ─────────────────────────────────────
        self.create_subscription(String, '/pi1/uart_response', self._on_uart1,     10)
        self.create_subscription(String, '/pi2/uart_response', self._on_uart2,     10)
        self.create_subscription(String, '/system/heartbeat',  self._on_heartbeat, 10)
        self.create_subscription(String, '/system/command',    self._on_command,   10)

        # ── 타이머 ───────────────────────────────────────
        self.create_timer(1.0, self._check_heartbeat)  # 1초마다 라파 연결 확인
        self.create_timer(0.5, self._publish_monitor)  # 0.5초마다 모니터에 상태 전송

        self.get_logger().info('Master 노드 시작')

    # ══════════════════════════════════════════════════════
    # 라파1 / STM32 #1  ─  기존 코드 그대로
    # ══════════════════════════════════════════════════════
    # STM32 #1이 라파1을 통해 보내는 상태 문자열 수신
    # 형식: "STM1:PC:STATE:xxx" 또는 "STM1:PC:DONE:xxx" 등
    def _on_uart1(self, msg: String):
        line = msg.data.strip()
        self.get_logger().info(f'[STM1] {line}')

        with self.state_lock:
            if line == 'STM1:PC:STATE:HOMING':
                self.stm_state = 'homing'          # 직교로봇 홈잉 중
            elif line == 'STM1:PC:DONE:HOMING':
                pass                               # 홈잉 완료 (별도 처리 없음)
            elif line == 'STM1:PC:STATE:RUN_CONVEYOR1':
                self.stm_state = 'running'         # 컨베이어 구동 중
            elif line == 'STM1:PC:STATE:SEEDING':
                self.stm_state = 'seeding'         # 파종 중
            elif line == 'STM1:PC:STATE:EJECTING':
                self.stm_state = 'ejecting'        # 트레이 배출 중
            elif line == 'STM1:PC:STATE:WAIT_SCARA_PICK':
                self.stm_state = 'waiting_scara'   # 스카라 픽업 대기 중
            elif line == 'STM1:PC:STATE:SCARA_PICK_STARTED':
                self.stm_state = 'waiting_scara'   # 스카라가 픽업 시작함
            elif line == 'STM1:PC:STATE:ESTOP':
                self.stm_state  = 'error'
                self.start_flag = False
                self.get_logger().warn('[STM1] ESTOP')
            elif line == 'STM1:PC:DONE:CYCLE1':
                self.stm_state  = 'idle'           # 한 사이클 완료, 대기 상태로 복귀
                self.start_flag = False
                self.get_logger().info('[STM1] 사이클 완료 → idle')
            elif line.startswith('STM1:PC:ERR:'):
                self.stm_state  = 'error'          # 타임아웃 등 에러 발생
                self.start_flag = False
                self.get_logger().error(line)
            elif line.startswith('STM1:PI1:ACK:'):
                self.get_logger().info(f'ACK: {line}')  # 명령 수신 확인

    # ══════════════════════════════════════════════════════
    # 라파2 / STM32 #2  ─  신규
    # ══════════════════════════════════════════════════════
    # STM32 #2가 라파2를 통해 보내는 상태 문자열 수신
    # 형식: "STM2:PC:STATE:xxx" 또는 "STM2:PC:DONE:xxx" 등
    def _on_uart2(self, msg: String):
        line = msg.data.strip()
        self.get_logger().info(f'[STM2] {line}')

        with self.state_lock:
            if   line == 'STM2:PC:STATE:CONVEY_RUN':   self.stm2_state = 'convey_run'    # 컨베이어 구동 중
            elif line == 'STM2:PC:STATE:IR_DETECTED':  self.stm2_state = 'ir_detected'   # IR 센서 트레이 감지
            elif line == 'STM2:PC:STATE:Z_FIX':        self.stm2_state = 'z_fix'         # Z축 트레이 고정 완료
            elif line == 'STM2:PC:STATE:HARVESTING':   self.stm2_state = 'harvesting'    # 스카라+매니퓰 수확 중
            elif line == 'STM2:PC:STATE:HARVEST_DONE': self.stm2_state = 'harvest_done'  # 수확 완료
            elif line == 'STM2:PC:STATE:EJECTING':     self.stm2_state = 'ejecting'      # 수확된 트레이 배출 중
            elif line == 'STM2:PC:STATE:EJECT_DONE':   self.stm2_state = 'eject_done'    # 배출 완료
            elif line == 'STM2:PC:STATE:IDLE':         self.stm2_state = 'idle'          # 대기 상태
            elif line == 'STM2:PC:DONE:CYCLE2':
                self.stm2_state = 'idle'                                                  # 한 사이클 완료, 대기 상태로 복귀
                self.get_logger().info('[STM2] 사이클 완료 → idle')
            elif line == 'STM2:PC:STATE:ESTOP':
                self.stm2_state = 'error'
                self.emergency  = True              # 긴급정지 → 이후 모든 동작 차단
                self.get_logger().error('[STM2] ESTOP')
            elif line.startswith('STM2:PC:ERR:'):
                self.stm2_state = 'error'
                self.emergency  = True              # 타임아웃 등 에러도 긴급정지로 처리
                self.get_logger().error(line)
            elif line == 'STM2:PC:STATE:RESET_DONE':
                self.stm2_state = 'idle'            # 리셋 완료, 정상 대기 상태로 복귀
                self.get_logger().info('[STM2] 리셋 완료 → idle')
            elif line.startswith('STM2:PI2:ACK:'):
                self.get_logger().info(f'ACK: {line}')  # 명령 수신 확인

    # ══════════════════════════════════════════════════════
    # YOLO 카메라 처리  ─  기존 코드 그대로
    # ══════════════════════════════════════════════════════
    # 라파1에서 받은 카메라 프레임에서 트레이를 감지하고
    # 조건이 맞으면 STM1에 파종 시작 명령 전송
    def process_frame(self, frame, model):
        # 긴급정지 중이면 카메라 처리 자체를 중단
        with self.state_lock:
            if self.emergency:
                return None

        # YOLO 추론 — 트레이 클래스만 탐지
        res = model.predict(
            source=frame,
            imgsz=416,
            conf=0.25,
            iou=0.7,
            device='cpu',
            classes=[TRAY_CLASS_ID],
            verbose=False
        )[0]

        best     = self._pick_best(res)               # 가장 신뢰도 높은 박스 선택
        detected = best is not None and best[4] >= MIN_CONF  # 기준 신뢰도 이상이면 감지로 판정
        conf     = best[4] if best else 0.0
        now      = time.time()
        send_cmd = None

        with self.state_lock:
            if detected:
                self.stable_cnt  += 1   # 감지 → 연속 감지 카운트 증가
                self.no_tray_cnt  = 0
            else:
                self.stable_cnt   = 0   # 미감지 → 연속 감지 카운트 초기화
                self.no_tray_cnt += 1

            # 트레이가 5프레임 연속 없으면 start_flag 해제 (오감지 방지)
            if self.no_tray_cnt >= 5 and self.stm_state == 'idle':
                self.start_flag = False

            # 아래 조건이 모두 충족될 때만 STM1에 파종 시작 명령 전송
            #   - STABLE_FRAMES 이상 연속 감지
            #   - 이미 명령을 보낸 상태가 아님 (start_flag=False)
            #   - STM1이 대기 상태 (idle)
            #   - 마지막 전송으로부터 COOLDOWN_SEC 이상 경과
            #   - 라파1이 연결되어 있음
            if (
                self.stable_cnt >= STABLE_FRAMES
                and not self.start_flag
                and self.stm_state == 'idle'
                and (now - self.last_tx) >= COOLDOWN_SEC
                and self.pi1_alive
            ):
                send_cmd        = 'PI1:STM1:EVT:CAM1_TRAY_DETECTED'
                self.start_flag = True   # 중복 전송 방지 플래그 세팅
                self.stm_state  = 'homing'
                self.last_tx    = now
                self.stable_cnt = 0

        if send_cmd is not None:
            self.get_logger().info(f'트레이 감지 (conf={conf:.2f}) → {send_cmd}')
            self._send_pi1(send_cmd)

        return best

    # 감지된 박스 중 트레이 클래스이면서 신뢰도가 가장 높은 것 하나 반환
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
    # 라파들이 1초마다 보내는 'pi1' / 'pi2' 문자열 수신
    # → 마지막 수신 시각 갱신
    def _on_heartbeat(self, msg: String):
        now = time.time()
        with self.state_lock:
            if msg.data == 'pi1':
                self.pi1_last_hb = now
                self.pi1_alive   = True
            elif msg.data == 'pi2':
                self.pi2_last_hb = now
                self.pi2_alive   = True

    # 1초마다 호출 — 마지막 하트비트로부터 3초 초과 시 연결 끊김으로 판정
    def _check_heartbeat(self):
        now = time.time()
        with self.state_lock:
            p1 = (now - self.pi1_last_hb) < 3.0
            p2 = (now - self.pi2_last_hb) < 3.0
            if self.pi1_alive != p1:   # 상태가 바뀔 때만 로그 출력
                self.pi1_alive = p1
                self.get_logger().info(f'Pi1 {"연결" if p1 else "끊김"}')
            if self.pi2_alive != p2:
                self.pi2_alive = p2
                self.get_logger().info(f'Pi2 {"연결" if p2 else "끊김"}')

    # ══════════════════════════════════════════════════════
    # /system/command 처리
    # ══════════════════════════════════════════════════════
    # 터미널에서 ros2 topic pub 으로 명령을 보내면 여기서 처리
    # 예) ros2 topic pub /system/command std_msgs/msg/String "data: 'EMERGENCY'"
    def _on_command(self, msg: String):
        cmd = msg.data.strip()

        # 전체 긴급정지 — Pi1·Pi2 양쪽에 ESTOP 전송
        if cmd == 'EMERGENCY':
            with self.state_lock:
                self.emergency  = True
                self.start_flag = False
                self.stm_state  = 'error'
                self.stm2_state = 'error'
            self._send_pi1('PC:STM1:CMD:ESTOP')
            self._send_pi2('PC:STM2:CMD:ESTOP')
            self.get_logger().error('긴급정지 — Pi1·Pi2 E1 전송')

        # 전체 리셋 — 긴급정지 해제 후 idle 상태로 복귀
        elif cmd == 'RESET':
            with self.state_lock:
                self.emergency   = False
                self.start_flag  = False
                self.stm_state   = 'idle'
                self.stm2_state  = 'idle'
                self.stable_cnt  = 0
                self.no_tray_cnt = 0
            self._send_pi1('PC:STM1:CMD:RESET')
            self._send_pi2('PC:STM2:CMD:RESET')
            self.get_logger().info('RESET — Pi1·Pi2 전송')

        # STM2 수동 시작 (테스트용) — 스카라 없이 PC에서 직접 두번째 컨베이어 시작
        elif cmd == 'START_2':
            with self.state_lock:
                if self.emergency:
                    self.get_logger().warn('긴급정지 중 — START_2 무시')
                    return
                if self.stm2_state != 'idle':
                    self.get_logger().warn(f'STM2 동작 중({self.stm2_state}) — START_2 무시')
                    return
            self._send_pi2('PI2:STM2:EVT:TRAY_PLACED')   # 트레이 올려놓음 이벤트 전송
            self.get_logger().info('STM2 수동 시작 — TRAY_PLACED 전송')

        else:
            self.get_logger().warn(f'알 수 없는 커맨드: {cmd}')

    # ══════════════════════════════════════════════════════
    # 모니터 상태 발행
    # ══════════════════════════════════════════════════════
    # 0.5초마다 현재 상태를 /monitor/state 토픽으로 전송
    # monitor_node가 이걸 받아서 터미널 대시보드에 표시
    def _publish_monitor(self):
        with self.state_lock:
            msg      = String()
            msg.data = (
                f'start_flag:{self.start_flag},'    # CAM1 감지 후 명령 전송 여부
                f'stm_state:{self.stm_state},'      # STM1 현재 상태
                f'stm2_state:{self.stm2_state},'    # STM2 현재 상태
                f'pi1_alive:{self.pi1_alive},'      # 라파1 연결 여부
                f'pi2_alive:{self.pi2_alive},'      # 라파2 연결 여부
                f'emergency:{self.emergency}'        # 긴급정지 여부
            )
        self.pub_monitor.publish(msg)

    # ══════════════════════════════════════════════════════
    # 송신 헬퍼
    # ══════════════════════════════════════════════════════
    def _send_pi1(self, cmd: str):
        # /pi1/uart_cmd 토픽으로 발행 → pi1_node가 받아서 STM1에 UART 전송
        msg = String(); msg.data = cmd
        self.pub_pi1.publish(msg)

    def _send_pi2(self, cmd: str):
        # /pi2/uart_cmd 토픽으로 발행 → pi2_node가 받아서 STM2에 UART 전송
        msg = String(); msg.data = cmd
        self.pub_pi2.publish(msg)


# ── 비디오 수신 루프  ─  기존 코드 그대로 ──────────────────
# 라파1이 TCP 소켓으로 보내는 카메라 영상을 수신
# 프레임마다 YOLO 추론 후 트레이 감지 여부 판단
# 형식: [4바이트 길이(빅엔디안)] + [JPEG 데이터]
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
                # 4바이트 길이 헤더 수신
                while len(buf) < 4:
                    chunk = conn.recv(4096)
                    if not chunk:
                        raise ConnectionResetError
                    buf += chunk

                size = struct.unpack('>I', buf[:4])[0]  # 빅엔디안 4바이트 → 정수
                buf  = buf[4:]

                # 본문(JPEG) 수신
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

                # YOLO 추론 + 트레이 감지 판정
                best = node.process_frame(frame, model)

                # 화면에 표시할 상태값을 락 안에서 한 번에 읽어옴
                with node.state_lock:
                    s1   = node.stm_state
                    s2   = node.stm2_state
                    p1ok = node.pi1_alive
                    p2ok = node.pi2_alive

                # 감지된 경우 박스와 신뢰도 표시
                if best is not None:
                    x1, y1, x2, y2, conf = best
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f'conf={conf:.2f}',
                                (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # 화면 상단에 현재 상태 오버레이
                cv2.putText(
                    frame,
                    (f'STM1={s1}  STM2={s2}'
                     f'  Pi1={"OK" if p1ok else "NG"}'
                     f'  Pi2={"OK" if p2ok else "NG"}'),
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2
                )

                cv2.imshow('PC - Pi1 cam', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):  # q 키로 종료
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


# ROS2 spin을 별도 스레드로 실행 (메인 스레드는 카메라 루프가 점유)
def spin_thread(node: MasterNode):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    rclpy.init()
    node  = MasterNode()
    model = YOLO(MODEL_PATH)

    # spin은 별도 스레드 → 메인 스레드에서 카메라 루프 실행
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