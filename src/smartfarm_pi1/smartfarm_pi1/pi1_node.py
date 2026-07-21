#!/usr/bin/env python3
"""
pi1_node.py  ─  라파1 노드 (ROS2)
=========================================================
PC ←(ROS2)→ Pi1 ←(UART 바이너리)→ STM32 #1

[프레임 구조]  SOF(1) | ID(1) | LEN(1) | DATA(LEN) | CHK(1)
  SOF = 0xAA
  CHK = (ID + LEN + sum(DATA)) & 0xFF

[ROS2 토픽]
  구독: /pi1/uart_cmd       PC → STM1 명령 (바이너리)
  발행: /pi1/uart_response  STM1 수신 내용 → PC
        /system/heartbeat   1초 생존 신호 ('pi1')

역할:
  1. 카메라 영상 → PC로 TCP 스트리밍
  2. PC 명령(바이너리) → STM1 UART 전달
  3. STM1 응답(바이너리) → 파싱 → PC로 ROS2 전송

self.flags 같은 전역 플래그 저장소를 Pi1에 추가하면 안 됨
Pi1은 어디까지나 프레임 → 문자열 변환 + 전달까지만 해야 됨
"""
from __future__ import annotations

import time
import threading
import socket
import struct

import cv2
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


# ── 설정 ──────────────────────────────────────
#PC_IP       = '192.168.0.183'
PC_IP = '10.10.10.1'
STREAM_PORT = 5000
SERIAL_PORT = '/dev/stm1'
BAUD        = 115200
CAM_INDEX = '/dev/video0'
CAM_WIDTH   = 640
CAM_HEIGHT  = 480
JPEG_QUALITY = 50
#FRAME_SKIP = 5
CONVEYOR_SEND_INTERVAL = 0.5    # 컨베이어는 유지 (빠른 반응 불필요)
NURSERY_SEND_INTERVAL  = 0.5   # 발아실은 5fps (5 * 0.25초  내 판정 가능)
# 추가 카메라: 발아실 감지용
NURSERY_LEFT_CAM_INDEX = '/dev/video2'
NURSERY_LEFT_STREAM_PORT = 5001

NURSERY_RIGHT_CAM_INDEX = '/dev/video4'
NURSERY_RIGHT_STREAM_PORT = 5002

# ── 프로토콜 상수 (comm.h와 동일) ─────────────
SOF          = 0xAA
MAX_DATA_LEN = 16

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

PID_STATE = 0x20
PID_DONE  = 0x21
PID_ERR   = 0x22

STATE_STR = {
    0x01: 'IDLE',
    0x02: 'HOMING',
    0x03: 'RUN_CONVEYOR1',
    0x04: 'SEEDING',
    0x05: 'EJECTING',
    0x06: 'WAIT_SCARA_PICK',
    0x07: 'ESTOP',
    0x08: 'PICKING',
}
DONE_STR = {
    0x01: 'HOMING',
    0x02: 'CYCLE1',
}
ERR_STR = {
    0x01: 'HOMING_TIMEOUT',
}
PID_NAME = {
    PID_SSF: 'SSF', PID_SMF: 'SMF', PID_CRF: 'CRF',
    PID_UV:  'UV',  PID_ULF: 'ULF', PID_URF: 'URF',
    PID_WCNT:'WCNT',PID_WLF: 'WLF', PID_WRF: 'WRF',
    PID_FF:  'FF',  PID_UEF: 'UEF', PID_WEF: 'WEF',
    PID_HF:  'HF',  PID_C1F: 'C1F', PID_C2F: 'C2F',
    PID_HMF: 'HMF',
}


# ── 바이너리 프레임 생성 ───────────────────────
def make_frame(pid: int, data: bytes = b'') -> bytes:
    length = len(data)
    chk    = (pid + length + sum(data)) & 0xFF
    return bytes([SOF, pid, length]) + data + bytes([chk])

def make_flag_u8(pid: int, val: int) -> bytes:
    return make_frame(pid, bytes([val & 0xFF]))

def make_flag_u16(pid: int, val: int) -> bytes:
    return make_frame(pid, bytes([(val >> 8) & 0xFF, val & 0xFF]))


# ── 바이너리 수신 파서 ────────────────────────
class BinaryParser:
    """
    STM1 바이너리 스트림을 프레임 단위로 파싱
    SOF(0xAA) 기준 상태머신
    """
    WAIT_SOF  = 0
    WAIT_ID   = 1
    WAIT_LEN  = 2
    WAIT_DATA = 3
    WAIT_CHK  = 4

    def __init__(self):
        self.state  = self.WAIT_SOF
        self.pid    = 0
        self.length = 0
        self.data   = []
        self.frames = []

    def feed(self, byte: int):
        if self.state == self.WAIT_SOF:
            if byte == SOF:
                self.state = self.WAIT_ID

        elif self.state == self.WAIT_ID:
            self.pid   = byte
            self.state = self.WAIT_LEN

        elif self.state == self.WAIT_LEN:
            self.length = byte
            if self.length > MAX_DATA_LEN:
                self.state = self.WAIT_SOF
            elif self.length == 0:
                self.state = self.WAIT_CHK
            else:
                self.data  = []
                self.state = self.WAIT_DATA

        elif self.state == self.WAIT_DATA:
            self.data.append(byte)
            if len(self.data) >= self.length:
                self.state = self.WAIT_CHK

        elif self.state == self.WAIT_CHK:
            chk = (self.pid + self.length + sum(self.data)) & 0xFF
            if chk == byte:
                self.frames.append((self.pid, bytes(self.data)))
            self.state = self.WAIT_SOF

    def pop_frames(self):
        result      = self.frames[:]
        self.frames = []
        return result


def reconnect_serial():
    """Serial 연결 시도. 실패 시 None 반환 (루프에서 재시도)"""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
        try:
            ser.setDTR(False)  # STM32 리셋 방지
        except Exception:
            pass
        time.sleep(1.5)
        print('[Serial] STM1 연결 성공')
        return ser
    except Exception as e:
        print(f'[Serial] STM1 연결 실패: {e}')
        return None


class Pi1Node(Node):

    def __init__(self):
        super().__init__('pi1_node')
        self.ser    = None
        self.parser = BinaryParser()

        # ── Publisher ─────────────────────────
        self.pub_uart = self.create_publisher(
            String, '/pi1/uart_response', 10)
        self.pub_hb   = self.create_publisher(
            String, '/system/heartbeat', 10)

        # ── Subscriber ────────────────────────
        self.create_subscription(
            UInt8MultiArray, '/pi1/uart_cmd',
            self._on_uart_cmd, 10)

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info('Pi1 노드 시작')

    def _on_uart_cmd(self, msg: UInt8MultiArray):
        """PC 바이너리 명령 → STM1 UART 전달"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('UART 미연결')
            return
        try:
            raw = bytes(msg.data)
            self.ser.write(raw)
            self.ser.flush()
            self.get_logger().info(f'STM1 전달: {raw.hex()}')
        except Exception as e:
            self.get_logger().error(f'UART 전송 실패: {e}')
            self.ser = None

    def publish_stm_frame(self, pid: int, data: bytes):
        """
        STM1 프레임 → 문자열로 변환 → PC로 전송
        형식: STM1:PC:STATE:HOMING / STM1:PC:DONE:CYCLE1 등
        """
        msg = String()

        if pid == PID_STATE:
            val      = data[0] if data else 0
            msg.data = f'STM1:PC:STATE:{STATE_STR.get(val, f"UNKNOWN_{val:02X}")}'

        elif pid == PID_DONE:
            val      = data[0] if data else 0
            msg.data = f'STM1:PC:DONE:{DONE_STR.get(val, f"UNKNOWN_{val:02X}")}'

        elif pid == PID_ERR:
            val      = data[0] if data else 0
            msg.data = f'STM1:PC:ERR:{ERR_STR.get(val, f"UNKNOWN_{val:02X}")}'

        #Pi1이 STM1 프레임을 해석해서 사람이 읽는 문자열로 바꾸는 중계 역할
        elif pid in PID_NAME:
            name = PID_NAME[pid]
            if pid in (PID_UV, PID_WCNT) and len(data) >= 2:
                val = (data[0] << 8) | data[1]
            else:
                val = data[0] if data else 0
            msg.data = f'STM1:PC:FLAG:{name}:{val}'

        else:
            msg.data = f'STM1:PC:RAW:{pid:02X}:{data.hex()}'

        self.pub_uart.publish(msg)
        self.get_logger().info(f'STM1→PC: {msg.data}')

    def _heartbeat(self):
        if self.ser is None or not self.ser.is_open:
            return

        msg = String()
        msg.data = 'pi1'
        self.pub_hb.publish(msg)


def uart_rx_loop(node: Pi1Node):
    """STM1 수신 루프 — 바이트 파싱 후 PC로 publish"""
    while rclpy.ok():
        if node.ser is None or not node.ser.is_open:
            node.ser    = reconnect_serial()
            node.parser = BinaryParser()
            time.sleep(3.0)
            continue
        try:
            if node.ser.in_waiting > 0:
                raw = node.ser.read(node.ser.in_waiting)
                for byte in raw:
                    node.parser.feed(byte)
                for pid, data in node.parser.pop_frames():
                    node.publish_stm_frame(pid, data)
        except OSError:
            try:
                node.ser.close()
            except Exception:
                pass
            node.ser = None
        except serial.SerialException:
            node.ser = None
        time.sleep(0.02)


def camera_stream_loop(cam_index: int, stream_port: int, label: str, send_interval: float = CONVEYOR_SEND_INTERVAL):
    """카메라 영상 → PC TCP 스트리밍"""
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    
    if not cap.isOpened():  
        print(f'[{label}] 카메라 열기 실패 index={cam_index}')
        return

    frame_count = 0
    
    while rclpy.ok():
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.connect((PC_IP, stream_port))
            print(f'[{label}] PC 연결: {PC_IP}:{stream_port}')

            last_send_ts = 0.0

            while rclpy.ok():
                now = time.time()
                if now - last_send_ts < send_interval:
                    time.sleep(0.005)
                    # 전송 주기가 아닐 때도 카메라 버퍼는 계속 비움
                    cap.grab()
                    continue

                # 전송할 시점에는 오래된 버퍼를 몇 장 버리고 최신 프레임만 가져옴
                for _ in range(3):
                    cap.grab()

                ok, frame = cap.retrieve()
                if not ok or frame is None:
                    continue
                last_send_ts = now

                ok, jpeg = cv2.imencode(
                    '.jpg',
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
                )
                if not ok:
                    continue

                data = jpeg.tobytes()
                sock.sendall(struct.pack('>I', len(data)) + data)

        except Exception as e:
            print(f'[{label}] 스트림 오류: {e}')
            time.sleep(1.0)

        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass

    cap.release()


def main():
    rclpy.init()
    node     = Pi1Node()
    node.ser = reconnect_serial()

    threading.Thread(
        target=rclpy.spin,
        args=(node,), daemon=True).start()
    threading.Thread(
        target=uart_rx_loop,
        args=(node,), daemon=True).start()
    
    threading.Thread(
        target=camera_stream_loop,
        args=(CAM_INDEX, STREAM_PORT, 'CAM1_CONVEYOR'),
        daemon=True
    ).start()

    threading.Thread(
        target=camera_stream_loop,
        args=(NURSERY_LEFT_CAM_INDEX, NURSERY_LEFT_STREAM_PORT, 'CAM_NURSERY_LEFT', NURSERY_SEND_INTERVAL),
        daemon=True
    ).start()

    threading.Thread(
        target=camera_stream_loop,
        args=(NURSERY_RIGHT_CAM_INDEX, NURSERY_RIGHT_STREAM_PORT, 'CAM_NURSERY_RIGHT', NURSERY_SEND_INTERVAL),
        daemon=True
    ).start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser and node.ser.is_open:
                node.ser.close()
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