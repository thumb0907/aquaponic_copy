#!/usr/bin/env python3
"""
pi1_node.py  ─  라파1 노드 (ROS2)
=========================================================
PC ←(ROS2)→ Pi1 ←(UART 바이너리)→ STM32 #1

[프레임 구조]  SOF(1) | ID(1) | LEN(1) | DATA(LEN) | CHK(1)
  SOF = 0xAA
  CHK = (ID + LEN + sum(DATA)) & 0xFF

[ROS2 토픽]
  구독: /pi1/uart_cmd      master_node → STM1 명령 (문자열 커맨드명)
        /pi1/interpi_send  master_node → Pi2 중계 요청
        /interpi/pi2_to_pi1 Pi2에서 오는 메시지
  발행: /pi1/uart_response STM1 수신 내용 → master_node (문자열)
        /system/heartbeat  1초 생존 신호
        /interpi/pi1_to_pi2 Pi2로 보내는 메시지
        /pi1/interpi_rx    Pi2 수신 내용 → master_node 알림

역할:
  1. 카메라 영상 → PC로 TCP 스트리밍
  2. PC 명령(바이너리) → STM1 UART 전달
  3. STM1 응답(바이너리) → 파싱 → PC로 ROS2 전송
  4. 라파2와 플래그 중계 (ROS2)

통신 흐름:
  PC ←(ROS2)→ 라파1 ←(UART)→ STM1
  PC ←(ROS2)→ 라파1 ←(ROS2)→ 라파2
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
from std_msgs.msg import String, ByteMultiArray, UInt8MultiArray


# ── 설정 ──────────────────────────────────────
PC_IP       = '192.168.0.183'
STREAM_PORT = 5000
SERIAL_PORT = '/dev/ttyACM0'
BAUD        = 115200
CAM_INDEX   = 0
CAM_WIDTH   = 640
CAM_HEIGHT  = 480

# ── 프로토콜 상수 (comm.h와 동일) ─────────────
SOF = 0xAA
MAX_DATA_LEN = 16

# 플래그 ID
PID_SSF  = 0x01
PID_SMF  = 0x02
PID_CRF  = 0x03
PID_UV   = 0x04
PID_ULF  = 0x05
PID_URF  = 0x06
PID_WCNT = 0x07
PID_WLF  = 0x08
PID_WRF  = 0x09
PID_FF   = 0x0A
PID_UEF  = 0x0B
PID_WEF  = 0x0C
PID_HF   = 0x0D
PID_C1F  = 0x0E
PID_C2F  = 0x0F
PID_ESTOP = 0x10
PID_RESET = 0x11

# STM1 → 라파1 상태 보고 ID
PID_STATE = 0x20
PID_DONE  = 0x21
PID_ERR   = 0x22

# STATE 값 → 문자열 변환 (ROS2로 보낼 때 사람이 읽기 편하게)
STATE_STR = {
    0x01: 'IDLE',
    0x02: 'HOMING',
    0x03: 'RUN_CONVEYOR1',
    0x04: 'SEEDING',
    0x05: 'EJECTING',
    0x06: 'WAIT_SCARA_PICK',
    0x07: 'ESTOP',
}
DONE_STR = {
    0x01: 'HOMING',
    0x02: 'CYCLE1',
}
ERR_STR = {
    0x01: 'HOMING_TIMEOUT',
}

# PID → 플래그 이름 변환
PID_NAME = {
    PID_SSF:  'SSF',
    PID_SMF:  'SMF',
    PID_CRF:  'CRF',
    PID_UV:   'UV',
    PID_ULF:  'ULF',
    PID_URF:  'URF',
    PID_WCNT: 'WCNT',
    PID_WLF:  'WLF',
    PID_WRF:  'WRF',
    PID_FF:   'FF',
    PID_UEF:  'UEF',
    PID_WEF:  'WEF',
    PID_HF:   'HF',
    PID_C1F:  'C1F',
    PID_C2F:  'C2F',
}


# ── 바이너리 프레임 생성 ───────────────────────
def make_frame(pid: int, data: bytes = b'') -> bytes:
    """
    SOF + ID + LEN + DATA + CHK 프레임 생성
    STM1의 send_frame()과 동일한 구조
    """
    length = len(data)
    chk = (pid + length + sum(data)) & 0xFF
    return bytes([SOF, pid, length]) + data + bytes([chk])


def make_flag_u8(pid: int, val: int) -> bytes:
    """1바이트 플래그 프레임 생성"""
    return make_frame(pid, bytes([val & 0xFF]))


def make_flag_u16(pid: int, val: int) -> bytes:
    """2바이트 플래그 프레임 생성 (UV용)"""
    return make_frame(pid, bytes([(val >> 8) & 0xFF, val & 0xFF]))


# ── 바이너리 수신 파서 ────────────────────────
class BinaryParser:
    """
    STM1에서 오는 바이너리 스트림을 프레임 단위로 파싱
    STM1의 process_byte()와 동일한 상태머신
    """
    WAIT_SOF  = 0
    WAIT_ID   = 1
    WAIT_LEN  = 2
    WAIT_DATA = 3
    WAIT_CHK  = 4

    def __init__(self):
        self.state = self.WAIT_SOF
        self.pid   = 0
        self.length = 0
        self.data  = []
        self.frames = []  # 완성된 프레임 저장

    def feed(self, byte: int):
        """바이트 하나씩 입력, 프레임 완성되면 frames에 추가"""
        if self.state == self.WAIT_SOF:
            if byte == SOF:
                self.state = self.WAIT_ID

        elif self.state == self.WAIT_ID:
            self.pid   = byte
            self.state = self.WAIT_LEN

        elif self.state == self.WAIT_LEN:
            self.length = byte
            if self.length > MAX_DATA_LEN:
                self.state = self.WAIT_SOF  # 이상한 값 → 버림
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
            # 체크섬 검증
            chk = (self.pid + self.length + sum(self.data)) & 0xFF
            if chk == byte:
                # 정상 프레임 → 저장
                self.frames.append((self.pid, bytes(self.data)))
            self.state = self.WAIT_SOF

    def pop_frames(self):
        """완성된 프레임 목록 반환 후 초기화"""
        result = self.frames[:]
        self.frames = []
        return result


def reconnect_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
        try:
            ser.setDTR(False)
        except Exception:
            pass
        time.sleep(1.5)
        print('[Serial] 연결 성공')
        return ser
    except Exception as e:
        print(f'[Serial] 실패: {e}')
        return None


class Pi1Node(Node):

    def __init__(self):
        super().__init__('pi1_node')
        self.ser    = None
        self.parser = BinaryParser()  # STM1 수신 파서

        # ── Publisher ─────────────────────────
        # STM1 상태 → PC
        self.pub_uart = self.create_publisher(
            String, '/pi1/uart_response', 10)
        # 생존 신호
        self.pub_hb = self.create_publisher(
            String, '/system/heartbeat', 10)
        # 라파2로 플래그 전달
        self.pub_to_pi2 = self.create_publisher(
            String, '/interpi/pi1_to_pi2', 10)
        # 마스터 확인용
        self.pub_link = self.create_publisher(
            String, '/pi1/interpi_rx', 10)

        # ── Subscriber ────────────────────────
        # PC → STM1 명령 (바이너리)
        self.create_subscription(
            UInt8MultiArray, '/pi1/uart_cmd',
            self._on_uart_cmd, 10)
        # PC → 라파2 중계 명령
        self.create_subscription(
            String, '/pi1/interpi_send',
            self._on_interpi_send, 10)
        # 라파2 → PC 중계
        self.create_subscription(
            String, '/interpi/pi2_to_pi1',
            self._on_from_pi2, 10)

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info('Pi1 노드 시작')

    def _on_uart_cmd(self, msg: UInt8MultiArray):
        """
        PC가 보낸 바이너리 명령을 STM1에 그대로 전달
        msg.data = [0xAA, 0x01, 0x01, 0x01, 0x03] 같은 바이트 배열
        """
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('UART 미연결')
            return
        try:
            raw = bytes(msg.data)
            self.ser.write(raw)
            self.ser.flush()
            self.get_logger().info(
                f'STM1 전달: {raw.hex()}')
        except Exception as e:
            self.get_logger().error(f'UART 전송 실패: {e}')
            self.ser = None

    def publish_stm_frame(self, pid: int, data: bytes):
        """
        STM1에서 온 프레임을 파싱해서 PC로 전송
        형식: "STM1:PC:STATE:HOMING" 같은 문자열로 변환
        """
        msg = String()

        if pid == PID_STATE:
            val = data[0] if data else 0
            state_str = STATE_STR.get(val, f'UNKNOWN_{val:02X}')
            msg.data = f'STM1:PC:STATE:{state_str}'

        elif pid == PID_DONE:
            val = data[0] if data else 0
            done_str = DONE_STR.get(val, f'UNKNOWN_{val:02X}')
            msg.data = f'STM1:PC:DONE:{done_str}'

        elif pid == PID_ERR:
            val = data[0] if data else 0
            err_str = ERR_STR.get(val, f'UNKNOWN_{val:02X}')
            msg.data = f'STM1:PC:ERR:{err_str}'

        elif pid in PID_NAME:
            # 플래그 업데이트 보고
            name = PID_NAME[pid]
            if pid == PID_UV:
                # UV는 2바이트
                val = (data[0] << 8 | data[1]) if len(data) >= 2 else 0
            else:
                val = data[0] if data else 0
            msg.data = f'STM1:PC:FLAG:{name}:{val}'

        else:
            msg.data = f'STM1:PC:RAW:{pid:02X}:{data.hex()}'

        self.pub_uart.publish(msg)
        self.get_logger().info(f'STM1→PC: {msg.data}')

    def _on_interpi_send(self, msg: String):
        """PC 요청으로 라파2에 메시지 전달"""
        payload = msg.data.strip()
        if not payload:
            return
        out = String()
        out.data = payload
        self.pub_to_pi2.publish(out)
        self.get_logger().info(f'Pi1→Pi2: {payload}')

    def _on_from_pi2(self, msg: String):
        """라파2에서 온 메시지를 PC로 전달"""
        payload = msg.data.strip()
        if not payload:
            return
        out = String()
        out.data = payload
        self.pub_link.publish(out)
        self.get_logger().info(f'Pi2→Pi1 수신: {payload}')

    def _heartbeat(self):
        msg = String()
        msg.data = 'pi1'
        self.pub_hb.publish(msg)


def uart_rx_loop(node: Pi1Node):
    """
    STM1 UART 수신 루프
    바이트를 받아서 BinaryParser에 넣고
    완성된 프레임을 PC로 publish
    """
    while rclpy.ok():
        if node.ser is None or not node.ser.is_open:
            #time.sleep(1.0)
            node.ser = reconnect_serial()
            node.parser = BinaryParser()  # 파서 초기화
            time.sleep(3.0)
            continue

        try:
            if node.ser.in_waiting > 0:
                raw = node.ser.read(node.ser.in_waiting)

                # 바이트 하나씩 파서에 입력
                for byte in raw:
                    node.parser.feed(byte)

                # 완성된 프레임 처리
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


def video_stream_loop():
    """카메라 영상을 PC로 TCP 스트리밍"""
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

    while rclpy.ok():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((PC_IP, STREAM_PORT))
            print(f'[Stream] PC 연결: {PC_IP}:{STREAM_PORT}')

            while rclpy.ok():
                ok, frame = cap.read()
                if not ok:
                    continue

                _, jpeg = cv2.imencode(
                    '.jpg', frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 80])
                data = jpeg.tobytes()
                size = struct.pack('>I', len(data))
                sock.sendall(size + data)

        except Exception as e:
            print(f'[Stream] 오류: {e} → 재연결')
        finally:
            try:
                sock.close()
            except Exception:
                pass
            time.sleep(1.0)

    cap.release()


def main():
    rclpy.init()
    node = Pi1Node()
    node.ser = reconnect_serial()

    threading.Thread(
        target=rclpy.spin,
        args=(node,), daemon=True).start()
    threading.Thread(
        target=uart_rx_loop,
        args=(node,), daemon=True).start()
    threading.Thread(
        target=video_stream_loop,
        daemon=True).start()

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