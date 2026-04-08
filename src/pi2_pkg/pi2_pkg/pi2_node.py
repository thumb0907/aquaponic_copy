#!/usr/bin/env python3
"""
pi2_node.py  ─  라파2 노드 (ROS2)
=========================================================
PC ←(ROS2)→ Pi2 ←(UART 바이너리)→ STM32 #2 / 스카라

[프레임 구조]  SOF(1) | ID(1) | LEN(1) | DATA(LEN) | CHK(1)
  SOF = 0xAA
  CHK = (ID + LEN + sum(DATA)) & 0xFF

[ROS2 토픽]
  구독: /pi2/uart_cmd       PC → STM2 명령 (바이너리)
  발행: /pi2/uart_response  STM2 수신 내용 → PC
        /pi2/flag_update    STM2 플래그 변경 → PC
        /system/heartbeat   1초 생존 신호 ('pi2')

역할:
  1. PC 명령(바이너리) → STM2 UART 전달
  2. STM2 응답(바이너리) → 파싱 → PC로 ROS2 전송
  3. 플래그 변경 시 PC에 별도 알림
"""
from __future__ import annotations

import time
import threading

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


# ── 설정 ──────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'
BAUD        = 115200

# ── 프로토콜 상수 (comm.h / Serial.h 와 동일) ──
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

PID_STATE = 0x20
PID_DONE  = 0x21
PID_ERR   = 0x22

STATE_STR = {
    0x01: 'IDLE',
    0x02: 'CONVEY_RUN',
    0x03: 'IR_DETECTED',
    0x04: 'Z_FIX',
    0x05: 'HARVESTING',
    0x06: 'EJECTING',
    0x07: 'ESTOP',
}
DONE_STR = {
    0x01: 'CYCLE2',
    0x02: 'RESET_DONE',
}
ERR_STR = {
    0x01: 'TIMEOUT',
    0x02: 'SENSOR_FAIL',
}
PID_NAME = {
    PID_SSF: 'SSF', PID_SMF: 'SMF', PID_CRF: 'CRF',
    PID_UV:  'UV',  PID_ULF: 'ULF', PID_URF: 'URF',
    PID_WCNT:'WCNT',PID_WLF: 'WLF', PID_WRF: 'WRF',
    PID_FF:  'FF',  PID_UEF: 'UEF', PID_WEF: 'WEF',
    PID_HF:  'HF',  PID_C1F: 'C1F', PID_C2F: 'C2F',
}


# ── 바이너리 수신 파서 (pi1과 동일) ───────────
class BinaryParser:
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
            ser.setDTR(False)
        except Exception:
            pass
        time.sleep(1.5)
        print('[Serial] STM2 연결 성공')
        return ser
    except Exception as e:
        print(f'[Serial] STM2 연결 실패: {e}')
        return None


class Pi2Node(Node):

    def __init__(self):
        super().__init__('pi2_node')
        self.ser    = None
        self.parser = BinaryParser()

        # ── Publisher ─────────────────────────
        self.pub_uart = self.create_publisher(
            String, '/pi2/uart_response', 10)
        self.pub_hb   = self.create_publisher(
            String, '/system/heartbeat', 10)
        # 플래그 변경 → PC 별도 알림
        self.pub_flag = self.create_publisher(
            String, '/pi2/flag_update', 10)

        # ── Subscriber ────────────────────────
        self.create_subscription(
            UInt8MultiArray, '/pi2/uart_cmd',
            self._on_uart_cmd, 10)

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info('Pi2 노드 시작')

    def _on_uart_cmd(self, msg: UInt8MultiArray):
        """PC 바이너리 명령 → STM2 UART 전달"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('UART 미연결')
            return
        try:
            raw = bytes(msg.data)
            self.ser.write(raw)
            self.ser.flush()
            self.get_logger().info(f'STM2 전달: {raw.hex()}')
        except Exception as e:
            self.get_logger().error(f'UART 전송 실패: {e}')
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

    def publish_stm_frame(self, pid: int, data: bytes):
        """
        STM2 프레임 → 문자열 변환 → PC 전송
        플래그 변경 시 /pi2/flag_update 에도 별도 publish
        """
        msg = String()

        if pid == PID_STATE:
            val      = data[0] if data else 0
            msg.data = f'STM2:PC:STATE:{STATE_STR.get(val, f"UNKNOWN_{val:02X}")}'

        elif pid == PID_DONE:
            val      = data[0] if data else 0
            msg.data = f'STM2:PC:DONE:{DONE_STR.get(val, f"UNKNOWN_{val:02X}")}'

        elif pid == PID_ERR:
            val      = data[0] if data else 0
            msg.data = f'STM2:PC:ERR:{ERR_STR.get(val, f"UNKNOWN_{val:02X}")}'

        elif pid in PID_NAME:
            name = PID_NAME[pid]
            val  = (data[0] << 8 | data[1]) if pid == PID_UV and len(data) >= 2 \
                   else (data[0] if data else 0)
            msg.data = f'STM2:PC:FLAG:{name}:{val}'

            # 플래그 변경 → PC에 별도 알림 (master_node가 flags 딕셔너리 업데이트)
            flag_msg      = String()
            flag_msg.data = f'FLAG:{name}:{val}'
            self.pub_flag.publish(flag_msg)

        else:
            msg.data = f'STM2:PC:RAW:{pid:02X}:{data.hex()}'

        self.pub_uart.publish(msg)
        self.get_logger().info(f'STM2→PC: {msg.data}')

    def _heartbeat(self):
        msg      = String()
        msg.data = 'pi2'
        self.pub_hb.publish(msg)


def uart_rx_loop(node: Pi2Node):
    """STM2 수신 루프 — 바이트 파싱 후 PC로 publish"""
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


def main():
    rclpy.init()
    node     = Pi2Node()
    node.ser = reconnect_serial()

    threading.Thread(
        target=rclpy.spin,
        args=(node,), daemon=True).start()
    threading.Thread(
        target=uart_rx_loop,
        args=(node,), daemon=True).start()

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