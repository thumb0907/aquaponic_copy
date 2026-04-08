#!/usr/bin/env python3
"""
pi2_node.py  ─  라파2 노드 (ROS2)
=========================================================
PC ←(ROS2)→ Pi2 ←(UART 바이너리)→ STM32 #2

[프레임 구조]  SOF(1) | ID(1) | LEN(1) | DATA(LEN) | CHK(1)
  SOF = 0xAA
  CHK = (ID + LEN + sum(DATA)) & 0xFF
  ※ 스카라 Serial.h / STM1 comm.h 와 완전히 동일

물리 연결 (Pi2 ↔ STM2):
  Pi2 /dev/ttyACM0 ─── STM32 USB (또는 ttyUSB0)

[ROS2 토픽]
  구독: /pi2/uart_cmd       master_node → STM2 명령 (바이너리)
        /pi2/interpi_send   master_node → Pi1 중계 요청
        /interpi/pi1_to_pi2 Pi1에서 오는 메시지
  발행: /pi2/uart_response  STM2 수신 내용 → master_node
        /system/heartbeat   1초 생존 신호 ('pi2')
        /interpi/pi2_to_pi1 Pi1으로 보내는 메시지
        /pi2/interpi_rx     Pi1 수신 내용 → master_node 알림
        /pi2/flag_update    STM2 플래그 변경 → master_node
"""
from __future__ import annotations

import time
import threading

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


# ── 설정 ──────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'   # STM2 연결 포트 (실제 포트로 수정)
BAUD        = 115200

# ── 프로토콜 상수 (comm.h / Serial.h 와 동일) ──
SOF          = 0xAA
MAX_DATA_LEN = 16

# 플래그 ID
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

# STM2 → 라파2 상태 보고 ID
PID_STATE = 0x20
PID_DONE  = 0x21
PID_ERR   = 0x22

# STATE 값 → 문자열
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

# PID → 플래그 이름
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


# ── 바이너리 수신 파서 (pi1과 동일) ───────────
class BinaryParser:
    """
    STM2에서 오는 바이너리 스트림을 프레임 단위로 파싱
    SOF(0xAA) 기준으로 상태머신 동작
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
                self.state = self.WAIT_SOF   # 이상한 값 → 버림
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
    """
    Serial 연결 재시도 (pi1과 동일한 구조)
    3초 대기 후 재시도
    """
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
        try:
            ser.setDTR(False)   # STM32 리셋 방지
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
        # STM2 상태 → PC
        self.pub_uart    = self.create_publisher(
            String, '/pi2/uart_response', 10)
        # 생존 신호
        self.pub_hb      = self.create_publisher(
            String, '/system/heartbeat', 10)
        # Pi1으로 메시지 전달
        self.pub_to_pi1  = self.create_publisher(
            String, '/interpi/pi2_to_pi1', 10)
        # master_node에 Pi1 수신 알림
        self.pub_link    = self.create_publisher(
            String, '/pi2/interpi_rx', 10)
        # 플래그 업데이트 → master_node
        self.pub_flag    = self.create_publisher(
            String, '/pi2/flag_update', 10)

        # ── Subscriber ────────────────────────
        # PC → STM2 명령 (바이너리)
        self.create_subscription(
            UInt8MultiArray, '/pi2/uart_cmd',
            self._on_uart_cmd, 10)
        # master_node → Pi1 중계
        self.create_subscription(
            String, '/pi2/interpi_send',
            self._on_interpi_send, 10)
        # Pi1에서 오는 메시지
        self.create_subscription(
            String, '/interpi/pi1_to_pi2',
            self._on_from_pi1, 10)

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info('Pi2 노드 시작')

    def _on_uart_cmd(self, msg: UInt8MultiArray):
        """
        PC가 보낸 바이너리 명령을 STM2에 그대로 전달
        pi1_node._on_uart_cmd()와 동일한 구조
        """
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
        STM2에서 온 프레임을 파싱해서 PC로 전송
        pi1_node.publish_stm_frame()과 동일한 구조
        STM1→STM2 구분을 위해 prefix를 STM2:PC로 변경
        """
        msg = String()

        if pid == PID_STATE:
            val       = data[0] if data else 0
            state_str = STATE_STR.get(val, f'UNKNOWN_{val:02X}')
            msg.data  = f'STM2:PC:STATE:{state_str}'

        elif pid == PID_DONE:
            val      = data[0] if data else 0
            done_str = DONE_STR.get(val, f'UNKNOWN_{val:02X}')
            msg.data = f'STM2:PC:DONE:{done_str}'

        elif pid == PID_ERR:
            val     = data[0] if data else 0
            err_str = ERR_STR.get(val, f'UNKNOWN_{val:02X}')
            msg.data = f'STM2:PC:ERR:{err_str}'

        elif pid in PID_NAME:
            name = PID_NAME[pid]
            if pid == PID_UV:
                val = (data[0] << 8 | data[1]) if len(data) >= 2 else 0
            else:
                val = data[0] if data else 0
            msg.data = f'STM2:PC:FLAG:{name}:{val}'

            # 플래그 변경을 master_node에 별도로 알림
            flag_msg      = String()
            flag_msg.data = f'FLAG:{name}:{val}'
            self.pub_flag.publish(flag_msg)

        else:
            msg.data = f'STM2:PC:RAW:{pid:02X}:{data.hex()}'

        self.pub_uart.publish(msg)
        self.get_logger().info(f'STM2→PC: {msg.data}')

    def _on_interpi_send(self, msg: String):
        """PC 요청으로 Pi1에 메시지 전달"""
        payload = msg.data.strip()
        if not payload:
            return
        out      = String()
        out.data = payload
        self.pub_to_pi1.publish(out)
        self.get_logger().info(f'Pi2→Pi1: {payload}')

    def _on_from_pi1(self, msg: String):
        """
        Pi1에서 온 메시지 수신
        SSF:1, CRF:1 같은 플래그 명령이면 STM2로 바이너리 전달
        """
        payload = msg.data.strip()
        if not payload:
            return

        # master_node에 수신 알림
        out      = String()
        out.data = payload
        self.pub_link.publish(out)
        self.get_logger().info(f'Pi1→Pi2 수신: {payload}')

        # 플래그 명령이면 STM2로 바이너리 전달
        # 형식: "SSF:1", "CRF:1" 등
        self._relay_to_stm2(payload)

    def _relay_to_stm2(self, payload: str):
        """
        Pi1에서 온 플래그 명령을 STM2에 바이너리로 전달
        형식: "SSF:1" → PID_SSF, val=1 → 바이너리 프레임
        """
        if self.ser is None or not self.ser.is_open:
            return

        # 플래그 이름 → PID 역변환
        NAME_TO_PID = {v: k for k, v in PID_NAME.items()}

        parts = payload.split(':')
        if len(parts) != 2:
            return

        name = parts[0].upper()
        try:
            val = int(parts[1])
        except ValueError:
            return

        pid = NAME_TO_PID.get(name)
        if pid is None:
            return

        # 바이너리 프레임 생성 후 STM2로 전송
        if pid == PID_UV:
            # UV는 2바이트
            frame = bytes([SOF, pid, 2,
                           (val >> 8) & 0xFF, val & 0xFF,
                           (pid + 2 + ((val >> 8) & 0xFF) + (val & 0xFF)) & 0xFF])
        else:
            chk   = (pid + 1 + val) & 0xFF
            frame = bytes([SOF, pid, 1, val & 0xFF, chk])

        try:
            self.ser.write(frame)
            self.ser.flush()
            self.get_logger().info(
                f'Pi1→STM2 전달: {name}={val} ({frame.hex()})')
        except Exception as e:
            self.get_logger().error(f'STM2 전달 실패: {e}')
            self.ser = None

    def _heartbeat(self):
        msg      = String()
        msg.data = 'pi2'
        self.pub_hb.publish(msg)


def uart_rx_loop(node: Pi2Node):
    """
    STM2 UART 수신 루프
    pi1_node.uart_rx_loop()과 동일한 구조
    3초 대기 후 재시도
    """
    while rclpy.ok():
        if node.ser is None or not node.ser.is_open:
            node.ser    = reconnect_serial()
            node.parser = BinaryParser()   # 파서 초기화
            time.sleep(3.0)                # pi1과 동일한 대기시간
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