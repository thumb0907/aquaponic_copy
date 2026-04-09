#!/usr/bin/env python3
"""
pi2_node.py  ─  라파2 노드 (ROS2)
=========================================================
PC ←(ROS2)→ Pi2 ←(UART 바이너리)→ STM2 / 스카라(아두이노) / 매니퓰레이터

[프레임 구조]  SOF(1) | ID(1) | LEN(1) | DATA(LEN) | CHK(1)
  SOF = 0xAA
  CHK = (ID + LEN + sum(DATA)) & 0xFF

[ROS2 토픽]
  구독: /pi2/uart_cmd       PC → Pi2 명령 (바이너리, 앞에 대상 식별자 포함)
  발행: /pi2/uart_response  장치 수신 내용 → PC
        /pi2/flag_update    플래그 변경 → PC
        /system/heartbeat   1초 생존 신호 ('pi2')

[대상 식별자 — master_node의 _send_scara / _send_manip / _send_stm2 와 일치]
  0x01 = 스카라 (아두이노)
  0x02 = 매니퓰레이터
  0x03 = STM2 (두 번째 컨베이어)
  식별자 없음(4바이트 이하) = STM2로 fallback  ← _send_binary_pi2() 호환용

[포트 설정]
  udev rules로 고정 권장:
    /dev/scara  ← 스카라 아두이노
    /dev/stm2   ← 두 번째 컨베이어 STM32
    /dev/manip  ← 매니퓰레이터 (미연결 시 None으로 처리됨)
"""
from __future__ import annotations

import time
import threading

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


# ── 포트 설정 ──────────────────────────────────
# udev rules로 고정하지 않았으면 /dev/ttyACM0, /dev/ttyACM1 등으로 변경
PORT_SCARA = '/dev/scara'   # 스카라 (아두이노)
PORT_STM2  = '/dev/stm2'   # 두 번째 컨베이어 STM32
PORT_MANIP = '/dev/manip'  # 매니퓰레이터 (미연결이면 None으로 처리됨)
BAUD       = 115200

# ── 대상 식별자 (master_node와 동일하게 맞춰야 함) ──
TARGET_SCARA = 0x01
TARGET_MANIP = 0x02
TARGET_STM2  = 0x03

# ── 프로토콜 상수 ──────────────────────────────
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

# STM2 상태 코드 → 문자열
STM2_STATE_STR = {
    0x01: 'IDLE',
    0x02: 'CONVEY_RUN',
    0x03: 'IR_DETECTED',
    0x04: 'Z_FIX',
    0x05: 'HARVESTING',
    0x06: 'EJECTING',
    0x07: 'ESTOP',
}
# STM2 완료 코드 → 문자열 (master_node의 'CYCLE2' 비교와 반드시 일치해야 함)
STM2_DONE_STR = {
    0x01: 'CYCLE2',       # master_node: elif line == 'STM2:PC:DONE:CYCLE2'
    0x02: 'RESET_DONE',
}
STM2_ERR_STR = {
    0x01: 'TIMEOUT',
    0x02: 'SENSOR_FAIL',
}

# 스카라(아두이노) 상태 코드 → 문자열
SCARA_STATE_STR = {
    0x01: 'IDLE',
    0x02: 'MOVING',
    0x03: 'DONE',
    0x04: 'ESTOP',
}

PID_NAME = {
    PID_SSF: 'SSF', PID_SMF: 'SMF', PID_CRF: 'CRF',
    PID_UV:  'UV',  PID_ULF: 'ULF', PID_URF: 'URF',
    PID_WCNT:'WCNT',PID_WLF: 'WLF', PID_WRF: 'WRF',
    PID_FF:  'FF',  PID_UEF: 'UEF', PID_WEF: 'WEF',
    PID_HF:  'HF',  PID_C1F: 'C1F', PID_C2F: 'C2F',
}


# ── 바이너리 수신 파서 ────────────────────────
class BinaryParser:
    """
    바이너리 스트림 → 프레임 단위 파싱
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
                self.state = self.WAIT_SOF  # 비정상 길이 → 버림
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


# ── 시리얼 연결 헬퍼 ──────────────────────────
def try_connect(port: str, label: str):
    """
    시리얼 포트 연결 시도.
    성공 시 Serial 객체 반환, 실패 시 None 반환.
    """
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
        try:
            ser.setDTR(False)  # STM32/아두이노 리셋 방지
        except Exception:
            pass
        time.sleep(1.5)
        print(f'[Serial] {label} 연결 성공 ({port})')
        return ser
    except Exception as e:
        print(f'[Serial] {label} 연결 실패 ({port}): {e}')
        return None


class Pi2Node(Node):

    def __init__(self):
        super().__init__('pi2_node')

        # ── 장치별 시리얼 핸들 + 파서 ────────
        # 각 장치가 독립적인 포트와 파서를 가짐
        self.ser_stm2  = None
        self.ser_scara = None
        self.ser_manip = None   # 매니퓰레이터: 연결 안 됐으면 None 유지

        self.parser_stm2  = BinaryParser()
        self.parser_scara = BinaryParser()
        self.parser_manip = BinaryParser()

        # ── Publisher ─────────────────────────
        self.pub_uart = self.create_publisher(
            String, '/pi2/uart_response', 10)
        self.pub_hb   = self.create_publisher(
            String, '/system/heartbeat', 10)
        self.pub_flag = self.create_publisher(
            String, '/pi2/flag_update', 10)

        # ── Subscriber ────────────────────────
        # PC → Pi2 명령 수신 (앞 1바이트가 대상 식별자)
        self.create_subscription(
            UInt8MultiArray, '/pi2/uart_cmd',
            self._on_uart_cmd, 10)

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info('Pi2 노드 시작')

    # ══════════════════════════════════════════
    # 수신: PC → Pi2 명령 → 장치별 라우팅
    # ══════════════════════════════════════════
    def _on_uart_cmd(self, msg: UInt8MultiArray):
        """
        master_node에서 오는 명령을 장치별로 라우팅.

        [패킷 구조]
          [대상식별자(1)] [SOF(1)] [PID(1)] [LEN(1)] [DATA...] [CHK(1)]
          └ 0x01=스카라, 0x02=매니퓰레이터, 0x03=STM2

        식별자가 없으면(=SOF로 시작하면) STM2로 fallback
        → _send_binary_pi2() 호환용 (긴급정지·리셋 등)
        """
        raw = bytes(msg.data)
        if not raw:
            return

        # 첫 바이트가 SOF(0xAA)이면 식별자 없이 보낸 것 → STM2 fallback
        if raw[0] == SOF:
            self._write_to('STM2', self.ser_stm2, raw)
            return

        # 첫 바이트가 식별자
        target  = raw[0]
        payload = raw[1:]  # 식별자 제거 후 실제 프레임

        if target == TARGET_SCARA:
            self._write_to('SCARA', self.ser_scara, payload)

        elif target == TARGET_MANIP:
            self._write_to('MANIP', self.ser_manip, payload)

        elif target == TARGET_STM2:
            self._write_to('STM2', self.ser_stm2, payload)

        else:
            self.get_logger().warn(
                f'알 수 없는 대상 식별자: 0x{target:02X}')

    def _write_to(self, label: str, ser, payload: bytes):
        """지정 장치의 시리얼 포트로 바이트 전송"""
        if ser is None or not ser.is_open:
            self.get_logger().error(f'{label} 미연결 — 전송 불가')
            return
        try:
            ser.write(payload)
            ser.flush()
            self.get_logger().info(
                f'→ {label}: {payload.hex()}')
        except Exception as e:
            self.get_logger().error(f'{label} 전송 실패: {e}')

    # ══════════════════════════════════════════
    # 수신 프레임 → PC publish (STM2용)
    # ══════════════════════════════════════════
    def publish_stm2_frame(self, pid: int, data: bytes):
        """
        STM2 프레임 → 문자열 변환 → /pi2/uart_response publish
        플래그 프레임이면 /pi2/flag_update 에도 publish
        """
        msg = String()

        if pid == PID_STATE:
            val      = data[0] if data else 0
            msg.data = f'STM2:PC:STATE:{STM2_STATE_STR.get(val, f"UNKNOWN_{val:02X}")}'

        elif pid == PID_DONE:
            val      = data[0] if data else 0
            # master_node: 'STM2:PC:DONE:CYCLE2' 로 비교하므로 반드시 CYCLE2
            msg.data = f'STM2:PC:DONE:{STM2_DONE_STR.get(val, f"UNKNOWN_{val:02X}")}'

        elif pid == PID_ERR:
            val      = data[0] if data else 0
            msg.data = f'STM2:PC:ERR:{STM2_ERR_STR.get(val, f"UNKNOWN_{val:02X}")}'

        elif pid in PID_NAME:
            name = PID_NAME[pid]
            # UV 플래그는 2바이트
            val = (data[0] << 8 | data[1]) if (pid == PID_UV and len(data) >= 2) \
                  else (data[0] if data else 0)
            msg.data = f'STM2:PC:FLAG:{name}:{val}'

            # 플래그 변경 → PC 별도 알림
            flag_msg      = String()
            flag_msg.data = f'FLAG:{name}:{val}'
            self.pub_flag.publish(flag_msg)

        else:
            msg.data = f'STM2:PC:RAW:{pid:02X}:{data.hex()}'

        self.pub_uart.publish(msg)
        self.get_logger().info(f'STM2→PC: {msg.data}')

    # ══════════════════════════════════════════
    # 수신 프레임 → PC publish (스카라용)
    # ══════════════════════════════════════════
    def publish_scara_frame(self, pid: int, data: bytes):
        """
        스카라(아두이노) 프레임 → 문자열 변환 → /pi2/uart_response publish
        CRF=0 전달 등 master_node가 수신하는 'SCARA:PC:...' 형식
        """
        msg = String()

        if pid == PID_STATE:
            val      = data[0] if data else 0
            msg.data = f'SCARA:PC:STATE:{SCARA_STATE_STR.get(val, f"UNKNOWN_{val:02X}")}'

        elif pid in PID_NAME:
            name = PID_NAME[pid]
            val  = data[0] if data else 0
            msg.data = f'SCARA:PC:FLAG:{name}:{val}'

            # 플래그 변경 → PC 별도 알림
            flag_msg      = String()
            flag_msg.data = f'FLAG:{name}:{val}'
            self.pub_flag.publish(flag_msg)

        else:
            msg.data = f'SCARA:PC:RAW:{pid:02X}:{data.hex()}'

        self.pub_uart.publish(msg)
        self.get_logger().info(f'SCARA→PC: {msg.data}')

    # ══════════════════════════════════════════
    # Heartbeat
    # ══════════════════════════════════════════
    def _heartbeat(self):
        msg      = String()
        msg.data = 'pi2'
        self.pub_hb.publish(msg)


# ══════════════════════════════════════════════
# 장치별 수신 루프 (각각 별도 스레드로 실행)
# ══════════════════════════════════════════════

def stm2_rx_loop(node: Pi2Node):
    """STM2 수신 루프"""
    while rclpy.ok():
        # 연결 끊겼으면 재시도
        if node.ser_stm2 is None or not node.ser_stm2.is_open:
            node.ser_stm2   = try_connect(PORT_STM2, 'STM2')
            node.parser_stm2 = BinaryParser()
            time.sleep(3.0)
            continue
        try:
            if node.ser_stm2.in_waiting > 0:
                raw = node.ser_stm2.read(node.ser_stm2.in_waiting)
                for byte in raw:
                    node.parser_stm2.feed(byte)
                for pid, data in node.parser_stm2.pop_frames():
                    node.publish_stm2_frame(pid, data)
        except OSError:
            _safe_close(node.ser_stm2)
            node.ser_stm2 = None
        except serial.SerialException:
            node.ser_stm2 = None
        time.sleep(0.02)


def scara_rx_loop(node: Pi2Node):
    """스카라(아두이노) 수신 루프"""
    while rclpy.ok():
        if node.ser_scara is None or not node.ser_scara.is_open:
            node.ser_scara    = try_connect(PORT_SCARA, 'SCARA')
            node.parser_scara = BinaryParser()
            time.sleep(3.0)
            continue
        try:
            if node.ser_scara.in_waiting > 0:
                raw = node.ser_scara.read(node.ser_scara.in_waiting)
                for byte in raw:
                    node.parser_scara.feed(byte)
                for pid, data in node.parser_scara.pop_frames():
                    node.publish_scara_frame(pid, data)
        except OSError:
            _safe_close(node.ser_scara)
            node.ser_scara = None
        except serial.SerialException:
            node.ser_scara = None
        time.sleep(0.02)


def manip_rx_loop(node: Pi2Node):
    """
    매니퓰레이터 수신 루프.
    포트 없으면 계속 재시도하며 대기 (연결되면 자동 동작)
    """
    while rclpy.ok():
        if node.ser_manip is None or not node.ser_manip.is_open:
            node.ser_manip    = try_connect(PORT_MANIP, 'MANIP')
            node.parser_manip = BinaryParser()
            # 매니퓰레이터는 없을 수도 있으므로 재시도 간격 길게
            time.sleep(5.0)
            continue
        try:
            if node.ser_manip.in_waiting > 0:
                raw = node.ser_manip.read(node.ser_manip.in_waiting)
                for byte in raw:
                    node.parser_manip.feed(byte)
                # 현재 매니퓰레이터 수신 처리 미구현 → 추후 publish_manip_frame 추가
        except OSError:
            _safe_close(node.ser_manip)
            node.ser_manip = None
        except serial.SerialException:
            node.ser_manip = None
        time.sleep(0.02)


def _safe_close(ser):
    """시리얼 포트 안전하게 닫기"""
    try:
        if ser is not None:
            ser.close()
    except Exception:
        pass


def main():
    rclpy.init()
    node = Pi2Node()

    # 초기 연결 시도
    node.ser_stm2  = try_connect(PORT_STM2,  'STM2')
    node.ser_scara = try_connect(PORT_SCARA, 'SCARA')
    # 매니퓰레이터는 미연결 상태면 None 유지 (루프에서 재시도)
    node.ser_manip = try_connect(PORT_MANIP, 'MANIP')

    # ROS2 spin + 장치별 수신 루프를 각각 스레드로 실행
    threading.Thread(target=rclpy.spin,    args=(node,), daemon=True).start()
    threading.Thread(target=stm2_rx_loop,  args=(node,), daemon=True).start()
    threading.Thread(target=scara_rx_loop, args=(node,), daemon=True).start()
    threading.Thread(target=manip_rx_loop, args=(node,), daemon=True).start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        _safe_close(node.ser_stm2)
        _safe_close(node.ser_scara)
        _safe_close(node.ser_manip)
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